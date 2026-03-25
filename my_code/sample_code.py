#!/usr/bin/env python3

# ============================================================
# robotics assignment 3
#
# this script creates a ros2 teleoperation interface for the
# turtlebot3 + openmanipulatorx platform.
#
# features:
# 1. keyboard-based mobile base control
# 2. keyboard-based gripper open / close
# 3. keyboard-based preset arm positions
# 4. live terminal display for:
#    - linear velocity
#    - angular velocity
#    - arm joint angles
#    - gripper state
#
# intended keys:
# w : increase forward velocity
# x : decrease forward velocity / go backward
# a : increase angular velocity left
# d : decrease angular velocity / turn right
# s : stop base
# g : open gripper
# h : close gripper
# 0 : extend forward pose
# 9 : home pose
# 8 : wave pose
# q : quit program
# ============================================================

import sys
import tty
import termios
import select

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState


# ============================================================
# base velocity tuning values
#
# UPDATED: increased to overcome motor deadband/static friction
# ============================================================
linVelStepSize = 0.05
angVelStepSize = 0.20


# ============================================================
# safety limits for the mobile base
# ============================================================
maxLinearVel = 0.20
minLinearVel = -0.20
maxAngularVel = 1.50
minAngularVel = -1.50


# ============================================================
# gripper command mapping
# ============================================================
gripperKeyBindings = {
    'g': 0.01,
    'h': -0.01
}


# ============================================================
# preset arm poses
# ============================================================
poses = {
    '9': [0.0, 0.0, 0.0, 0.0],         # home pose
    '0': [0.0, -1.10, 0.75, 0.35],     # extend forward
    '8': [0.8, -0.65, 0.30, 0.85]      # wave / custom pose
}


def getKey(settings):
    """
    read one key from the keyboard without waiting for enter.
    """
    tty.setraw(sys.stdin.fileno())
    readable, _, _ = select.select([sys.stdin], [], [], 0.05)

    if readable:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def clamp(value, low, high):
    """
    limit a numeric value to stay inside [low, high].
    """
    return max(low, min(value, high))


class SimpleDemoController(Node):
    """
    main ros2 node for the assignment.
    """

    def __init__(self):
        super().__init__('simple_demo_controller')

        self.armJointNames = ['joint1', 'joint2', 'joint3', 'joint4']

        self.targetLinearVel = 0.0
        self.targetAngularVel = 0.0

        self.currentJ1 = 0.0
        self.currentJ2 = 0.0
        self.currentJ3 = 0.0
        self.currentJ4 = 0.0

        self.currentGripper = 0.0
        self.lastGripperCommand = 0.0

        self.armActionClient = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )

        self.gripperActionClient = ActionClient(
            self,
            GripperCommand,
            '/gripper_controller/gripper_cmd'
        )

        # ----------------------------------------------------
        # publisher for mobile base movement
        #
        # UPDATED: Removed leading slash to make topic relative
        # ----------------------------------------------------
        self.cmdVelPub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.jointStateSub = self.create_subscription(
            JointState,
            '/joint_states',
            self.jointStateCallback,
            10
        )

        self.settings = termios.tcgetattr(sys.stdin)

        self.timer = self.create_timer(0.1, self.runLoop)

        self.printInstructions()
        self.printStatus()

    def jointStateCallback(self, msg):
        if 'joint1' in msg.name:
            idx = msg.name.index('joint1')
            self.currentJ1 = msg.position[idx]

        if 'joint2' in msg.name:
            idx = msg.name.index('joint2')
            self.currentJ2 = msg.position[idx]

        if 'joint3' in msg.name:
            idx = msg.name.index('joint3')
            self.currentJ3 = msg.position[idx]

        if 'joint4' in msg.name:
            idx = msg.name.index('joint4')
            self.currentJ4 = msg.position[idx]

        possibleGripperNames = [
            'gripper',
            'gripper_left_joint',
            'gripper_right_joint',
            'gripper_sub_joint',
            'joint5'
        ]

        for name in possibleGripperNames:
            if name in msg.name:
                idx = msg.name.index(name)
                self.currentGripper = msg.position[idx]
                break

    def runLoop(self):
        # ----------------------------------------------------
        # UPDATED: Forced key input to lowercase immediately
        # ----------------------------------------------------
        key = getKey(self.settings).lower()

        # ----------------------------------------------------
        # mobile base controls
        # ----------------------------------------------------
        if key == 'w':
            self.targetLinearVel = clamp(
                self.targetLinearVel + linVelStepSize,
                minLinearVel,
                maxLinearVel
            )

        elif key == 'x':
            self.targetLinearVel = clamp(
                self.targetLinearVel - linVelStepSize,
                minLinearVel,
                maxLinearVel
            )

        elif key == 'a':
            self.targetAngularVel = clamp(
                self.targetAngularVel + angVelStepSize,
                minAngularVel,
                maxAngularVel
            )

        elif key == 'd':
            self.targetAngularVel = clamp(
                self.targetAngularVel - angVelStepSize,
                minAngularVel,
                maxAngularVel
            )

        elif key == 's':
            self.targetLinearVel = 0.0
            self.targetAngularVel = 0.0

        # ----------------------------------------------------
        # gripper controls
        # ----------------------------------------------------
        elif key in gripperKeyBindings:
            self.lastGripperCommand = gripperKeyBindings[key]
            self.sendGripperGoal(self.lastGripperCommand)

        # ----------------------------------------------------
        # arm preset controls
        # ----------------------------------------------------
        elif key in poses:
            self.sendArmGoal(poses[key], 2.0)

        # ----------------------------------------------------
        # quit
        # ----------------------------------------------------
        elif key == 'q':
            self.stopRobot()
            self.destroy_node()
            rclpy.shutdown()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            sys.stdout.write('\nExiting...\n')
            sys.exit(0)

        # publish current base command every loop
        self.publishBaseCommand()

        # refresh terminal display
        self.printStatus()

    def publishBaseCommand(self):
        twist = Twist()
        twist.linear.x = self.targetLinearVel
        twist.angular.z = self.targetAngularVel
        self.cmdVelPub.publish(twist)

    def stopRobot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmdVelPub.publish(twist)

    def sendArmGoal(self, positions, durationSec):
        if not self.armActionClient.server_is_ready():
            self.get_logger().info('Arm action server not available')
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.armJointNames

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(
            sec=int(durationSec),
            nanosec=int((durationSec % 1) * 1e9)
        )

        goal.trajectory.points.append(point)
        self.armActionClient.send_goal_async(goal)

    def sendGripperGoal(self, position):
        if not self.gripperActionClient.server_is_ready():
            self.get_logger().info('Gripper action server not available')
            return

        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 1.0

        self.gripperActionClient.send_goal_async(goal)

    def printStatus(self):
        sys.stdout.write('\r' + ' ' * 170 + '\r')

        statusString = (
            f"Present Linear Velocity: {self.targetLinearVel:.3f}, "
            f"Angular Velocity: {self.targetAngularVel:.3f}\n"
            f"Present Arm Joint Angle "
            f"J1: {self.currentJ1:.3f} "
            f"J2: {self.currentJ2:.3f} "
            f"J3: {self.currentJ3:.3f} "
            f"J4: {self.currentJ4:.3f}\n"
            f"Present Gripper Status: cmd={self.lastGripperCommand:.3f} "
            f"state={self.currentGripper:.3f}\n"
            f"---------------------------\n"
        )

        sys.stdout.write(statusString + "\033[4A")
        sys.stdout.flush()

    def printInstructions(self):
        print("""
---------------------------
 Teleoperation Control of TurtleBot3 + OpenManipulatorX
 ---------------------------
 Base
 w : increase linear velocity
 x : decrease linear velocity
 a : increase angular velocity
 d : decrease angular velocity
 s : base stop

 Gripper
 g : gripper open
 h : gripper close

 Arm Preset
 0 : Extend Forward
 9 : Home pose
 8 : Wave pose

 q to quit
 ---------------------------
""")


def main(args=None):
    rclpy.init(args=args)
    node = SimpleDemoController()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.stopRobot()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
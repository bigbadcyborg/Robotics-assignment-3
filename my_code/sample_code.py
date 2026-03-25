#!/usr/bin/env python3

# Sample Code for Robotics_Assignment_3
# Modified to satisfy milestone 3 requirements

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


linVelStepSize = 0.01
angVelStepSize = 0.1

gripperKeyBindings = {
    'g': 0.01,   # open
    'h': -0.01   # close
}

# These are example presets.
# You should test and adjust them on your robot if needed.
poses = {
    '9': [0.0, 0.0, 0.0, 0.0],         # Home pose
    '0': [0.0, -1.10, 0.75, 0.35],     # Extend Forward
    '8': [0.8, -0.65, 0.30, 0.85]      # Wave / custom pose
}


def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    readable, _, _ = select.select([sys.stdin], [], [], 0.1)

    if readable:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class SimpleDemoController(Node):
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

        self.cmdVelPub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.jointStateSub = self.create_subscription(
            JointState,
            'joint_states',
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

        # Some setups expose the gripper joint name differently.
        # Try a few common possibilities.
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
        key = getKey(self.settings)

        if not key:
            return

        if key == 'w':
            self.targetLinearVel += linVelStepSize

        elif key == 'x':
            self.targetLinearVel -= linVelStepSize

        elif key == 'a':
            self.targetAngularVel += angVelStepSize

        elif key == 'd':
            self.targetAngularVel -= angVelStepSize

        elif key == 's':
            self.targetLinearVel = 0.0
            self.targetAngularVel = 0.0

        elif key in gripperKeyBindings:
            self.lastGripperCommand = gripperKeyBindings[key]
            self.sendGripperGoal(self.lastGripperCommand)

        elif key in poses:
            self.sendArmGoal(poses[key], 2.0)

        elif key.lower() == 'q':
            self.stopRobot()
            self.destroy_node()
            rclpy.shutdown()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            sys.stdout.write('\nExiting...\n')
            sys.exit(0)

        twist = Twist()
        twist.linear.x = self.targetLinearVel
        twist.angular.z = self.targetAngularVel
        self.cmdVelPub.publish(twist)

        self.printStatus()

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
        sys.stdout.write('\r' + ' ' * 140 + '\r')

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
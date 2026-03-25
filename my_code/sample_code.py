#!/usr/bin/env python3

# ============================================================
# robotics assignment 3 - Teleop Node
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
# safety limits for the mobile base
# RESTORED: Negative limits so the robot can actually go in reverse!
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
    def __init__(self):
        super().__init__('simple_demo_controller')

        self.armJointNames = ['joint1', 'joint2', 'joint3', 'joint4']

        # Command velocities sent to the robot
        self.targetLinearVel = 0.0
        self.targetAngularVel = 0.0

        # Speed settings modified by v, b, n, m
        self.control_lin_vel = 0.05
        self.control_ang_vel = 0.20

        self.currentJ1 = 0.0
        self.currentJ2 = 0.0
        self.currentJ3 = 0.0
        self.currentJ4 = 0.0

        self.currentGripper = 0.0
        self.lastGripperCommand = 0.0

        self.armActionClient = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')

        self.gripperActionClient = ActionClient(
            self, GripperCommand, '/gripper_controller/gripper_cmd')

        self.cmdVelPub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.jointStateSub = self.create_subscription(
            JointState, '/joint_states', self.jointStateCallback, 10)

        self.settings = termios.tcgetattr(sys.stdin)
        self.timer = self.create_timer(0.1, self.runLoop)

        self.status_printed = False

        self.printInstructions()

    def jointStateCallback(self, msg):
        if 'joint1' in msg.name:
            self.currentJ1 = msg.position[msg.name.index('joint1')]
        if 'joint2' in msg.name:
            self.currentJ2 = msg.position[msg.name.index('joint2')]
        if 'joint3' in msg.name:
            self.currentJ3 = msg.position[msg.name.index('joint3')]
        if 'joint4' in msg.name:
            self.currentJ4 = msg.position[msg.name.index('joint4')]

        possibleGripperNames = ['gripper', 'gripper_left_joint', 'gripper_right_joint', 'gripper_sub_joint', 'joint5']
        for name in possibleGripperNames:
            if name in msg.name:
                self.currentGripper = msg.position[msg.name.index(name)]
                break

    def runLoop(self):
        key = getKey(self.settings).lower()

        # ----------------------------------------------------
        # Magnitude Control (Modify the Step Settings)
        # ----------------------------------------------------
        if key == 'v':
            self.control_lin_vel = min(self.control_lin_vel + 0.01, maxLinearVel)
        elif key == 'b':
            self.control_lin_vel = max(self.control_lin_vel - 0.01, 0.01) # keep above 0
        elif key == 'n':
            self.control_ang_vel = min(self.control_ang_vel + 0.10, maxAngularVel)
        elif key == 'm':
            self.control_ang_vel = max(self.control_ang_vel - 0.10, 0.10) # keep above 0

        # ----------------------------------------------------
        # Directional Control (Increment the current speed)
        # ----------------------------------------------------
        elif key == 'w':
            self.targetLinearVel = clamp(self.targetLinearVel + self.control_lin_vel, minLinearVel, maxLinearVel)
        elif key == 'x':
            self.targetLinearVel = clamp(self.targetLinearVel - self.control_lin_vel, minLinearVel, maxLinearVel)
        elif key == 'q':
            self.targetAngularVel = clamp(self.targetAngularVel + self.control_ang_vel, minAngularVel, maxAngularVel)
        elif key == 'e':
            self.targetAngularVel = clamp(self.targetAngularVel - self.control_ang_vel, minAngularVel, maxAngularVel)
        elif key == 's':
            self.targetLinearVel = 0.0
            self.targetAngularVel = 0.0

        # ----------------------------------------------------
        # Gripper & Arm Controls
        # ----------------------------------------------------
        elif key in gripperKeyBindings:
            self.lastGripperCommand = gripperKeyBindings[key]
            self.sendGripperGoal(self.lastGripperCommand)
        elif key in poses:
            self.sendArmGoal(poses[key], 2.0)

        # ----------------------------------------------------
        # Quit (Cleanly raises an error to trigger final shutdown)
        # ----------------------------------------------------
        elif key == '\x03':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            sys.stdout.write('\nExiting...\n')
            raise KeyboardInterrupt

        self.publishBaseCommand()
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
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.armJointNames

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=int(durationSec), nanosec=int((durationSec % 1) * 1e9))

        goal.trajectory.points.append(point)
        self.armActionClient.send_goal_async(goal)

    def sendGripperGoal(self, position):
        if not self.gripperActionClient.server_is_ready():
            return

        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 1.0
        self.gripperActionClient.send_goal_async(goal)

    def printStatus(self):
        if self.status_printed:
            # \033[5A moves up 5 lines, \r returns to the start of that line
            sys.stdout.write('\033[5A\r')
        else:
            sys.stdout.write('\r')

        # \033[K clears the line cleanly before printing the new values
        sys.stdout.write('\033[K' + f"Settings | Lin Step: {self.control_lin_vel:.2f} | Ang Step: {self.control_ang_vel:.2f}\n")
        sys.stdout.write('\033[K' + f"Velocity | Linear: {self.targetLinearVel:.2f} | Angular: {self.targetAngularVel:.2f}\n")
        sys.stdout.write('\033[K' + f"Arm      | J1:{self.currentJ1:.2f} J2:{self.currentJ2:.2f} J3:{self.currentJ3:.2f} J4:{self.currentJ4:.2f}\n")
        sys.stdout.write('\033[K' + f"Gripper  | Cmd: {self.lastGripperCommand:.2f} | State: {self.currentGripper:.2f}\n")
        sys.stdout.write('\033[K' + f"----------------------------------------------------\n")
        sys.stdout.flush()
        
        self.status_printed = True

    def printInstructions(self):
        print("""
----------------------------------------------------
 Teleoperation Control of TurtleBot3 + OpenManipulator
----------------------------------------------------
 Speed Settings:
 v / b : increase / decrease linear velocity
 n / m : increase / decrease angular velocity

 Base Movement:
 w : move forward
 x : move backward
 q : turn left
 e : turn right
 s : base stop

 Gripper
 g : gripper open
 h : gripper close

 Arm Preset
 0 : Extend Forward
 9 : Home pose
 8 : Wave pose

 Ctrl+C to quit
----------------------------------------------------""")


def main(args=None):
    rclpy.init(args=args)
    node = SimpleDemoController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # Handled cleanly
    finally:
        if rclpy.ok():
            node.stopRobot()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
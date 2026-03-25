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
# these step sizes decide how much velocity changes each time
# the user presses a movement key.
#
# smaller values = smoother, slower changes
# larger values = faster, more aggressive changes
# ============================================================
linVelStepSize = 0.01
angVelStepSize = 0.10


# ============================================================
# safety limits for the mobile base
#
# these prevent the program from increasing velocities forever.
# keeping values clamped makes the robot easier to control and
# safer during the demo.
# ============================================================
maxLinearVel = 0.20
minLinearVel = -0.20
maxAngularVel = 1.50
minAngularVel = -1.50


# ============================================================
# gripper command mapping
#
# these are simple preset gripper target positions.
# depending on your robot setup, you may adjust these slightly.
#
# g = open
# h = close
# ============================================================
gripperKeyBindings = {
    'g': 0.01,
    'h': -0.01
}


# ============================================================
# preset arm poses
#
# each pose is a list of target joint angles:
# [joint1, joint2, joint3, joint4]
#
# required by assignment:
# - home pose
# - extend forward pose
# - one extra custom pose
#
# important:
# these values may need minor tuning on your physical robot.
# if needed, use moveit / rviz to find better joint angles.
# ============================================================
poses = {
    '9': [0.0, 0.0, 0.0, 0.0],         # home pose
    '0': [0.0, -1.10, 0.75, 0.35],     # extend forward
    '8': [0.8, -0.65, 0.30, 0.85]      # wave / custom pose
}


def getKey(settings):
    """
    read one key from the keyboard without waiting for enter.

    why this is needed:
    normal terminal input waits for the enter key.
    for teleoperation, we want immediate single-key responses.

    steps:
    1. put terminal into raw mode
    2. check whether a key is available
    3. read one character if present
    4. restore terminal settings
    5. return the key or empty string
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

    example:
    if value = 2.0 and high = 1.5, return 1.5
    if value = -3.0 and low = -1.5, return -1.5
    """
    return max(low, min(value, high))


class SimpleDemoController(Node):
    """
    main ros2 node for the assignment.

    this single node handles:
    - keyboard input
    - base motion publishing
    - arm action goals
    - gripper action goals
    - joint state subscription
    - terminal status display
    """

    def __init__(self):
        """
        constructor for the ros2 node.

        here we create:
        - action clients for arm and gripper
        - publisher for base velocity commands
        - subscriber for joint states
        - timer for repeated control loop execution
        """
        super().__init__('simple_demo_controller')

        # ----------------------------------------------------
        # list of arm joint names expected by the trajectory
        # controller. these must match the robot configuration.
        # ----------------------------------------------------
        self.armJointNames = ['joint1', 'joint2', 'joint3', 'joint4']

        # ----------------------------------------------------
        # target base velocities.
        #
        # these are the current commanded values that will be
        # published repeatedly on /cmd_vel.
        # ----------------------------------------------------
        self.targetLinearVel = 0.0
        self.targetAngularVel = 0.0

        # ----------------------------------------------------
        # most recent joint angle readings from /joint_states.
        #
        # these are updated live by the subscriber callback and
        # shown on the terminal.
        # ----------------------------------------------------
        self.currentJ1 = 0.0
        self.currentJ2 = 0.0
        self.currentJ3 = 0.0
        self.currentJ4 = 0.0

        # ----------------------------------------------------
        # gripper status values
        #
        # currentGripper:
        #     measured from joint_states if available
        #
        # lastGripperCommand:
        #     most recent commanded open / close value
        # ----------------------------------------------------
        self.currentGripper = 0.0
        self.lastGripperCommand = 0.0

        # ----------------------------------------------------
        # action client for sending preset arm joint goals
        #
        # follow_joint_trajectory is the standard ros2 action
        # interface for commanding arm trajectories.
        # ----------------------------------------------------
        self.armActionClient = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )

        # ----------------------------------------------------
        # action client for sending gripper open / close goals
        # ----------------------------------------------------
        self.gripperActionClient = ActionClient(
            self,
            GripperCommand,
            '/gripper_controller/gripper_cmd'
        )

        # ----------------------------------------------------
        # publisher for mobile base movement
        #
        # the turtlebot base listens to geometry_msgs/Twist
        # messages on /cmd_vel.
        # ----------------------------------------------------
        self.cmdVelPub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ----------------------------------------------------
        # subscriber for robot joint states
        #
        # this lets us monitor the arm joint angles live.
        # ----------------------------------------------------
        self.jointStateSub = self.create_subscription(
            JointState,
            '/joint_states',
            self.jointStateCallback,
            10
        )

        # ----------------------------------------------------
        # save current terminal settings so we can restore them
        # after reading raw keyboard input.
        # ----------------------------------------------------
        self.settings = termios.tcgetattr(sys.stdin)

        # ----------------------------------------------------
        # timer that runs every 0.1 seconds
        #
        # this acts like the main control loop.
        # every cycle it:
        # - reads keyboard input
        # - updates velocities if needed
        # - publishes base command continuously
        # - refreshes status display
        # ----------------------------------------------------
        self.timer = self.create_timer(0.1, self.runLoop)

        # print menu once at startup
        self.printInstructions()

        # print initial status values
        self.printStatus()

    def jointStateCallback(self, msg):
        """
        callback for /joint_states messages.

        whenever the robot publishes joint positions, this
        function updates the values shown on the terminal.

        msg.name contains a list of joint names
        msg.position contains matching joint angle values

        we search for each arm joint by name, then store its
        current position in the class variables.
        """
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

        # ----------------------------------------------------
        # try several possible gripper joint names.
        #
        # different robot configs sometimes expose gripper
        # state under different names. this makes the script
        # more robust across setups.
        # ----------------------------------------------------
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
        """
        main control loop called by the timer.

        responsibilities:
        1. read a key if one is pressed
        2. update velocities or trigger arm / gripper commands
        3. continuously publish current base command
        4. update terminal display

        important:
        we publish the base command every cycle, even if no key
        is pressed. this helps the robot move more reliably than
        publishing only on keypress.
        """
        key = getKey(self.settings)

        # ----------------------------------------------------
        # mobile base controls
        # ----------------------------------------------------
        if key == 'w':
            # increase forward speed
            self.targetLinearVel = clamp(
                self.targetLinearVel + linVelStepSize,
                minLinearVel,
                maxLinearVel
            )

        elif key == 'x':
            # decrease linear velocity
            # if this goes below zero, the robot moves backward
            self.targetLinearVel = clamp(
                self.targetLinearVel - linVelStepSize,
                minLinearVel,
                maxLinearVel
            )

        elif key == 'a':
            # increase positive angular velocity
            # positive angular.z usually means turning left
            self.targetAngularVel = clamp(
                self.targetAngularVel + angVelStepSize,
                minAngularVel,
                maxAngularVel
            )

        elif key == 'd':
            # decrease angular velocity
            # negative angular.z usually means turning right
            self.targetAngularVel = clamp(
                self.targetAngularVel - angVelStepSize,
                minAngularVel,
                maxAngularVel
            )

        elif key == 's':
            # immediately stop all base motion
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
        elif key.lower() == 'q':
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
        """
        publish current mobile base command to /cmd_vel.

        twist.linear.x:
            forward / backward motion

        twist.angular.z:
            left / right turning motion
        """
        twist = Twist()
        twist.linear.x = self.targetLinearVel
        twist.angular.z = self.targetAngularVel
        self.cmdVelPub.publish(twist)

    def stopRobot(self):
        """
        publish a zero velocity command to stop the base.
        used when quitting or when cleanup is needed.
        """
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmdVelPub.publish(twist)

    def sendArmGoal(self, positions, durationSec):
        """
        send a preset arm trajectory goal.

        positions:
            list of desired joint angles for joints 1 to 4

        durationSec:
            how long the motion should take

        follow_joint_trajectory goals contain:
        - joint names
        - one or more trajectory points
        - each point has positions and a time_from_start
        """
        if not self.armActionClient.server_is_ready():
            self.get_logger().info('Arm action server not available')
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.armJointNames

        point = JointTrajectoryPoint()

        # set target joint positions for this preset pose
        point.positions = positions

        # specify how long the robot should take to reach them
        point.time_from_start = Duration(
            sec=int(durationSec),
            nanosec=int((durationSec % 1) * 1e9)
        )

        # add this point to the trajectory
        goal.trajectory.points.append(point)

        # send the goal asynchronously
        self.armActionClient.send_goal_async(goal)

    def sendGripperGoal(self, position):
        """
        send a gripper open / close command.

        position:
            desired gripper position

        max_effort:
            how strongly the gripper may apply force
        """
        if not self.gripperActionClient.server_is_ready():
            self.get_logger().info('Gripper action server not available')
            return

        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 1.0

        self.gripperActionClient.send_goal_async(goal)

    def printStatus(self):
        """
        print the live status block on the terminal.

        values shown:
        - current linear velocity command
        - current angular velocity command
        - current arm joint angles
        - gripper command and gripper state

        the cursor escape sequence at the end moves the cursor
        back upward so the same lines are updated in place,
        instead of printing endlessly downward.
        """
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
        """
        print the on-screen user interface once when the
        program starts.
        """
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
    """
    standard ros2 python entry point.

    steps:
    1. initialize ros2
    2. create node
    3. spin node so callbacks and timer can run
    4. stop robot and shut down ros2 on exit
    """
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
#!/usr/bin/env python3

# ============================================================
# robotics assignment 3 - Teleop Node (Direct Drive)
# ============================================================

import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Twist
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# ============================================================
# Configuration & Limits
# ============================================================
BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

# Increased to overcome physical floor friction
LIN_VEL_STEP_SIZE = 0.05
ANG_VEL_STEP_SIZE = 0.20

poses = {
    '9': [0.0, 0.0, 0.0, 0.0],         # home pose
    '0': [0.0, -1.10, 0.75, 0.35],     # extend forward
    '8': [0.8, -0.65, 0.30, 0.85]      # wave / custom pose
}

gripper_bindings = {
    'g': 0.01,  # Open
    'h': -0.01  # Close
}

msg = """
----------------------------------------------------
Teleoperation Control of TurtleBot3 + OpenManipulator
----------------------------------------------------
Base Movement Controls:
        i / w             (Move Forward)
 j / a    s    l / d      (Left / Stop / Right)
        k / x             (Move Backward)

s : force stop all base movement

Arm Preset Poses:
0 : Extend Forward
9 : Home pose
8 : Wave pose

Gripper:
g : gripper open
h : gripper close

CTRL-C to quit
----------------------------------------------------
"""

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %.2f\t angular vel %.2f " % (target_linear_vel, target_angular_vel)

def constrain(input_vel, low, high):
    if input_vel < low:
        input_vel = low
    elif input_vel > high:
        input_vel = high
    return input_vel

def checkLinearLimitVelocity(vel):
    return constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)

def checkAngularLimitVelocity(vel):
    return constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)

class ManipulationTeleop(Node):
    def __init__(self):
        super().__init__('manipulation_teleop')
        
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')
        
        self.arm_joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

    def send_arm_goal(self, positions, duration_sec=2.0):
        if not self.arm_client.server_is_ready():
            self.get_logger().warn('Arm action server not available yet...')
            return

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.arm_joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))

        goal_msg.trajectory.points.append(point)
        self.arm_client.send_goal_async(goal_msg)

    def send_gripper_goal(self, position):
        if not self.gripper_client.server_is_ready():
            self.get_logger().warn('Gripper action server not available yet...')
            return

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = 1.0
        self.gripper_client.send_goal_async(goal_msg)


def main(args=None):
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)

    node = ManipulationTeleop()

    status = 0
    target_linear_vel   = 0.0
    target_angular_vel  = 0.0

    print(msg)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            
            key = getKey(settings).lower()

            # ----------------------------------------------------
            # Base Controls (w/a/s/d and i/j/k/l supported)
            # ----------------------------------------------------
            if key in ['w', 'i']:
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel, target_angular_vel))
            
            elif key in ['x', 'k']:
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel, target_angular_vel))
            
            elif key in ['a', 'j', 'q']:
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel, target_angular_vel))
            
            elif key in ['d', 'l', 'e']:
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel, target_angular_vel))
            
            elif key == 's':
                target_linear_vel   = 0.0
                target_angular_vel  = 0.0
                print(vels(target_linear_vel, target_angular_vel))
            
            # ----------------------------------------------------
            # Arm Controls
            # ----------------------------------------------------
            elif key in poses:
                print(f"Executing Arm Pose: {key}")
                node.send_arm_goal(poses[key])
            
            # ----------------------------------------------------
            # Gripper Controls
            # ----------------------------------------------------
            elif key in gripper_bindings:
                print(f"Executing Gripper Command: {key}")
                node.send_gripper_goal(gripper_bindings[key])

            # ----------------------------------------------------
            # Quit Command
            # ----------------------------------------------------
            elif key == '\x03': # CTRL-C
                break

            # Print menu every 14 inputs to keep the terminal clean
            if status == 14:
                print(msg)
                status = 0

            # ----------------------------------------------------
            # Direct Velocity Assignment (Bypasses Stuttering Math)
            # ----------------------------------------------------
            twist = Twist()
            twist.linear.x = float(target_linear_vel)
            twist.linear.y = 0.0
            twist.linear.z = 0.0

            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = float(target_angular_vel)

            node.cmd_vel_pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        # Safety Stop on exit
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        node.cmd_vel_pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
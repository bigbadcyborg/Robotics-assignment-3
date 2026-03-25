# Sample Code for Robotics_Assignment_3
# Copyright: 2026 CS 4379K / CS 5342 Introduction to Autonomous Robotics, Robotics and Autonomous Systems

# Teleoperation Code for Turtlebot3 for Assignment 3.

# This code will give you an introduction to coding in ROS2. This code will run as-is, but you are required to modify it for the assignment requirement and submit the source code on Canvas.

# Refer to the Lab PowerPoint materials and Appendix of Assignment 3 to learn more about coding on ROS2 and the hardware architecture of Turtlebot3.
# You can either run this code on the simulator, Turtlebot3 Nvidia Jetson, or on a Remote-PC docker image.
# You would need a basic understanding of Python Data Structure and Object Oriented Programming to understand this code.

# ROS2
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# ROS2 Programming
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
    # Advice: You will need geometry_msgs for the Twist message to control the base
from geometry_msgs.msg import Twist
    # Advice: You will need sensor_msgs for JointState to read the arm angles
from sensor_msgs.msg import JointState

# Terminal related
import sys
import tty
import termios
import select

LIN_VEL_STEP_SIZE = 0.01
##############################################################################
# HINT: Add an ANG_VEL_STEP_SIZE as a constant here for your angular velocity steps. A big part of Robot programming is calibrating constants, just like what you have done for parameters in assignment 1. 
# HINT: 0.1 should serve you well for the assignment.


##############################################################################

# We are giving you complete constants for the gripper to save you the trouble. 
GRIPPER_KEY_BINDINGS = {
    'g': 0.01,  # Open
    'h': -0.01  # Close
}

POSES = {
    '9': [0.0, 0.0, 0.0, 0.0],  # Home pose
    ##############################################################################
    # HINT: Each element in the Python list represents arm angles in radians.
    # Note: In Python lists, the order of elements as they were inserted is ordered and respected.
    # HINT: Add key '0' for Extend arm forward
    # HINT: Add key '8' for Custom pose


    ##############################################################################
}

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class SimpleDemoController(Node):
    def __init__(self):

        # Initialize and Define Node name
        super().__init__('simple_demo_controller')

        self.arm_joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        
        # State Variables
        self.target_linear_vel = 0.0
        ##############################################################################
        # Linear Velocity: This is your standard "speedometer" speed. It measures how much distance you cover over time in m/s.
        # Angular Velocity: This measures how fast something is rotating or turning. Instead of distance, it measures the angle covered over time in rad/s.
        # HINT: Add a variable to track target_angular_vel. Refer to linear vel for reference.

        
        ##############################################################################        
        
        self.current_j1 = 0.0
        
        ##############################################################################
        # HINT: Add variables to track current_j2, current_j3, and current_j4 . Refer to j1 for reference.



        ##############################################################################
        # Action Clients
        self.arm_action_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper_action_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')

        # Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.joint_state_sub = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)

        self.settings = termios.tcgetattr(sys.stdin)
        self.timer = self.create_timer(0.1, self.run_loop)
        self.print_instructions()

    def joint_state_callback(self, msg):
        
        ##############################################################################
        # Extract the current joint angles from the robot
        # HINT: Extract the positions for 'joint2', 'joint3', and 'joint4' here by looking at joint 1 as an example.
        # Make sure to update your tracking variables!
        
        if 'joint1' in msg.name:
            idx = msg.name.index('joint1')
            self.current_j1 = msg.position[idx]


        ##############################################################################


    def run_loop(self):
        key = get_key(self.settings)
        if not key:
            return

        # Base Control Logic
        if key == 'w':
            self.target_linear_vel += LIN_VEL_STEP_SIZE
        elif key == 's':
            ##############################################################################
            # Implement 's' base stop functionality so that robots' wheels stop when you press s.
            # HINT: 's' is supposed to be a base stop. Don't forget to reset angular velocity to 0.0 here too!
            self.target_linear_vel = 0.0            


        
            ##############################################################################

        ##############################################################################
        # Implement other keys that control the base by referring to w.
        # HINT: Implement 'x' (decrease linear velocity)
        # HINT: Implement 'a' (increase angular velocity)
        # HINT: Implement 'd' (decrease angular velocity)


        ##############################################################################
        
        # Arm & Gripper Logic
        elif key in GRIPPER_KEY_BINDINGS:
            self.send_gripper_goal(GRIPPER_KEY_BINDINGS[key])
        
        elif key in POSES:
            self.send_arm_goal(POSES[key], 2.0)
            
        elif key.lower() == 'q':
            self.destroy_node()
            rclpy.shutdown()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            sys.stdout.write('\nExiting...\n')
            sys.exit(0)

        # Publish Twist Message 
        ##############################################################################
        # Modify the content of the message you are publishing to add the content of your Angular variable.
        # HINT: Set twist.angular.z to your target_angular_vel variable
        
        twist = Twist()
        twist.linear.x = self.target_linear_vel
        



        
        ##############################################################################
        self.cmd_vel_pub.publish(twist)

        # Print the current status to the terminal
        self.print_status()

    def send_arm_goal(self, positions, duration_sec):
        if not self.arm_action_client.server_is_ready():
            self.get_logger().info("Arm action server not available")
            return
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.arm_joint_names
        point = JointTrajectoryPoint(
            positions=positions, 
            time_from_start=Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
        )
        goal.trajectory.points.append(point)
        self.arm_action_client.send_goal_async(goal)

    def send_gripper_goal(self, position):
        if not self.gripper_action_client.server_is_ready():
            self.get_logger().info("Gripper action server not available")
            return
            
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 1.0
        self.gripper_action_client.send_goal_async(goal)

    def print_status(self):
        # Clear the previous line and print the updated status
        
        ##############################################################################
        # Modify this print block to include Angular Velocity and Joint angles J2, J3, and J4 you implemented in the above code.
        # Hint
        # Target Output: "Present Linear Velocity: 0.000, Angular Velocity: 0.000"
        # Target Output: "Present Arm Joint Angle J1: 0.000 J2: 0.000 J3: 0.000 J4: 0.000"
        
        sys.stdout.write('\r' + ' ' * 80 + '\r') # Clear line
        status_string = (f"Present Linear Velocity: {self.target_linear_vel:.3f}\n"
                         f"Present Arm Joint Angle J1: {self.current_j1:.3f}\n"
                         f"---------------------------\n")

        
        ##############################################################################
        # Moves cursor up 3 lines so the next print overwrites it cleanly
        sys.stdout.write(status_string + "\033[3A")
        sys.stdout.flush()

    def print_instructions(self):
        ##############################################################################
        # HINT: Update this menu to reflect the complete set of controls once you implement x, a, d, 0, and 8.
        print("""
        ---------------------------
        Sample_Code: Teleoperation Control of TurtleBot3 + OpenManipulatorX
        ---------------------------
        w : increase linear velocity
        s : base stop

        g : gripper open
        h : gripper close
        
        9 : Home pose
        
        q to quit
        ---------------------------
        """)
        
        ##############################################################################

def main(args=None):
    rclpy.init(args=args)
    node = SimpleDemoController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

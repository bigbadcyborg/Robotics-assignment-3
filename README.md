# 2026 CS 4379K / CS 5342 Introduction to Autonomous Robotics, Robotics and Autonomous Systems

## Programming Assignment: Milestone 3 (V1.05)

**Minhyuk Park and Tsz-Chiu Au**

### Introduction

Welcome to CS 4379K / CS 5342. The following milestones will give you an idea of how to interact with ROBOTIS’ Turtlebot3 Waffle Pi with a Manipulator Arm using the Robot Operating System 2.

The third milestone is about ROS 2 programming. In this milestone, you will try operating a physical robot arm and write a ROS 2 program to control the OpenManipulator on the Turtlebot using a keyboard. The program should be similar to `turtlebot3_teleop_key`, but it will accept a few more keystrokes to control the robot arm. Your program should provide some basic functions, such as setting the arm to some predefined poses and opening/closing the gripper. In addition, your program should read the current configuration of the arm and the gripper and display the information on the screen. Of course, please feel free to add more functionalities to your program.

To do this, you will deploy our pre-configured Docker container that sets up all the software that is required for the assignment.
Please refer to the following video for an explanation of what a Docker container environment is. 

[https://www.youtube.com/watch?v=Gjnup-PuquQ](https://www.youtube.com/watch?v=Gjnup-PuquQ)

Robot Operating System version is associated with Ubuntu Long-Term Support Versions (e.g. Ubuntu 22.04 with Humble). We are using **ROS 2 Humble in a Docker environment** for Remote-PC. You might find the official tutorial on ROS 2 Humble useful in this course:

[https://docs.ros.org/en/humble/Tutorials.html](https://docs.ros.org/en/humble/Tutorials.html)

For all questions regarding milestone assignments and the robot, **you should contact the Doctoral Instructor Assistant via direct message on Slack**. Please do not contact the Instructor with questions regarding the milestone assignments. This is the URL for Slack for this course. 

<https://spring2026txstrobot.slack.com/>


### Assignment requirement

**Source Code Submission** is required for Milestone Assignment 3 on Canvas.

In addition, as usual, a **hardware video demonstration submission** is required for Milestone Assignment 3. 

You need to demonstrate that you have a working setup and can operate the turtlebot by making videos. This will also demonstrate that you have a working setup for working with a physical turtlebot. Refer to the demo requirement section at the end of the milestone assignment on what to include in the video.

**[SUBMISSION RULES]**

* **Individual Submission:** **Every team member must submit the video link(s) separately to Canvas.** If the video is duplicated within a team, that is acceptable; however, this ensures that only active participants who have access to the team’s recordings can receive credit. 

* **Standardized Hosting:** **To manage file sizes, do not upload raw video files (e.g., MP4) directly to Canvas.** Instead, **upload your videos to YouTube (set as "Unlisted")** and submit the links via a document.

### Video Demo Requirements

Your group will **record** one or more video clips. The estimated total length of the video clips is approximately three and a half minutes. **While you do not need to perform complex editing, please keep the total duration to a few minutes to ensure it remains concise.** One group member should narrate the video, explaining each step as it's performed. At the beginning of the first video clip, please show every group member's face and state the names of all group members.

Your recording setup should be organized to show all relevant windows at once: the terminal(s) used for launching nodes, the Gazebo simulation window, and the RViz visualization window.

You do not need to edit the videos, and uploading raw **footage** will suffice. You can split the demonstration into multiple videos **if necessary to show different parts of the requirement.** 

Rules for robot usage will apply for working with the physical Turtlebot3. Please refer to the inventory list given to you separately and the rules for the Robot room usage.

> **Major Changes**
> * **v 1.0.5:** Some Changes to the assignment as requested.
> * **v 1.0.0:** Initial Release.

---

### Part 1: Assignment 2 Revision: Operation of Physical Turtlebot 3 Arm

We will do a revision on assignment 2 as part 1 of this assignment. It is always important to verify that the hardware for the robot is working properly before you move on to programming the robot hardware.

This manual is based on the following manual for Humble.

* **(Original URL)** [https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#simulation)

**Operating Physical Turtlebot 3 Arm**

This assumes that you have a working setup from Milestone Assignment 1 Part 1. Please execute all instructions with **[Remote PC]** on the Docker shell. Note that you have to enable the GUI and start the Docker container by following instruction from Milestone Assignment 1. Please execute all instructions with **[Turtlebot Nvidia Jetson]** on Turtlebot Jetson's native bash shell without Docker.

**[Turtlebot Nvidia Jeston]** **[Remote PC]** Make sure Jeston is connected to the small router we provided (small_blue_wifi), along with the remote PC. 

```
SSID: small_blue_wifi
Password: turtlebot

This network does not have internet access.

Your Remote PC does not need sudo to connect to this network because it is a remembered network.

Do not press the reset button on the router just because you have trouble connecting to this network.
```

**[Turtlebot Nvidia Jeston]** The following command will bring up the actual TurtleBot3 hardware with OpenMANIPULATOR-X on it. Open a terminal from the **TurtleBot3 SBC**. Bring up the TurtleBot3 Manipulation using the following command.

```bash
ros2 launch turtlebot3_manipulation_bringup hardware.launch.py

```

**[Remote PC]** Enter the command below on another Docker shell to launch MoveIt on RViz. The arm should move when you move the blue ball on RViz and click “plan and execute”. We highly recommend noting down some joint angles for the next part of the assignment.

```bash
ros2 launch turtlebot3_manipulation_moveit_config moveit_core.launch.py

```

**[Remote PC]** To operate the robot with the keyboard teleoperation node, the RViz must be terminated. Then launch the servo server node and teleoperation nodes on a separate docker terminal window.

```bash
ros2 launch turtlebot3_manipulation_moveit_config servo.launch.py
```
```bash
ros2 run turtlebot3_manipulation_teleop turtlebot3_manipulation_teleop

```

**[Optional]** How to set the clock on Remote-PC and Jetson 

You should not need to do this, as we installed a Real Time Clock on both Jetson and the laptop.

A common reason why physical manipulation of the robot arm fails is that the clock between the NVIDIA Jetson and the Remote PC is not synced. If you suspect this, read the error message on the terminal for bringup to verify.

The accuracy of time itself does not matter; the timestamps for ROS 2 messages between two machines should be within 1s of each other.

If you are using the provided laptop, the GUI from Ubuntu settings prevents you from changing the time without sudo. However, you can change it with the timedatectl command on the terminal

```bash
# View status
timedatectl

# Change timezone
#timedatectl set-timezone Region/City
timedatectl set-timezone America/Chicago

# Enable / Disable Network Time Protocol. This must be disabled for manual time setting.
timedatectl set-ntp true
timedatectl set-ntp false

# Manually set time
#timedatectl set-time "YYYY-MM-DD HH:MM:SS"
timedatectl set-time "2025-02-09 15:30:00

```

---

### Part 2: Programming a Command Line Interface for custom teleoperation of Turtlebot3 with OMX Arm

In part 2 of the assignment, you are going to program your own client teleoperation program in ROS2 as a Python script to interface with the ROBOTIS Turtlebot 3.

We are providing you with a sample code to help you get started. We recommend reading the lab guides and appendix if you want to learn how to program using ROS2. 

**[Remote-PC][Turtlebot_Jeston]** Make sure both remote-pc and jetson are connected to the same local wifi network, small_blue_wifi. 

**[Remote-PC][Turtlebot_Jeston]** Close all programs you have opened for Part 1 of the exercise.

**[Turtlebot Nvidia Jeston]** The following command will bring up the actual TurtleBot3 hardware with OpenMANIPULATOR-X on it. Open a **Terminal from the TurtleBot3 SBC**. Bring up the TurtleBot3 Manipulation using the following command.

```bash
ros2 launch turtlebot3_manipulation_bringup hardware.launch.py

```

**[Remote-PC]** Clone the course Docker repository on the **Terminal shell** on the remote pc. 

```bash
cd ~/turtlebot_docker
git clone https://github.com/dmz44/Robotics_Assignment_3.git
cp Robotics_Assignment_3/sample_code.py my_code/sample_code.py
```

**[Remote-PC]** Download and execute the provided sample code on the **Docker shell** on the remote pc. 

```bash
cd ~/my_code
python3 sample_code.py
```

**[Remote-PC]** You can edit your Python script by going to ~/turtlebot_docker/my_code. Changes made within the folder will be reflected inside Docker immediately.


**[Remote-PC]** You would modify the sample_code.py to achieve the following requirements. 

* **Interface:** Your custom interface should appear on the screen.
* **Live Status Display:** As you perform the following actions, show the status of the joint angles and gripper on your terminal display in real-time. 
* **Mobile Base Control:** Your code should be able to drive the robot smoothly forward, backward, turning left, and turning right.
* **Manipulator Arm Control Preset:**
* Demonstrate commanding the arm to move between at least three distinct, predefined positions. You must include a "Extend Forward" and a "Home" position (as if you were to grab an object directly in front of the robot and lift it off the ground), plus one more of your own design (e.g., a "Ready" or "Wave" position).
* **Gripper Control:** While the arm is in an extended position, demonstrate your program's ability to reliably open and close the gripper on command.

**Example interface of your program**

```text
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
 Present Linear Velocity: 0.000, Angular Velocity: 0.000
 Present Arm Joint Angle J1: 0.000 J2: 0.000 J3: 0.000 J4: 0.000
 ---------------------------

```

**[Optional][Turtlebot Nvidia Jeston]** Note that it does not matter if you execute the sample code on Turtlebot's Jetson or the Remote PC, thanks to ROS2. As an optional exercise, you can try to copy over and execute sample_code on Jetson either with scp or physically with flipbook. 

**[Optional][Remote PC]** The sample code does work on the simulator from assignment 1. This can help you develop while you wait your turn to work on Turtlebot 3. This kind of workflow is called "Digital Twin" and "Sim to Real".

---

### Video Demo Requirements (3-4 Minute Demonstration)

Please refer to the video submission requirements in the introduction.

Your submission must include two items: **links to the video file and a single .zip file containing all of your source code**.

**Part A: Code Walkthrough**
The goal of this part is to explain the design and structure of your program. With your source code visible on the screen, you will guide us through the key components.

* **High-Level Overview:** Start with a brief architectural explanation. What is the overall structure of your code? (e.g., "We created a single ROS2 node with classes for handling user input, publishing commands, and subscribing to robot state.")

**Show Key Functions:** Point to and briefly explain the specific parts of your code that are responsible for:
* **Base Movement:** The function or code block that creates and publishes relevant messages to the relevant topic.
* **Arm & Gripper Control:** The code that sends goals to the arm and gripper.
* **State Subscription:** How are you listening to the relevant topics to receive real-time arm positions?

**Part B: Live Program Demonstration**
In this part, you will run your program and demonstrate all of its features with the physical robot. 

* **Launch Program:** From a standard terminal, run your program. Your custom interface should appear on the screen.
* **Live Status Display:** As you perform the following actions, point out that the joint angles and gripper status on your terminal display are updating in real-time. 
* **Mobile Base Control:** Using the controls you implemented, demonstrate driving the robot smoothly forward, backward, turning left, and turning right.
* **Manipulator Arm Control:**
* Demonstrate commanding the arm to move between at least three distinct, predefined positions. You must include a "Home" position and an "Extend Forward" position (as if you were to grab an object directly in front of the robot), plus one more of your own design (e.g., a "Ready" or "Wave" position).
* Show the arm moving smoothly from one predefined position to the next.
* **Gripper Control:** While the arm is in an extended position, demonstrate your program's ability to reliably open and close the gripper on command.

---

### Appendix

#### [Guide] ROS2 Programming Guide

**Contents in the Guide are for your reference, and we do not need you to demonstrate what you have learned in this section**

**Open Manipulator X Arm Control Documentation**

Reference: “ROS Robot Programming” Chapter 13 by Robotis
[https://www.robotis.us/ros-robot-programming-book-digital-copy/?srsltid=AfmBOoq5eDXDdPVY6gIEhJ889Goef0EndZsfy17A8Q5FPfRbjAXwc4WW](https://www.robotis.us/ros-robot-programming-book-digital-copy/?srsltid=AfmBOoq5eDXDdPVY6gIEhJ889Goef0EndZsfy17A8Q5FPfRbjAXwc4WW)

Before we begin, we would like to introduce you to the ROS 1 programming book from ROBOTIS and refer to their documentation on the Open-Manipulator (Chapter 13). While this book is outdated with information on ROS 1, not 2, it gives us an idea of how the designers designed the robotic arm on the turtlebot.
The robot arm is a 4-degree-of-freedom (DOF) robot arm, with the end effector or gripper being just a single 1 DOF joint arm.

Recall from milestone assignment 2 that we moved around the arm end effector or directly used joint angles under the joint tab in RViz. Behind the scenes, RViz has been interfacing with the bringup program and Moveit 2 planner to plan and give an intermediate state the robot arm should take in sequence so that the arm does not exceed its physical limits or collide with itself.

A critical concept to go over is inverse kinematics and forward kinematics. These two concepts are used to calculate relevant control commands. Note that the unit for joint angles is in radians and task space control is in m.

In summary, from the documentation, we can deduce that we somehow need to provide either target joint angles or target task space coordinates with pitch, roll, and yaw values to control the arm. Similarly, we should also be able to receive relevant joint information and target task space coordinates through the same pipeline. Recall that in RViz, you can either drag the gripper around in 3D space or command specific joint angles on the Joints tab to command the robot arm movement. This means you have already exploited forward and inverse kinematics to plan and move the robotic arm. Availability of both forward and inverse kinematics is a powerful tool for robot programmers.

To make things simple, our course will focus on control through joint space as illustrated on the forward kinematics diagram.

**[Optional]** While not necessary for the set of assignments we are going to give you, if you are interested in forward kinematics results and utilizing the resulting X Y Z gripper location in your program, create a tf buffer, tf transform listener from ROS tf2 packages and look up the transform between ‘base_link’ and ‘link5’ for the transform between the target frame and source frame.

**ROS2 Communication Programming**

Largely speaking, there are three interfaces, publisher & subscriber, service & client, and action & action client, that you need to be aware of. Below are links to relevant tutorials. It is highly recommended that you read the tutorials to get a comprehensive idea of the ROS2 ecosystem.

References
* Suggested reading: [Publisher & Subscriber] Getting continuous sensor data or other continuous vehicle command: [https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
* Suggested reading: [Action Server & Client] Commands that take time to complete for Arm and Gripper: [https://docs.ros.org/en/humble/Tutorials/Intermediate/Creating-an-Action.html](https://docs.ros.org/en/humble/Tutorials/Intermediate/Creating-an-Action.html) [https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html)
* Suggested reading: [rclpy] ROS Client Library for Python: [https://docs.ros.org/en/iron/p/rclpy/api.html](https://docs.ros.org/en/iron/p/rclpy/api.html)
* Suggested reading: [Service and Client] Short, one-off command for tasks such as parameter updates: [https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)
* Suggested reading: [Moveit Python API Tutorial] Note that because their robot arm configuration is different from ours, this tutorial will not directly help you with this assignment. [https://moveit.picknik.ai/main/doc/examples/motion_planning_python_api/motion_planning_python_api_tutorial.html](https://moveit.picknik.ai/main/doc/examples/motion_planning_python_api/motion_planning_python_api_tutorial.html)

**ROS2 Fundamentals**

* **Nodes:** You need to build around a class that inherits from Node. A node is the primary building block of a ROS2 system, an executable process that performs a specific task.

**ROS2 Communication Paradigms**

Your code would use three different ways to communicate with the robot’s systems and rclpy to facilitate the communication.

1. **Actions (for the Arm and Gripper):** The arm and gripper movement is a process that takes time. For tasks like this, ROS2 uses actions. You need to understand the Action Client/Server model:
* **Goal:** The client (our node) sends a goal to the server (the robot's controller). In our code, the goal is a trajectory, or a sequence of joint positions over time.
* **Feedback:** The server can provide continuous feedback (e.g., the arm's current position) while it's executing the goal.
* **Result:** Once the task is done, the server sends back a final result (e.g., success or an error code).
* **Pipeline:** The ActionClient for FollowJointTrajectory is created, and the `send_joint_goal` function builds and sends the goal.


2. **Services (for Parameter update):** A parameter update is a quick, one-off command. For this, ROS2 uses services. You need to understand the Service Client/Server model:
* **Request:** The client (our node) sends a request to the server.
* **Response:** The server performs the task and sends back a single response. This is a synchronous-style communication. The client usually waits for the response before continuing.
* **Pipeline:** You need to set up the connection to the relevant parameter service, and your function needs to send the request to update the parameter.


3. **rclpy (ROS Client Library for Python) library:**
* **Node and Client Initialization:** You need to understand how to initialize rclpy, create a node class, and create action and service clients in the `__init__` method.
* **Asynchronous Calls (async and future):** Instead of blocking the whole program, asynchronous functions immediately return a "future" object. For example, `rclpy.spin_until_future_complete(self, future)` pauses the execution of that function to wait for the result to come back. This is essential for preventing nodes from freezing.
* **Message Types:** Every ROS2 communication uses a specific message type. You must understand how to import these (from `control_msgs.action` import `FollowJointTrajectory`) and how to create and populate them with data (e.g., creating `JointTrajectoryPoint` and setting its positions).

4. **Publishers and Subscribers (for Continuous Data Streams):** This is for continuous, one-way data flow. A publisher node sends messages to a specific topic (like a radio channel). Any number of subscriber nodes can listen to that topic to receive the messages. The publisher doesn't know, or care who is listening, and the subscribers don't know who is publishing. This creates a flexible, decoupled system.
* **The Callback Function (Event-Driven Programming):** You must understand that they do not call this function directly in their main logic. Instead, the ROS2 framework automatically calls it for them in the background whenever a new message is published to the topic. This is a core principle of event-driven programming. The function's job is to take the incoming message (msg) and do something with it.
* **"Spinning" to Process Callbacks:** A ROS2 node doesn't process incoming data by default; it must be told to do so. "Spinning" is that process. `spin_once()` tells the node: "Check for any new messages or service requests right now, execute their callbacks, and then return control to my script." We use this when we want to manually process incoming data at specific points in our code. `spin()` would tell the node: "Continuously check for messages and execute callbacks forever until the node is shut down." This is used when a node's only job is to react to events. Without a spin or `spin_once` call, the `joint_state_callback` would never run, and the subscriber would be useless.
* **Pipeline:** It's perfect for broadcasting sensor data, robot status, or any information that needs to be continuously available to the rest of the system. The robot's hardware drivers are constantly publishing the current state of all joints to the `/joint_states` topic. It is the perfect place to hook into to get real-time position data.

Finally, you need a grasp of some basic robotic arm concepts that are described above, such as joint space. 

**[Guide] ROS2 Robot Control and Feedback Interface Investigation**

Reference: ROS2 CLI tools
[https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)

With the theoretical background dialed in, let us try exposing what interface is available for secondary development ourselves using built-in ROS2 CLI tools.

The first interface we are going to investigate is the publisher-subscriber interface. ROS 2 breaks complex systems down into many modular nodes. Topics are a vital element of the ROS graph that act as a bus for nodes to exchange messages through a publisher-subscriber interface.

Let us try running a rostopic list while running bringup. This could give us some hints on how to interact with the turtlebot.

**[Turtlebot Nvidia Jetson]** Run bringup.

```bash
ros2 launch turtlebot3_manipulation_bringup hardware.launch.py

```

While hardware bringup is running on Jetson on Turtlebot3,

**[Turtlebot Jetson or Remote PC Docker Shell on same network]** Run the ros2 topic list command on a new terminal

```bash
ros2 topic list -t

```
This should expose available topics in ROS2 and their message type. This command is designed to expose available ROS2 topics and their message type.

The output should be the following.

```text
/arm_controller/joint_trajectory [trajectory_msgs/msg/JointTrajectory]
/arm_controller/state [control_msgs/msg/JointTrajectoryControllerState]
/cmd_vel [geometry_msgs/msg/Twist]
/dynamic_joint_states [control_msgs/msg/DynamicJointState]
/imu_broadcaster/imu [sensor_msgs/msg/Imu]
/joint_states [sensor_msgs/msg/JointState]
/odom [nav_msgs/msg/Odometry]
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/robot_description [std_msgs/msg/String]
/rosout [rcl_interfaces/msg/Log]
/scan [sensor_msgs/msg/LaserScan]
/tf [tf2_msgs/msg/TFMessage]
/tf_static [tf2_msgs/msg/TFMessage]

```

If you do not get the full list and just get the following, check if your hardware bringup is running properly.

```text
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/rosout [rcl_interfaces/msg/Log]
```

We would like you to pay attention to the following topics and message types. We have added some descriptions to make your life easier.

Commanding wheels
`/cmd_vel` `[geometry_msgs/msg/Twist]`

Getting feedback on estimated positions and velocity in the world
`/odom` `[nav_msgs/msg/Odometry]`

Getting feedback on joints, including wheels
`/joint_states` `[sensor_msgs/msg/JointState]`

Controlling Arm
`/arm_controller/joint_trajectory` `[trajectory_msgs/msg/JointTrajectory]`

You can read the following on what a message is in ROS2. 

Suggested reading: Data passing in ROS2: Understanding and defining msg in ROS2
[https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)

Let's do a more in-depth investigation of the topics.

**[Turtlebot Nvidia Jetson or Remote PC on same network]** Try ros2 topic echo to get the real-time message status of the topic odom

```bash
ros2 topic echo /odom

```

That should output “echo” of your real-time topic information of topic /odom. This exposes the structure and elements of the messages.

```yaml
twist:
  twist:
    linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
  covariance:
  - 0.001
  - 0.0
  - 0.0
…

```

**[Turtlebot Nvidia Jetson or Remote PC on same network]**  You might find the hz command useful to find the rate at which the data is published.

```bash
ros 2 topic hz /odom

```

It should output the measured rate at which information is being produced.

```text
average rate: 49.833
    min: 0.009s max: 0.026s std dev: 0.00517s window: 51

```

**[Turtlebot Nvidia Jetson or Remote PC on same network]**  Try ros2 topic info on topic odom.

```bash
ros2 topic info /odom

```

It should output the following information about the topic

```text
Type: nav_msgs/msg/Odometry
Publisher count: 1
Subscription count: 0

```

From this, we can see that the topic `/odom` has type `nav_msgs/msg/Odometry`, with 1 publisher active with no subscriber active.

**[Turtlebot Nvidia Jetson or Remote PC on same network]**  Let us investigate another topic.

```bash
ros2 topic info /cmd_vel

```

It should output the following information about the topic

```text
Type: geometry_msgs/msg/Twist
Publisher count: 0
Subscription count: 1

```

This topic has no publisher but has an active subscriber.

From our short investigation, we can see that bringup from manufacturer Robotis designed topics `/odom` to be a **producer** of information, and `/cmv_vel` to be a **consumer** of the information. You can continue the investigation to get a better picture of the overall turtlebot architecture.

**[Turtlebot Nvidia Jetson or Remote PC on same network]** Lastly, let us investigate the other ROS2 interface called actions.

```bash
ros2 action list
ros2 action info

ros2 service list
ros2 service type

```

We recommend that you try out investigating the following actions on your own. 

Actions

`/arm_controller/follow_joint_trajectory`
`/gripper_controller/gripper_cmd`

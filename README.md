# Sens-O-ROS
An Intercollege event that aims to provide deep insights on basic hardware components, sensors used in mobile robots and deep understanding of ROS framework and simulations along with hardware integration with ROS.

**ROS INTRODUCTION** 

ROS is an open-source robot operating system
A set of software libraries and tools that help you build robot applications that work across a wide variety of robotic platforms 

  ![image](https://github.com/user-attachments/assets/2420e536-6362-49d9-965d-8bf769e608af)

**Why should one learn this and what was it like before?**

Lack of standards and Little code reusability

Keeping reinventing (or rewriting) device drivers, access to robot’s interfaces, management of onboard processes, inter-process communication protocols, … 

New robot in the lab (or in the factory). 

Start re-coding (mostly) from scratch


**FEATURES OF ROS**

The operating system side, which provides standard OS-like services such as: 

   – hardware abstraction
   
   – low-level device control 
   
   – implementation of commonly used functionality
   
   – message-passing between processes
   
   – package management
   
A suite of user contributed packages that implement common robot functionality such as SLAM, planning, perception, vision, manipulation, etc.

------------------------------------------------------------------------------------------------------------------------------------------
**But it's not a standard OS**


  ![image](https://github.com/user-attachments/assets/28fef109-7bab-4cb5-a424-82d4067b5359)


------------------------------------------------------------------------------------------------------------------------------------------
**CHARACTERISTICS**

Peer-to-Peer Messaging

  - ROS comprises nodes exchanging messages continuously.
  
Tool-based Approach

  - Various generic programs facilitate tasks like visualization, logging and data plotting.
  
Multilingual Support

  - Modules can be written in languages like C++, Python, LISP, JAVA etc.
  
Free and Open Source Community

  - ROS is community-driven with repositories for collaboration
  
------------------------------------------------------------------------------------------------------------------------------------------
  **SOME COMMON MYTHS**

An actual operating system OR
A programming language

A programming environment / IDE OR
A hard real-time architecture 

------------------------------------------------------------------------------------------------------------------------------------------
**ROS INSTALLATION**

https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

------------------------------------------------------------------------------------------------------------------------------------------
**ROS Node**

Single-purpose, Executable program.
Individually compiled, executed and managed.
Organized in packages.

**Usually we run a node by the command**,
ros2 run package_name node_name

**To see the list of active nodes**, 
ros2 node list

------------------------------------------------------------------------------------------------------------------------------------------
**ROS MESSAGES**

Data structure defining the type of the topic,
 Comprised of a nested structure of integers, floats, strings etc. and array of objects.

Defined in *.msg files.
Message files type can be viewed in ROS folder in /opt directory

------------------------------------------------------------------------------------------------------------------------------------------
**ROS COMMUNICATION**

![image](https://github.com/user-attachments/assets/54a13549-3ad0-4575-bb9a-078ee406328a)

Topics

  - Asynchronous "stream-like" communication
  
  - Multiple Publisher and Subscriber
  
Services

  - Synchronous "function-call-like" communication
  
  - Single server, multiple clients
  
Actions

  - Built on top of topics
  
  - Supports Cancellation

------------------------------------------------------------------------------------------------------------------------------------------
**ROS TOPICS**

Nodes communicate over topics,

  - Nodes can publish or subscribe to a topic.
Topic is name for stream of messages

**To see the list of active topics**, 
ros2 topic list

**To see the data passing over the topic**,
ros2 topic echo /topic

------------------------------------------------------------------------------------------------------------------------------------------
**ROS SERVICES**

Request/ Response communication between nodes is realized with services.

 The service server advertises the service.
 The service client accesses the service.

Services are defined in *.srv files.
Service files type can we viewed in ROS folder in /opt directory.

------------------------------------------------------------------------------------------------------------------------------------------
**ROS PACKAGES**

ROS packages are the main unit of an ROS software framework.

A ROS package may contain executables, ROS dependent libraries, configuration files and so on.

ROS packages can be reused and shared.

------------------------------------------------------------------------------------------------------------------------------------------
**Ever wondered how everything is organised in ROS**

![image](https://github.com/user-attachments/assets/fb64bb6b-5ac2-45f0-b5aa-ec1f619a5324)

------------------------------------------------------------------------------------------------------------------------------------------
**ROS Parameter Server**

Nodes use parameter server to store and retrieve parameters at runtime.
Best used for static data such as configuration parameters.

Parameters can be defined in launch files or separate YAML files.
The user can set the privacy of the parameters  too.

------------------------------------------------------------------------------------------------------------------------------------------
**ROS BAGS**

A bag is a format for storing message data.                   
Binary format with file extension *.bag

Suited for logging and recording datasets for later visualization and analysis

------------------------------------------------------------------------------------------------------------------------------------------
![image](https://github.com/user-attachments/assets/4f0bcaf6-2149-4ee1-acba-d7d75af096c1)

------------------------------------------------------------------------------------------------------------------------------------------
**SOME BASIC KINEMATICS TERMS U SHOULD BE AWARE OF**

  - Pose
  - Velocity
  - Acceleration
  - Transformations

------------------------------------------------------------------------------------------------------------------------------------------
**LET'S START WITH TURTLESIM**

https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html


------------------------------------------------------------------------------------------------------------------------------------------
**PUBLISHER AND SUBSCRIBER SAMPLE PROGRAMMING**

https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html

------------------------------------------------------------------------------------------------------------------------------------------
**SERVICE AND CLIENT SAMPLE PROGRAMMING**

https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html

------------------------------------------------------------------------------------------------------------------------------------------

**Procedure to run the code from above links**

1. mkdir [workspace_name]/src -p       -----> To create a new workspace

2. cd [workspace_name]                 -----> Navigate inside the workspace through terminal

3. colcon build                        -----> Build the workspace to get the build, install and log folders

4. cd src                              -----> Navigate inside the src folder

5. ros2 pkg create --build-type ament_python [package_name]    -----> Create a new package

6. cd [package_name]/[package_name]    -----> Navigate inside the package

7. touch [program_name].py             -----> Create a new python file

8. Complete the code in the created file.

9. Open the package.xml file and write the executable dependencies used in the code. In our case,

    <exec_depend>rclpy</exec_depend>

    <exec_depend>std_msgs</exec_depend>

10. Open the setup.py file and give the path to our program file inside the console scripts. In our case,

    'talker = py_pubsub.publisher_:main',

11. cd ../../..                       -----> Navigate out to the workspace name   [Path should be just workspace_name in the terminal]

12. colcon build                      -----> Build the workspace to get the build, install and log folders.

13. source install/setup.bash         -----> Source the workspace in the terminal

14. ros2 run [package_name] [file]    -----> In our case [file] is talker. 

------------------------------------------------------------------------------------------------------------------------------------------
**Time to test what we learnt and do something cool**

1. Launch turtlesim node and write a simple publisher code to move the turtle with velocity command inputs.

2. Launch turtlesim node and write a simple suubscriber code to get the pose of the turtle spawned.

3. Spawn another turtle, now when u move the turtle1 with the teleop key make the second turtle to follow it.

------------------------------------------------------------------------------------------------------------------------------------------
**Odometry Sensors for Mobile Robots**

1. Wheel Encoders.
2. IMU (Inertial Measuring Unit).
3. LIDAR
4. Ultrasonic Sensor

------------------------------------------------------------------------------------------------------------------------------------------
**What is Sensor Fusion ?**

  - Kalman Filters
  - Extended Kalman filters
  - Unscented Kalman filters

------------------------------------------------------------------------------------------------------------------------------------------
**Localisation Techniques**

  - AMCL
  - GMCL
  - Particle filter

------------------------------------------------------------------------------------------------------------------------------------------
**SLAM Algorithms**

  - ORAB SLAM2
  - Cartographer SLAM
  - Gmapping
  - Hector SLAM

------------------------------------------------------------------------------------------------------------------------------------------
**Navigation Keywords**

  - Local Costmap
  - Global Costmap
  - Local Planner
  - Global Planner

------------------------------------------------------------------------------------------------------------------------------------------
**Gazebo Vs RViz, When and Why**

Gazebo is a realistic physics-based simulator that allows one to test robots in virtual environment.

RViz is a 3D visualization tool for ROS-based robots. It does not simulate the physics but visualises robot state, TFs, Sensor data and Planned paths etc. 

------------------------------------------------------------------------------------------------------------------------------------------
**LET'S UNDERSTAND A COMPLETE AUTONOMOUS MOBILE USING TURTLEBOT3 SIMULATIONS**

**Follow the commands given below,**

sudo apt install ros-humble-turtlebot3* -y

mkdir -p ~/turtlebot3_ws/src

cd ~/turtlebot3_ws/src/

git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git

git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git

git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git

sudo apt install python3-colcon-common-extensions

git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

cd ~/turtlebot3_ws && colcon build --symlink-install

export TURTLEBOT3_MODEL=burger

ros2 launch turtlebot3_gazebo empty_world.launch.py

export TURTLEBOT3_MODEL=waffle

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

export TURTLEBOT3_MODEL=waffle_pi

ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py

ros2 run turtlebot3_teleop teleop_keyboard

------------------------------------------------------------------------------------------------------------------------------------------

**IMPLEMENTATION OF SLAM**

**Terminal 1**

export TURTLEBOT3_MODEL=burger

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

**Terminal 2**

export TURTLEBOT3_MODEL=burger

ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True

**Terminal 3**

export TURTLEBOT3_MODEL=burger

ros2 run turtlebot3_teleop teleop_keyboard

ros2 run nav2_map_server map_saver_cli -f ~/map

------------------------------------------------------------------------------------------------------------------------------------------

**NAVIGATION OF THE TURTLEBOT**

**Terminal 1**

export TURTLEBOT3_MODEL=burger

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

**Terminal 2**

export TURTLEBOT3_MODEL=burger

ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map.yaml

**To efficiently localise the mobile robot, we have to move the robot to shrink the probablistic points of the location of robot**

ros2 run turtlebot3_teleop teleop_keyboard

------------------------------------------------------------------------------------------------------------------------------------------

**Autonomous Implementation of Turtlebot**

Click on the 2D pose estimate option and select the desired location and orientation of the robot and watch the robot do its magic.

------------------------------------------------------------------------------------------------------------------------------------------



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
**To make a new directory-->** mkdir

**To build a ROS workspace-->** colcon build

------------------------------------------------------------------------------------------------------------------------------------------
**PUBLISHER AND SUBSCRIBER SAMPLE PROGRAMMING**

https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html

------------------------------------------------------------------------------------------------------------------------------------------
**SERVICE AND CLIENT SAMPLE PROGRAMMING**

https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html

------------------------------------------------------------------------------------------------------------------------------------------
**Time to test what we learnt and do something cool**

1. Launch turtlesim node and write a simple publisher code to move the turtle with velocity command inputs.

2. Launch turtlesim node and write a simple suubscriber code to get the pose of the turtle spawned.

3. Spawn another turtle, now when u move the turtle1 with the teleop key make the second turtle to follow it.

------------------------------------------------------------------------------------------------------------------------------------------

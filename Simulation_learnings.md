The Transform System (tf2)
==========================

What is tf2
-----------

tf2 is the transform system used in ROS 2.\
It keeps track of multiple coordinate frames over time and helps us understand where different parts of a robot are in space at any given moment.

Why do we need tf2?
-------------------

Every sensor and actuator on a robot works in its own coordinate frame.\
For example:

-   The camera gives data in camera_link frame

-   The lidar gives data in lidar_link frame

-   But we usually want to understand everything in terms of a single frame like base_link or map

tf2 helps us know where these parts are relative to each other, and also how they move over time.

Important Concepts
------------------

**Frame** -- A coordinate system (like base_link, map, odom, camera_link)\
**Transform** -- A spatial relationship between two frames\
**Transform Tree** -- All the transforms form a tree structure, with one root (usually map or odom)

tf2 Main Components
-------------------

-   **tf2_ros::Buffer** -- Stores transform data

-   **tf2_ros::TransformListener** -- Listens to transform data and fills the buffer

-   **tf2_ros::TransformBroadcaster** -- Used by nodes to publish dynamic transforms

-   **tf2_ros::StaticTransformBroadcaster** -- Used to publish static (unchanging) transforms

-   **tf2_geometry_msgs** -- Provides methods to transform poses and vectors between frames

How it works
------------

1.  A node publishes transforms between two frames\
    (e.g., base_link → camera_link)

2.  Another node can request this transform from the buffer\
    and convert pose or data from one frame to another

Example:\
If we want to know where the camera was relative to the base_link, we ask the buffer for the latest transform.

Static vs Dynamic Transforms
----------------------------

Static transforms don't change over time. Example: camera is always mounted 0.1m above the base.

Dynamic transforms change with time. Example: robot moving in the map, so base_link to odom keeps updating.

Common Tools for tf2
--------------------

-   `view_frames.py` -- creates a PDF image of the transform tree

-   `static_transform_publisher` -- command-line tool to publish a fixed transform

-   `tf2_echo` -- prints out transform between two frames in real time

Things to Remember
------------------

-   Always make sure your transforms are being published continuously (usually at 10Hz or more)

-   Use `rclcpp::Time(0)` if Wewant the latest available transform

-   tf2 keeps transform data only for a short duration (usually ~10 seconds)

-   The transform tree must not have any cycles. It must be a tree (not a looped graph)

URDF (Unified Robot Description Format)
=======================================

What is URDF?
-------------

URDF stands for Unified Robot Description Format.\
It is an XML-based format used in ROS to describe a robot's physical structure.\
This includes the robot's links, joints, sensors, visuals, collisions, and inertial properties.

Why URDF?
---------

ROS needs to know the structure of your robot so it can simulate motion, publish transforms, and use sensors properly.\
URDF gives a standard way to define the robot's parts and how they are connected.

Basic Structure of a URDF file
------------------------------

A URDF file is basically a set of XML tags that define:

1.  Links -- The rigid parts of the robot

2.  Joints -- The connections between links (they define motion)

3.  Sensors -- Like cameras or LIDARs (can be attached as plugins)

4.  Visuals -- What the robot looks like (can use meshes or basic shapes)

5.  Collision -- Simplified shapes used for detecting collisions

6.  Inertials -- Mass, center of mass, and inertia matrix

Important Tags in URDF
----------------------

-   `<robot name="robot_name">` -- Root tag of the URDF

-   `<link name="link_name">` -- Describes one rigid part

-   `<joint name="joint_name" type="type">` -- Connects two links

-   `<visual>` -- How the link looks (geometry + material)

-   `<collision>` -- Defines collision shape

-   `<inertial>` -- Physical properties (mass, inertia, origin)

-   `<origin>` -- Position and orientation (xyz and rpy)

-   `<geometry>` -- Shape (box, cylinder, sphere, or mesh)

Joint Types
-----------

-   **fixed** -- No motion (like base_link to chassis)

-   **revolute** -- Rotational joint with limits (like elbow)

-   **continuous** -- Like revolute but infinite rotation (like wheels)

-   **prismatic** -- Linear movement (like telescopic arm)

-   **floating** -- 6 DOF (used in simulation)

-   **planar** -- 2D motion in a plane

Common Practice
---------------

-   Start from base_link (robot's root frame)

-   Connect all links using joints

-   Define visuals with simple shapes (box, cylinder) or use .dae/.stl meshes

-   Use proper origin tags to place each part correctly

-   Add inertials if Wewant to use physics in Gazebo or other simulators

URDF vs XACRO
-------------

URDF can get repetitive and hard to manage for big robots.\
XACRO (XML Macro) is an extended format that allows macros, variables, includes etc.\
Use `.xacro` files to make your robot description more modular and readable.

Example: instead of copying same wheel code 4 times, define it once as a macro and reuse.

URDF Visualization
------------------

-   `ros2 run robot_state_publisher robot_state_publisher` -- Publishes TFs based on URDF

-   `ros2 run rviz2 rviz2` -- To see the robot model

-   `ros2 launch your_pkg your_urdf_launch_file.py` -- Launch with RViz and state publisher

Errors I faced very often
-------------------------

Forgetting origin tags → parts not positioned correctly

Wrong joint types → simulation behaves unexpectedly

No inertial → simulation works but physics is not accurate

Mesh path issues → robot appears blank in Rviz

Simulating with Gazebo in ROS 2
===============================

What is Gazebo?
---------------

Gazebo is a 3D simulation tool used with ROS to test robots in a virtual environment.\
It simulates physics (gravity, friction, collisions), sensors (LIDAR, camera), and environments (walls, objects).\
It helps us test our robot's URDF and behavior **without needing real hardware**.

Why use Gazebo?
---------------

-   Wecan test robot movement, sensor data, obstacle avoidance, etc.

-   Saves time and cost -- no need for physical setup.

-   Integrates tightly with ROS 2 using plugins.

-   Ideal for debugging URDF, tf2, navigation stacks, and control systems.

How Gazebo Works with ROS 2
---------------------------

1.  Wewrite your robot in URDF or XACRO

2.  Add Gazebo plugins (e.g., diff_drive, sensors) inside the URDF

3.  Launch Gazebo with the robot model and world

4.  Use ROS 2 nodes to control and interact with the robot

Required Packages
-----------------

Make sure these packages are installed:

-   `gazebo_ros`

-   `ros_gz` or `ros_ign` if you're using Ignition Gazebo

-   `ros2_control` (optional, for joint control)

Common Workflow
---------------

1.  Create a URDF or XACRO for your robot

2.  Add Gazebo-specific plugins and tags (e.g., `gazebo_ros_diff_drive`)

3.  Create a launch file to start Gazebo and spawn the robot

4.  Use RViz or ROS 2 topics to control and visualize the robot

5.  Add obstacles in the world file if needed

Important Plugins
-----------------

-   `gazebo_ros_diff_drive` -- Simulates differential drive movement

-   `gazebo_ros_camera` -- Publishes camera images to a ROS 2 topic

-   `gazebo_ros_ray_sensor` -- For LIDAR/laser scans

-   `gazebo_ros_control` -- Enables ros2_control interfaces

-   `gazebo_ros_joint_state_publisher` -- Publishes joint states

Spawning the Robot
------------------

Usually done in the launch file using:

```
Node(
  package='gazebo_ros',
  executable='spawn_entity.py',
  arguments=['-topic', 'robot_description', '-entity', 'my_robot'],
)

```

This takes your robot description from `/robot_description` and places it in the Gazebo world.

Simulating Sensors
------------------

-   Use plugins like `gazebo_ros_camera` or `ray_sensor`

-   We can simulate RGB cameras, depth sensors, IMU, GPS, LIDAR, etc.

-   Each sensor publishes to a ROS 2 topic just like a real robot would

Creating a World
----------------

Gazebo uses `.world` files to define the environment.\
We can create custom worlds with models, lights, terrain, etc.\
Default empty world: just a ground plane and some lighting

Launching Everything Together
-----------------------------

We create a launch file that:

-   Starts Gazebo

-   Loads the robot URDF

-   Spawns the robot in the world

-   Starts state publishers and controllers

Useful Tools and Commands
-------------------------

-   `ros2 launch gazebo_ros gazebo.launch.py` -- Starts Gazebo

-   `gzclient` -- Opens Gazebo GUI

-   `ros2 topic list` -- See all topics including sensor data

-   `ros2 run rviz2 rviz2` -- Visualize TF, LIDAR, camera in RViz

-   `ros2 service call /spawn_entity` -- Alternative way to spawn robots

Common Errors
-------------

-   Robot not spawning → check URDF errors or plugin paths

-   Robot floating or sinking → fix mass/inertial values

-   Wheels not moving → check `gazebo_ros_diff_drive` config

-   Sensors not publishing → check plugin topics and names



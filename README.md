# ROS2_Humble
# ROS 2 Humble Documentation 

This repository contains all my learnings and hands-on documentation from online research on ROS 2 Humble as a software intern in the MaRS Club aimed at building a strong foundation in robotics.

## Learnings

This is a overview of whatever I have learnt till now.

1. **Install and Setup ROS2 Humble**
   - System update and upgrade
   - Installing dependencies
   - Adding ROS 2 repository and GPG key
   - Installing ROS 2 Humble packages
   - Sourcing the ROS 2 environment

2. **Start Your First ROS2 Node**
   - Launching `turtlesim_node`
   - Launching `turtle_teleop_key` for keyboard control
   - Listing active nodes and topics
   - Echoing topic messages
   - Calling services

3. **Create and Set Up a ROS2 Workspace**
   - Creating a workspace directory
   - Initializing the workspace
   - Building the workspace
   - Sourcing the workspace

4. **Create Your First ROS2 Package**
   - Using `ros2 pkg create` command
   - Understanding package structure
   - Adding dependencies
   - Building the package

5. **Create Your First ROS2 Node**
   - Writing a simple Python node
   - Using `rclpy` and `Node` class
   - Adding the node to `setup.py`
   - Running the node

6. **Create Your First ROS2 Publisher**
   - Creating a publisher node
   - Publishing messages to a topic
   - Setting publish frequency
   - Observing messages with `ros2 topic echo`

7. **Create Your First ROS2 Subscriber**
   - Creating a subscriber node
   - Subscribing to a topic
   - Processing incoming messages
   - Combining publisher and subscriber

8. **Create Your First ROS2 Service**
   - Defining a service interface
   - Implementing a service server
   - Implementing a service client
   - Calling the service and handling responses

9. **Create Your First ROS2 Action Server**
   - Defining an action interface
   - Implementing an action server
   - Handling goals, feedback, and results

10. **Create Your First ROS2 Action Client**
    - Implementing an action client
    - Sending goals to the action server
    - Receiving feedback and results

## Prerequisites

- Ubuntu 22.04 
- ROS 2 Humble installed
- Basic knowledge of Python and C++
- Familiarity with terminal commands

## Repository Structure

Organize your workspace as follows:

```
ros2_ws/
├── src/
│   ├── my_robot_controller/                 # ROS 2 Python package with custom nodes
│   │   ├── launch/
│   │   │   └── start_all_nodes.py           # Launch all controller nodes
│   │   ├── my_robot_controller/
│   │   │   ├── __init__.py
│   │   │   ├── my_first_node.py             # Basic custom node
│   │   │   ├── number_publisher.py          # Publishes numbers (QoS enabled)
│   │   │   ├── number_subscriber.py         # Subscribes to numbers (QoS enabled)
│   │   │   ├── pose_subscriber.py           # Subscribes to /pose topic
│   │   │   └── draw_circle.py               # Publishes circular movement
│   │   ├── setup.py
│   │   └── package.xml
│
│   ├── 4wd_robot_sim/                       # Simulation package: Gazebo + RViz + URDF
│   │   ├── launch/
│   │   ├── urdf/                            # 4WD robot with camera & LIDAR
│   │   ├── obstacle_stop/                   # Node for obstacle avoidance using LIDAR
│   │   ├── rviz/, worlds/, ...
│   │   ├── setup.py, package.xml, ...
│
│   └── 4wd_robot_control/                   # Control package (modular logic)
│       ├── 4wd_robot_control/
│       ├── setup.py, package.xml, ...
│
├── build/                                   # colcon build files (auto-generated)
├── install/                                 # colcon install (auto-generated)
└── log/                                     # Build logs
```
## Question: Create a publisher node that sends a number, and a subscriber node that receives this number, calculates its square, and displays the result in the terminal.


**Publisher Node**
```bash
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class NumberPublisher(Node):
    def __init__(self):
        super().__init__('number_publisher')
        self.publisher_ = self.create_publisher(Int32, 'number_topic', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_number)
        self.number = 1

    def publish_number(self):
        msg = Int32()
        msg.data = self.number
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.number += 1

def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

```



**Subscriber Node**
```bash
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class NumberSubscriber(Node):
    def __init__(self):
        super().__init__('number_subscriber')
        self.subscription = self.create_subscription(
            Int32,
            'number_topic',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        square = msg.data ** 2
        self.get_logger().info(f'Received: {msg.data}, Square: {square}')

def main(args=None):
    rclpy.init(args=args)
    node = NumberSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

```

## Launching Nodes Using Launch File

You can launch all your publisher and subscriber nodes together using the launch file:

```bash
ros2 launch my_robot_controller start_all_nodes.py
```

### Quality of Service (QoS)

Both the publisher and subscriber nodes now use a custom QoS profile to ensure reliable message delivery:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    depth=10
)
```

## Useful Commands

- Update and upgrade system:

  ```bash
  sudo apt update && sudo apt upgrade -y
  ```

- Install dependencies:

  ```bash
  sudo apt install -y software-properties-common curl
  ```

- Add ROS 2 repository and GPG key:

  ```bash
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  ```

- Install ROS 2 Humble:

  ```bash
  sudo apt update
  sudo apt install -y ros-humble-desktop
  ```

- Source ROS 2 environment:

  ```bash
  source /opt/ros/humble/setup.bash
  ```

- Create and build workspace:

  ```bash
  mkdir -p ~/ros2_ws/src
  cd ~/ros2_ws
  colcon build
  source install/setup.bash
  ```

- Create a new package:

  ```bash
  cd ~/ros2_ws/src
  ros2 pkg create --build-type ament_python my_package
  ```

- Run a node:

  ```bash
  ros2 run my_package publisher_node
  ```

- List active nodes:

  ```bash
  ros2 node list
  ```

- List topics:

  ```bash
  ros2 topic list
  ```

- Echo topic messages:

  ```bash
  ros2 topic echo /topic_name
  ```

- Call a service:

  ```bash
  ros2 service call /service_name std_srvs/srv/Empty
  ```

  ## Additional files

📄 [ROS 1 vs ROS 2 - Key Differences](./ROS1_VS_ROS2.md)


## Additional Resources

- (https://docs.ros.org/en/humble/index.html)
- (https://articulatedrobotics.xyz/tutorials/)
- (https://www.google.com/url?sa=t&source=web&rct=j&opi=89978449&url=https://www.theconstruct.ai/ros-for-beginners-how-to-learn-ros/&ved=2ahUKEwj_obTulKWNAxUKb2wGHX_yPO8QjJEMegQIBRAC&usg=AOvVaw1E4IPsFpUXhT78h5zZ9A1T)

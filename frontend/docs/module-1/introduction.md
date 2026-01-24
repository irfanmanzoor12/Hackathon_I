---
sidebar_position: 1
title: Introduction to ROS 2
---

<ChapterActions />

# Module 1: The Robotic Nervous System (ROS 2)

## Welcome to the World of Robot Operating System

Just as the human nervous system coordinates communication between the brain and body, **ROS 2** (Robot Operating System 2) serves as the middleware that enables distributed processes in a robot to communicate seamlessly.

## 🎯 Module Goals

By the end of this module, you will:

- Understand the architecture and design philosophy of ROS 2
- Create ROS 2 nodes, topics, and services
- Bridge Python AI agents to ROS controllers using rclpy
- Describe robots using URDF (Unified Robot Description Format)
- Build a complete ROS 2 package for humanoid robot control

## 📖 What is ROS 2?

**ROS 2** is not an operating system—it's a **middleware framework** that provides:

- **Communication Infrastructure**: Publish-subscribe messaging, services, actions
- **Hardware Abstraction**: Unified interface for sensors and actuators
- **Package Management**: Modular, reusable components
- **Tools & Libraries**: Visualization, simulation, debugging

### Why ROS 2 (Not ROS 1)?

ROS 2 brings critical improvements:

| Feature | ROS 1 | ROS 2 |
|---------|-------|-------|
| **Real-time Support** | Limited | Full RTOS support |
| **Security** | None | DDS security plugins |
| **Multi-robot** | Difficult | Native support |
| **Communication** | Custom | DDS standard |
| **Windows/Mac** | Poor | Full support |
| **Production Ready** | Research | Industry-grade |

## 🏗️ ROS 2 Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    Application Layer                    │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐             │
│  │  Node A  │  │  Node B  │  │  Node C  │             │
│  │ (Python) │  │  (C++)   │  │ (Python) │             │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘             │
└───────┼─────────────┼─────────────┼────────────────────┘
        │             │             │
┌───────┴─────────────┴─────────────┴────────────────────┐
│               ROS 2 Client Libraries                    │
│         (rclpy, rclcpp, rclnodejs, etc.)               │
└────────────────────────┬────────────────────────────────┘
                         │
┌────────────────────────┴────────────────────────────────┐
│                 ROS Middleware (rmw)                    │
│            Abstraction over DDS implementations         │
└────────────────────────┬────────────────────────────────┘
                         │
┌────────────────────────┴────────────────────────────────┐
│                        DDS                              │
│     (Data Distribution Service - OMG Standard)          │
│   ┌──────────┐  ┌──────────┐  ┌──────────┐            │
│   │ Fast DDS │  │ Cyclone  │  │ Connext  │            │
│   │          │  │   DDS    │  │   DDS    │            │
│   └──────────┘  └──────────┘  └──────────┘            │
└─────────────────────────────────────────────────────────┘
```

## 🔑 Core Concepts

### 1. Nodes
**Definition**: A node is a participant in the ROS 2 graph—a single, modular purpose.

**Examples**:
- Camera driver node (publishes images)
- Motion planner node (computes paths)
- Motor controller node (sends commands)

```python
import rclpy
from rclpy.node import Node

class MyRobotNode(Node):
    def __init__(self):
        super().__init__('my_robot_node')
        self.get_logger().info('Robot node started!')

def main():
    rclpy.init()
    node = MyRobotNode()
    rclpy.spin(node)
    rclpy.shutdown()
```

### 2. Topics
**Definition**: Named channels for asynchronous message streaming (publish-subscribe pattern).

**Use Case**: Streaming sensor data (camera, LiDAR, IMU)

```python
from sensor_msgs.msg import Image

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher = self.create_publisher(Image, 'camera/image', 10)
        self.timer = self.create_timer(0.1, self.publish_image)

    def publish_image(self):
        msg = Image()
        # Fill image data
        self.publisher.publish(msg)
        self.get_logger().info('Published image')
```

### 3. Services
**Definition**: Synchronous request-response communication.

**Use Case**: Triggering actions (start/stop, reset, compute trajectory)

```python
from std_srvs.srv import Trigger

class ServiceProvider(Node):
    def __init__(self):
        super().__init__('service_provider')
        self.srv = self.create_service(Trigger, 'reset_robot', self.reset_callback)

    def reset_callback(self, request, response):
        # Reset robot state
        response.success = True
        response.message = 'Robot reset successfully'
        return response
```

### 4. Actions
**Definition**: Long-running tasks with feedback (combination of topics and services).

**Use Case**: Navigation to a goal, grasping an object

```python
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose

# Client code
goal_msg = NavigateToPose.Goal()
goal_msg.pose.header.frame_id = 'map'
goal_msg.pose.pose.position.x = 2.0
goal_msg.pose.pose.position.y = 1.0
```

## 🛠️ ROS 2 Distributions

ROS 2 releases follow a time-based schedule with LTS (Long-Term Support) versions:

| Distribution | Release Date | Ubuntu | Python | Status |
|--------------|--------------|--------|--------|---------|
| **Humble** | May 2022 | 22.04 | 3.10 | **LTS** (Recommended) |
| Iron | May 2023 | 22.04 | 3.10 | Active |
| Jazzy | May 2024 | 24.04 | 3.12 | Latest |

**For this course, we use ROS 2 Humble Hawksbill.**

## 🧩 ROS 2 Packages

A **package** is the organizational unit of ROS 2 code:

```
my_robot_package/
├── CMakeLists.txt          # Build configuration (C++)
├── package.xml             # Package metadata
├── setup.py                # Python setup (if Python package)
├── my_robot_package/       # Python module
│   ├── __init__.py
│   ├── node1.py
│   └── node2.py
├── launch/                 # Launch files
│   └── robot_launch.py
├── config/                 # Configuration files
│   └── params.yaml
└── urdf/                   # Robot descriptions
    └── robot.urdf
```

## 🎬 Your First ROS 2 Node

Let's create a simple "Hello, Physical AI!" node:

```python
# File: hello_physical_ai.py
import rclpy
from rclpy.node import Node

class HelloPhysicalAI(Node):
    def __init__(self):
        super().__init__('hello_physical_ai')
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        self.get_logger().info(f'Hello, Physical AI! Count: {self.count}')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = HelloPhysicalAI()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Run it**:
```bash
python3 hello_physical_ai.py
```

**Output**:
```
[INFO] [hello_physical_ai]: Hello, Physical AI! Count: 0
[INFO] [hello_physical_ai]: Hello, Physical AI! Count: 1
[INFO] [hello_physical_ai]: Hello, Physical AI! Count: 2
```

## 📊 Module Roadmap

### Week 3: ROS 2 Fundamentals
- Installation and workspace setup
- Understanding the computation graph
- Creating your first nodes

### Week 4: Communication Patterns
- Topics for data streaming
- Services for request-response
- Actions for long-running tasks

### Week 5: Python Integration
- rclpy client library
- Bridging Python AI code to ROS
- Custom message types

## 🎯 Module Project Preview

**Goal**: Build a ROS 2 package that controls a simulated humanoid's head to track faces.

**Components**:
- Face detection node (Python + OpenCV)
- Head controller node (publishes joint commands)
- Visualization in RViz2

**Skills Applied**:
- Node creation
- Topic publishing/subscribing
- Custom messages
- Launch files

## 💡 Why This Matters for Humanoid Robotics

Humanoid robots are complex systems with:
- **20-40+ degrees of freedom** (joints)
- **Multiple sensor types** (cameras, IMUs, force sensors)
- **Real-time control requirements** (balance, walking)
- **Distributed processing** (vision on GPU, control on CPU)

**ROS 2 provides the communication backbone that makes this coordination possible.**

## 🚀 Next Steps

Ready to dive deeper? The next chapter covers:
- Installing ROS 2 on Ubuntu
- Setting up your workspace
- Running your first official ROS 2 examples

Click "Next" to continue! →

---

**Questions?** Use the chatbot at the bottom-right to ask anything about ROS 2! You can also:
- **Personalize** this content based on your programming background
- **Translate** to Urdu for better understanding

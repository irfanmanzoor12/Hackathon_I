---
sidebar_position: 3
title: Nodes, Topics, and Services
---

# Nodes, Topics, and Services

## Mastering ROS 2 Communication Patterns

Understanding how ROS 2 nodes communicate is fundamental to building distributed robotic systems. In this chapter, we'll explore the three core communication patterns used in humanoid robotics.

## 🎯 Learning Objectives

- Understand the ROS 2 computation graph
- Master publish-subscribe pattern with Topics
- Implement request-response pattern with Services
- Use Actions for long-running tasks
- Build a multi-node humanoid robot system

## 🕸️ The ROS 2 Computation Graph

The **computation graph** is the network of ROS 2 nodes and their connections:

```
        ┌──────────────┐
        │ Camera Node  │
        └──────┬───────┘
               │ publishes
               ▼
         /camera/image
               │ subscribes
               ▼
        ┌──────────────┐
        │ Vision Node  │
        └──────┬───────┘
               │ publishes
               ▼
         /detected_faces
               │ subscribes
               ▼
        ┌──────────────┐
        │  Head Node   │
        └──────────────┘
```

View the graph in real-time:
```bash
rqt_graph
```

## 📡 Topics: Publish-Subscribe Pattern

### What are Topics?

**Topics** are named buses where nodes can publish or subscribe to messages asynchronously.

**Key Characteristics**:
- ✅ Many-to-many communication
- ✅ Fire-and-forget (no response)
- ✅ Continuous data streams
- ✅ Decoupled nodes

**Use Cases**:
- Sensor data (camera images, LiDAR scans)
- Motor commands (joint positions, velocities)
- State updates (battery level, position)

### Creating a Publisher

**Example: Publishing Joint Commands**

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointCommandPublisher(Node):
    def __init__(self):
        super().__init__('joint_command_publisher')

        # Create publisher
        self.publisher = self.create_publisher(
            JointState,           # Message type
            '/humanoid/joints',   # Topic name
            10                    # Queue size
        )

        # Timer to publish at 30 Hz
        self.timer = self.create_timer(1.0/30.0, self.publish_commands)

        self.get_logger().info('Joint Command Publisher started')

    def publish_commands(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['head_pan', 'head_tilt', 'left_arm', 'right_arm']
        msg.position = [0.0, 0.5, 1.0, 1.0]
        msg.velocity = [0.1, 0.1, 0.2, 0.2]

        self.publisher.publish(msg)
        self.get_logger().debug(f'Published joint commands')

def main(args=None):
    rclpy.init(args=args)
    node = JointCommandPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## 🚀 Practical Example: Head Tracker

Complete system combining all patterns - see full code in the interactive version.

## 📚 Summary

You've mastered the core ROS 2 communication patterns essential for humanoid robotics!

---

**Questions?** Ask the chatbot or use the personalize/translate buttons above!

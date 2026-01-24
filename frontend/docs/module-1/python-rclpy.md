---
sidebar_position: 4
title: Python & rclpy
---

# Python Integration with rclpy

## Bridging AI and Robotics with Python

Python is the dominant language for AI/ML, and **rclpy** (ROS Client Library for Python) bridges your AI models with robot control systems.

## 🎯 Learning Objectives

- Master rclpy for Python-based ROS 2 nodes
- Integrate ML models with robot controllers
- Handle parameters and launch files
- Build production-ready robot code

## 🐍 Why Python for Robotics?

**Advantages**:
- ✅ Rapid prototyping
- ✅ Rich AI/ML ecosystem (PyTorch, TensorFlow, OpenCV)
- ✅ Easy integration with sensors
- ✅ Readable, maintainable code

**Trade-offs**:
- ⚠️ Slower than C++ (but good enough for most tasks)
- ⚠️ GIL limitations (use multiprocessing if needed)

## 🚀 rclpy Basics

### Minimal Node Template

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Node started!')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()
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

### Publishing & Subscribing

```python
from std_msgs.msg import String

class PubSubNode(Node):
    def __init__(self):
        super().__init__('pubsub_node')

        # Publisher
        self.pub = self.create_publisher(String, 'output', 10)

        # Subscriber
        self.sub = self.create_subscription(
            String, 'input', self.callback, 10)

        # Timer for periodic publishing
        self.timer = self.create_timer(1.0, self.timer_callback)

    def callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello from Python!'
        self.pub.publish(msg)
```

## 🤖 AI Integration Example: Face Tracking

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge
import cv2
import numpy as np

class FaceTracker(Node):
    def __init__(self):
        super().__init__('face_tracker')

        # CV Bridge for image conversion
        self.bridge = CvBridge()

        # Load Haar Cascade for face detection
        self.face_cascade = cv2.CascadeClassifier(
            cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        )

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publishers
        self.joint_pub = self.create_publisher(
            JointState,
            '/humanoid/head_joints',
            10
        )

        self.get_logger().info('Face Tracker initialized')

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Detect faces
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(
            gray, 1.3, 5
        )

        if len(faces) > 0:
            # Track first detected face
            x, y, w, h = faces[0]

            # Calculate center
            face_center_x = x + w/2
            face_center_y = y + h/2

            # Convert to joint angles
            image_width = cv_image.shape[1]
            image_height = cv_image.shape[0]

            # Normalize to [-1, 1]
            pan = (face_center_x / image_width - 0.5) * 2
            tilt = (face_center_y / image_height - 0.5) * 2

            # Publish joint commands
            self.publish_joint_command(pan, tilt)

    def publish_joint_command(self, pan, tilt):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['head_pan', 'head_tilt']
        msg.position = [pan * 0.7, tilt * 0.5]  # Scale to joint limits

        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FaceTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## ⚙️ Parameters

Parameters allow runtime configuration without code changes:

```python
class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with defaults
        self.declare_parameter('update_rate', 30.0)
        self.declare_parameter('robot_name', 'humanoid_01')
        self.declare_parameter('enable_tracking', True)

        # Get parameter values
        self.update_rate = self.get_parameter('update_rate').value
        self.robot_name = self.get_parameter('robot_name').value
        self.tracking_enabled = self.get_parameter('enable_tracking').value

        self.get_logger().info(
            f'Parameters: rate={self.update_rate}, '
            f'name={self.robot_name}, tracking={self.tracking_enabled}'
        )

        # Set up timer with parameter
        self.timer = self.create_timer(
            1.0 / self.update_rate,
            self.timer_callback
        )

    def timer_callback(self):
        # Re-read parameters (they can change at runtime)
        self.tracking_enabled = self.get_parameter('enable_tracking').value
        if self.tracking_enabled:
            self.get_logger().info('Tracking...')
```

Set parameters from command line:
```bash
ros2 run pkg node --ros-args -p update_rate:=60.0 -p robot_name:=test_bot
```

Or from YAML file (`config/params.yaml`):
```yaml
parameter_node:
  ros__parameters:
    update_rate: 60.0
    robot_name: "humanoid_02"
    enable_tracking: true
```

Load with launch file:
```python
parameters=[{'path/to/params.yaml'}]
```

## 🎬 Launch Files in Python

Launch files start multiple nodes with configuration:

```python
# launch/robot_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='humanoid_01',
        description='Name of the robot'
    )

    # Get launch arguments
    robot_name = LaunchConfiguration('robot_name')

    return LaunchDescription([
        robot_name_arg,

        # Camera node
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='camera',
            parameters=[{
                'video_device': '/dev/video0',
                'framerate': 30.0
            }],
            output='screen'
        ),

        # Face tracker
        Node(
            package='humanoid_control',
            executable='face_tracker',
            name='face_tracker',
            parameters=[{
                'robot_name': robot_name,
                'update_rate': 30.0
            }],
            output='screen'
        ),

        # Head controller
        Node(
            package='humanoid_control',
            executable='head_controller',
            name='head_controller',
            output='screen'
        ),
    ])
```

Run with:
```bash
ros2 launch humanoid_control robot_launch.py robot_name:=test_bot
```

## 🔧 Best Practices

### 1. Logging Levels
```python
self.get_logger().debug('Detailed info')
self.get_logger().info('General info')
self.get_logger().warn('Warning')
self.get_logger().error('Error occurred')
self.get_logger().fatal('Critical failure')
```

### 2. Graceful Shutdown
```python
def main():
    rclpy.init()
    node = MyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down gracefully...')
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

### 3. Timer Management
```python
# Don't create timers in callbacks!
# Create once in __init__:
self.timer = self.create_timer(0.1, self.callback)

# Cancel when done:
self.timer.cancel()
```

### 4. Threading
```python
from rclpy.executors import MultiThreadedExecutor

executor = MultiThreadedExecutor()
executor.add_node(node1)
executor.add_node(node2)
executor.spin()
```

## 📊 Performance Tips

1. **Use appropriate QoS** for your data type
2. **Avoid blocking operations** in callbacks
3. **Use timers** instead of `while` loops
4. **Profile with** `ros2 topic hz` and `ros2 topic bw`
5. **Consider C++** for high-frequency control loops

## 🎓 Exercise: Build a Gesture Controller

Create a node that:
1. Subscribes to hand pose detection
2. Maps gestures to robot commands
3. Publishes joint positions
4. Uses parameters for sensitivity

## 📚 Summary

You've learned:
- ✅ rclpy fundamentals
- ✅ Integrating AI/ML with ROS 2
- ✅ Parameters and launch files
- ✅ Production code patterns

## 🚀 Next

Learn how to describe your robot's physical structure with **URDF**!

---

**Personalize** this chapter for your programming level or **Translate** to Urdu!

---
sidebar_position: 2
title: ROS 2 Fundamentals
---

<ChapterActions />

# ROS 2 Fundamentals

## Deep Dive into the Robot Operating System

Now that you understand what ROS 2 is, let's explore its fundamental architecture and learn how to install and configure it for humanoid robotics development.

## 🔧 Installation Guide

### System Requirements

- **OS**: Ubuntu 22.04 LTS (Jammy Jellyfish)
- **ROS 2 Distribution**: Humble Hawksbill (LTS)
- **Disk Space**: Minimum 5GB
- **Internet**: Required for package installation

### Step-by-Step Installation

#### 1. Set Locale
```bash
locale  # check UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

#### 2. Add ROS 2 Repository
```bash
# Enable Ubuntu Universe repository
sudo apt install software-properties-common
sudo add-apt-repository universe

# Add ROS 2 GPG key
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

#### 3. Install ROS 2 Humble
```bash
# Update package index
sudo apt update

# Install ROS 2 Desktop (recommended)
sudo apt install ros-humble-desktop -y

# Or install ROS 2 Base (minimal)
# sudo apt install ros-humble-ros-base -y
```

#### 4. Install Development Tools
```bash
# Install rosdep for dependency management
sudo apt install python3-rosdep python3-colcon-common-extensions -y

# Initialize rosdep
sudo rosdep init
rosdep update
```

#### 5. Setup Environment
Add to your `~/.bashrc`:
```bash
# ROS 2 Humble
source /opt/ros/humble/setup.bash

# Optional: Autocomplete
eval "$(register-python-argcomplete3 ros2)"
eval "$(register-python-argcomplete3 colcon)"
```

Apply changes:
```bash
source ~/.bashrc
```

### Verify Installation
```bash
# Check ROS 2 version
ros2 --version

# Test with demo nodes
# Terminal 1:
ros2 run demo_nodes_cpp talker

# Terminal 2:
ros2 run demo_nodes_py listener
```

**Expected Output**:
- Terminal 1: Publishing "Hello World" messages
- Terminal 2: Receiving and displaying those messages

## 🏗️ Workspace Setup

A **ROS 2 workspace** is a directory structure for organizing your packages.

### Create Workspace
```bash
# Create workspace directory
mkdir -p ~/humanoid_ws/src
cd ~/humanoid_ws

# Build workspace (even if empty)
colcon build

# Source the workspace
source install/setup.bash
```

### Workspace Structure
```
humanoid_ws/
├── build/          # Build artifacts (auto-generated)
├── install/        # Installed packages (auto-generated)
├── log/            # Build logs (auto-generated)
└── src/            # Source code (you work here!)
    ├── package_1/
    ├── package_2/
    └── package_3/
```

## 📦 Creating Your First Package

### Python Package (Pure Python)
```bash
cd ~/humanoid_ws/src

# Create package
ros2 pkg create humanoid_control \
  --build-type ament_python \
  --dependencies rclpy std_msgs sensor_msgs

cd humanoid_control
```

### Package Structure
```
humanoid_control/
├── package.xml           # Package metadata
├── setup.py              # Python setup script
├── setup.cfg             # Setup configuration
├── humanoid_control/     # Python module
│   └── __init__.py
├── resource/
│   └── humanoid_control
└── test/                 # Unit tests
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py
```

### Key Files Explained

#### `package.xml`
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>humanoid_control</name>
  <version>0.0.1</version>
  <description>Humanoid robot control package</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

#### `setup.py`
```python
from setuptools import setup

package_name = 'humanoid_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Humanoid robot control package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'head_controller = humanoid_control.head_controller:main',
        ],
    },
)
```

## 🔄 Build System: Colcon

**Colcon** (collective construction) is the build tool for ROS 2.

### Basic Build Commands
```bash
# Build all packages
colcon build

# Build specific package
colcon build --packages-select humanoid_control

# Build with symlink (Python development)
colcon build --symlink-install

# Parallel build (faster)
colcon build --parallel-workers 8
```

### After Building
Always source your workspace:
```bash
source install/setup.bash
```

Or add to `~/.bashrc`:
```bash
source ~/humanoid_ws/install/setup.bash
```

## 🎯 ROS 2 CLI Tools

### Essential Commands

#### Node Management
```bash
# List running nodes
ros2 node list

# Get node info
ros2 node info /node_name
```

#### Topic Operations
```bash
# List active topics
ros2 topic list

# Show topic details
ros2 topic info /topic_name

# Monitor topic messages
ros2 topic echo /topic_name

# Publish to topic
ros2 topic pub /topic_name std_msgs/msg/String "data: 'Hello'"

# Check publishing frequency
ros2 topic hz /topic_name
```

#### Service Operations
```bash
# List services
ros2 service list

# Call a service
ros2 service call /service_name std_srvs/srv/Trigger
```

#### Parameter Management
```bash
# List parameters
ros2 param list

# Get parameter value
ros2 param get /node_name parameter_name

# Set parameter value
ros2 param set /node_name parameter_name value
```

## 🎬 Running ROS 2 Nodes

### Method 1: Direct Execution
```bash
ros2 run package_name executable_name
```

Example:
```bash
ros2 run humanoid_control head_controller
```

### Method 2: Launch Files
Launch files start multiple nodes with configuration.

**launch/robot_launch.py**:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='humanoid_control',
            executable='head_controller',
            name='head_controller',
            parameters=[{'update_rate': 30.0}],
            output='screen'
        ),
        Node(
            package='humanoid_control',
            executable='face_detector',
            name='face_detector',
            output='screen'
        ),
    ])
```

**Run launch file**:
```bash
ros2 launch humanoid_control robot_launch.py
```

## 📊 Visualization Tools

### RViz2
3D visualization tool for robot state, sensors, and planning.

```bash
# Launch RViz2
rviz2

# With config file
rviz2 -d config.rviz
```

### rqt
Qt-based GUI framework for ROS 2.

```bash
# Launch rqt
rqt

# Specific plugins
rqt_graph        # Node graph
rqt_plot         # Data plotting
rqt_console      # Log messages
```

## 🔍 Debugging Tools

### Command-Line Inspection
```bash
# Monitor all topics
ros2 topic list -v

# Check node connectivity
ros2 node list
ros2 topic info /topic_name

# Live data monitoring
ros2 topic echo /camera/image_raw
```

### Logging
ROS 2 uses structured logging:

```python
self.get_logger().debug('Debug message')
self.get_logger().info('Info message')
self.get_logger().warn('Warning message')
self.get_logger().error('Error message')
self.get_logger().fatal('Fatal message')
```

View logs:
```bash
ros2 run rqt_console rqt_console
```

## 🎓 Hands-On Exercise

### Exercise 1: Hello World Node

Create `humanoid_control/hello_humanoid.py`:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HelloHumanoid(Node):
    def __init__(self):
        super().__init__('hello_humanoid')
        self.publisher_ = self.create_publisher(String, 'greeting', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello from humanoid robot! Count: {self.count}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = HelloHumanoid()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Update `setup.py`:
```python
entry_points={
    'console_scripts': [
        'hello_humanoid = humanoid_control.hello_humanoid:main',
    ],
},
```

Build and run:
```bash
cd ~/humanoid_ws
colcon build --packages-select humanoid_control --symlink-install
source install/setup.bash
ros2 run humanoid_control hello_humanoid
```

Monitor in another terminal:
```bash
ros2 topic echo /greeting
```

## 📚 Summary

You've learned:
- ✅ How to install ROS 2 Humble on Ubuntu 22.04
- ✅ Creating and managing ROS 2 workspaces
- ✅ Creating your first ROS 2 package
- ✅ Essential CLI tools for development
- ✅ Building and running nodes
- ✅ Using visualization and debugging tools

## 🚀 Next Chapter

In the next chapter, we'll dive into **Nodes, Topics, and Services**—the core communication patterns that make ROS 2 powerful for humanoid robotics.

---

**Need help?** Ask our AI chatbot about any ROS 2 concept! Just select text and click the chat button.

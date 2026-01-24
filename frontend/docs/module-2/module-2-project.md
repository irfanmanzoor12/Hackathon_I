---
sidebar_position: 6
title: "Module 2 Hands-On Project: Digital Twin Development"
---

# Module 2 Project: Building a Complete Digital Twin System

## Project Overview

In this project, you'll create a complete digital twin system combining Gazebo physics simulation, sensor simulation, and ROS 2 integration. You'll develop a simulated mobile robot with realistic sensors and controllers that can be validated against a physical system.

## Learning Objectives

- Set up Gazebo simulation environment
- Create realistic sensor simulators (LiDAR, camera, IMU)
- Implement domain randomization for robust training
- Validate simulation against real data
- Deploy simulation to hardware-in-the-loop testing

## Project Architecture

```
┌─────────────────────────────────────────────────────┐
│        Digital Twin System Architecture             │
├─────────────────────────────────────────────────────┤
│                                                     │
│  ┌────────────────────────────────────────────┐    │
│  │  Gazebo Physics Simulator                  │    │
│  │  ├─ Robot Model (URDF)                     │    │
│  │  ├─ World Environment                      │    │
│  │  ├─ Physics Engine (ODE)                   │    │
│  │  └─ Collision Detection                    │    │
│  └────────────────────────────────────────────┘    │
│                    │                                │
│  ┌────────────────▼────────────────────────────┐   │
│  │  Sensor Simulators                         │   │
│  │  ├─ LiDAR (Point Cloud)                    │   │
│  │  ├─ RGB-D Camera                           │   │
│  │  └─ IMU (Accelerometer + Gyroscope)        │   │
│  └────────────────────────────────────────────┘   │
│                    │                                │
│  ┌────────────────▼────────────────────────────┐   │
│  │  ROS 2 Middleware                          │   │
│  │  ├─ Topics (Sensors)                       │   │
│  │  ├─ Services (Commands)                    │   │
│  │  └─ Parameters (Configuration)             │   │
│  └────────────────────────────────────────────┘   │
│                    │                                │
│  ┌────────────────▼────────────────────────────┐   │
│  │  Control Layer                             │   │
│  │  ├─ Motion Controller                      │   │
│  │  ├─ Path Planner                           │   │
│  │  └─ Behavior Manager                       │   │
│  └────────────────────────────────────────────┘   │
│                                                     │
└─────────────────────────────────────────────────────┘
```

## Part 1: Gazebo World and Model Setup

### Create World File

Create `gazebo_world/worlds/robot_world.sdf`:

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <world name="robot_world">
    <!-- Physics configuration -->
    <physics name="default_ode" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <ode>
        <solver>
          <type>quick</type>
          <iters>50</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <!-- Lighting -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>5 5 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Walls/Obstacles -->
    <model name="wall_1">
      <static>true</static>
      <pose>5 0 0 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>

    <!-- Include robot model -->
    <include>
      <uri>model://digital_twin_robot</uri>
      <pose>0 0 0 0 0 0</pose>
    </include>
  </world>
</sdf>
```

### Create Enhanced Robot Model

Create `gazebo_models/digital_twin_robot/model.sdf`:

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <model name="digital_twin_robot">
    <static>false</static>
    <self_collide>false</self_collide>

    <!-- Base Link -->
    <link name="base_link">
      <pose>0 0 0.1 0 0 0</pose>
      <inertial>
        <mass>10.0</mass>
        <inertia>
          <ixx>0.1</ixx>
          <iyy>0.1</iyy>
          <izz>0.2</izz>
        </inertia>
      </inertial>

      <visual name="base_visual">
        <geometry>
          <box>
            <size>0.5 0.4 0.2</size>
          </box>
        </geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
        </material>
      </visual>

      <collision name="base_collision">
        <geometry>
          <box>
            <size>0.5 0.4 0.2</size>
          </box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>0.7</mu>
              <mu2>0.7</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
    </link>

    <!-- LiDAR Link -->
    <link name="lidar_link">
      <pose>0 0 0.2 0 0 0</pose>
      <inertial>
        <mass>0.2</mass>
      </inertial>

      <visual name="lidar_visual">
        <geometry>
          <cylinder>
            <radius>0.04</radius>
            <length>0.07</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0 0 0 1</ambient>
        </material>
      </visual>

      <sensor name="lidar" type="lidar">
        <lidar>
          <scan>
            <horizontal>
              <samples>360</samples>
              <resolution>1</resolution>
              <min_angle>0</min_angle>
              <max_angle>6.283</max_angle>
            </horizontal>
            <vertical>
              <samples>16</samples>
              <min_angle>-0.262</min_angle>
              <max_angle>0.262</max_angle>
            </vertical>
          </scan>
          <range>
            <min>0.1</min>
            <max>100</max>
            <resolution>0.01</resolution>
          </range>
          <noise>
            <type>gaussian</type>
            <mean>0</mean>
            <stddev>0.01</stddev>
          </noise>
        </lidar>
        <always_on>true</always_on>
        <update_rate>10</update_rate>
        <topic>lidar</topic>
      </sensor>
    </link>

    <!-- Camera Link -->
    <link name="camera_link">
      <pose>0.2 0 0.15 0 0 0</pose>
      <inertial>
        <mass>0.1</mass>
      </inertial>

      <sensor name="camera" type="camera">
        <camera>
          <horizontal_fov>1.047</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
          </image>
          <clip>
            <near>0.1</near>
            <far>100</far>
          </clip>
        </camera>
        <always_on>true</always_on>
        <update_rate>30</update_rate>
        <topic>camera</topic>
      </sensor>
    </link>

    <!-- IMU Link -->
    <link name="imu_link">
      <inertial>
        <mass>0.01</mass>
      </inertial>

      <sensor name="imu" type="imu">
        <imu>
          <angular_velocity>
            <x>
              <noise type="gaussian">
                <mean>0</mean>
                <stddev>0.009</stddev>
              </noise>
            </x>
            <y>
              <noise type="gaussian">
                <stddev>0.009</stddev>
              </noise>
            </y>
            <z>
              <noise type="gaussian">
                <stddev>0.009</stddev>
              </noise>
            </z>
          </angular_velocity>
          <linear_acceleration>
            <x>
              <noise type="gaussian">
                <mean>0</mean>
                <stddev>0.021</stddev>
              </noise>
            </x>
            <y>
              <noise type="gaussian">
                <stddev>0.021</stddev>
              </noise>
            </y>
            <z>
              <noise type="gaussian">
                <stddev>0.021</stddev>
              </noise>
            </z>
          </linear_acceleration>
        </imu>
        <always_on>true</always_on>
        <update_rate>250</update_rate>
        <topic>imu</topic>
      </sensor>
    </link>

    <!-- Wheels and Joints -->
    <link name="left_wheel">
      <pose>0 0.15 0.1 1.5708 0 0</pose>
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.01</ixx>
          <iyy>0.01</iyy>
          <izz>0.02</izz>
        </inertia>
      </inertial>

      <visual name="left_wheel_visual">
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>0.08</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0 0 0 1</ambient>
        </material>
      </visual>

      <collision name="left_wheel_collision">
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>0.08</length>
          </cylinder>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>1.0</mu>
              <mu2>1.0</mu2>
              <slip1>0.0</slip1>
              <slip2>0.0</slip2>
            </ode>
          </friction>
        </surface>
      </collision>
    </link>

    <link name="right_wheel">
      <pose>0 -0.15 0.1 1.5708 0 0</pose>
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.01</ixx>
          <iyy>0.01</iyy>
          <izz>0.02</izz>
        </inertia>
      </inertial>

      <visual name="right_wheel_visual">
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>0.08</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0 0 0 1</ambient>
        </material>
      </visual>

      <collision name="right_wheel_collision">
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>0.08</length>
          </cylinder>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>1.0</mu>
              <mu2>1.0</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
    </link>

    <!-- Joints -->
    <joint name="left_wheel_joint" type="revolute">
      <parent>base_link</parent>
      <child>left_wheel</child>
      <axis>
        <xyz>0 1 0</xyz>
      </axis>
    </joint>

    <joint name="right_wheel_joint" type="revolute">
      <parent>base_link</parent>
      <child>right_wheel</child>
      <axis>
        <xyz>0 1 0</xyz>
      </axis>
    </joint>

    <joint name="lidar_joint" type="fixed">
      <parent>base_link</parent>
      <child>lidar_link</child>
    </joint>

    <joint name="camera_joint" type="fixed">
      <parent>base_link</parent>
      <child>camera_link</child>
    </joint>

    <joint name="imu_joint" type="fixed">
      <parent>base_link</parent>
      <child>imu_link</child>
    </joint>

    <!-- Gazebo plugins -->
    <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.3</wheel_separation>
      <wheel_radius>0.1</wheel_radius>
      <odom_publish_frequency>50</odom_publish_frequency>
    </plugin>
  </model>
</sdf>
```

## Part 2: Sensor Data Integration

### Create Sensor Publisher Node

Create `src/sensor_publisher.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu, PointCloud2
from std_msgs.msg import Header
import numpy as np

class SensorPublisherNode(Node):
    def __init__(self):
        super().__init__('sensor_publisher')

        # Create publishers
        self.lidar_pub = self.create_publisher(LaserScan, '/lidar/scan', 10)
        self.camera_pub = self.create_publisher(Image, '/camera/image', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)

        # Create timer
        self.timer = self.create_timer(0.1, self.publish_sensor_data)

        self.get_logger().info('Sensor Publisher initialized')

    def publish_sensor_data(self):
        """Publish simulated sensor data"""

        # Publish LiDAR scan
        scan_msg = LaserScan()
        scan_msg.header.frame_id = 'lidar_link'
        scan_msg.header.stamp = self.get_clock().now().to_msg()

        # Create sample scan data
        scan_msg.angle_min = 0.0
        scan_msg.angle_max = 2.0 * np.pi
        scan_msg.angle_increment = (2.0 * np.pi) / 360
        scan_msg.range_min = 0.1
        scan_msg.range_max = 100.0
        scan_msg.ranges = [1.0 + 0.1 * np.sin(i * 0.01) for i in range(360)]

        self.lidar_pub.publish(scan_msg)

        # Publish IMU data
        imu_msg = Imu()
        imu_msg.header.frame_id = 'imu_link'
        imu_msg.header.stamp = self.get_clock().now().to_msg()

        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 9.81

        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = 0.0

        imu_msg.orientation.w = 1.0

        imu_msg.linear_acceleration_covariance = [0.01] * 9
        imu_msg.angular_velocity_covariance = [0.001] * 9

        self.imu_pub.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SensorPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Part 3: Domain Randomization

Create `src/domain_randomizer.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import random
import subprocess

class DomainRandomizer(Node):
    def __init__(self):
        super().__init__('domain_randomizer')

        self.declare_parameter('friction_range', [0.3, 0.8])
        self.declare_parameter('mass_variation', [0.8, 1.2])
        self.declare_parameter('sensor_noise_range', [0.0, 0.1])

        self.timer = self.create_timer(1.0, self.randomize_simulation)

        self.get_logger().info('Domain Randomizer started')

    def randomize_simulation(self):
        """Apply random parameters to simulation"""

        friction = random.uniform(0.3, 0.8)
        mass_variation = random.uniform(0.8, 1.2)
        sensor_noise = random.uniform(0.0, 0.1)

        self.get_logger().info(
            f'Randomized - Friction: {friction:.2f}, '
            f'Mass: {mass_variation:.2f}, '
            f'Noise: {sensor_noise:.3f}'
        )

        # Apply to simulation parameters
        self.apply_randomization(friction, mass_variation, sensor_noise)

    def apply_randomization(self, friction, mass_variation, sensor_noise):
        """Apply randomization to Gazebo simulation"""
        # Use Gazebo service calls to update parameters
        pass

def main(args=None):
    rclpy.init(args=args)
    node = DomainRandomizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Part 4: Validation Framework

Create `src/sim_validator.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from collections import deque

class SimulationValidator(Node):
    def __init__(self):
        super().__init__('sim_validator')

        self.lidar_data = deque(maxlen=100)
        self.imu_data = deque(maxlen=100)
        self.real_lidar_data = deque(maxlen=100)

        # Create subscribers
        self.lidar_sub = self.create_subscription(
            LaserScan, '/lidar/scan', self.lidar_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10
        )

        # Validation timer
        self.timer = self.create_timer(1.0, self.validate)

        self.get_logger().info('Simulation Validator started')

    def lidar_callback(self, msg):
        self.lidar_data.append(np.array(msg.ranges))

    def imu_callback(self, msg):
        self.imu_data.append({
            'ax': msg.linear_acceleration.x,
            'ay': msg.linear_acceleration.y,
            'az': msg.linear_acceleration.z,
        })

    def validate(self):
        """Validate simulation against real data"""

        if len(self.lidar_data) < 10:
            return

        # Compute statistics
        lidar_array = np.array(list(self.lidar_data))
        lidar_mean = np.mean(lidar_array)
        lidar_std = np.std(lidar_array)

        self.get_logger().info(
            f'LiDAR Stats - Mean: {lidar_mean:.2f}m, Std: {lidar_std:.2f}m'
        )

        # Compare with real data if available
        # rmse = self.compute_rmse(sim_data, real_data)
        # self.get_logger().info(f'RMSE vs Real: {rmse:.3f}')

    def compute_rmse(self, sim_data, real_data):
        """Compute RMSE between simulated and real data"""
        return np.sqrt(np.mean((sim_data - real_data) ** 2))

def main(args=None):
    rclpy.init(args=args)
    node = SimulationValidator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Part 5: Launch Configuration

Create `launch/digital_twin.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    return LaunchDescription([
        # Start Gazebo with world file
        ExecuteProcess(
            cmd=['gazebo', '--verbose',
                 'path/to/robot_world.sdf'],
            output='screen'
        ),

        # Sensor publisher node
        Node(
            package='digital_twin_pkg',
            executable='sensor_publisher',
            name='sensor_publisher',
            output='screen'
        ),

        # Domain randomizer
        Node(
            package='digital_twin_pkg',
            executable='domain_randomizer',
            name='domain_randomizer',
            output='screen',
            parameters=[
                {'randomization_enabled': True}
            ]
        ),

        # Validation node
        Node(
            package='digital_twin_pkg',
            executable='sim_validator',
            name='sim_validator',
            output='screen'
        ),
    ])
```

## Testing and Validation

### Run the System

```bash
# Build package
cd ~/ros2_ws
colcon build --packages-select digital_twin_pkg

# Source
source install/setup.bash

# Launch
ros2 launch digital_twin_pkg digital_twin.launch.py
```

### Monitor Simulation

```bash
# In separate terminals:

# View LiDAR data
ros2 topic echo /lidar/scan | head -20

# View IMU data
ros2 topic echo /imu/data

# Record bag file for analysis
ros2 bag record -a -o sim_recording

# Visualize in RViz
ros2 run rviz2 rviz2
```

## Deliverables

1. **Gazebo World and Models**: Complete SDF files
2. **Sensor Integration**: Publishers for all sensors
3. **Domain Randomization**: Parameter randomization system
4. **Validation Framework**: Comparison with real data
5. **Documentation**: System overview and testing guide

## Evaluation Criteria

- **Simulation Fidelity**: Realistic physics and sensor behavior (30%)
- **Sensor Integration**: Accurate sensor publishing (25%)
- **Validation**: Comparison metrics and analysis (25%)
- **Documentation**: Clear explanations and guides (20%)

## Extension Challenges

1. **Add Multiple Robots**: Simulate multi-robot scenarios
2. **Custom Gazebo Plugins**: Develop specialized controllers
3. **Real-time Performance**: Optimize simulation speed
4. **Synthetic Data Generation**: Create training datasets
5. **Hardware Validation**: Compare with real robot

## Summary

This project integrates physics simulation, sensor modeling, and ROS 2 to create a complete digital twin. The system provides a foundation for testing, validation, and training before hardware deployment.

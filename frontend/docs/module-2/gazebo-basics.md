---
sidebar_position: 3
title: "Gazebo Simulator Tutorial"
---

<ChapterActions />

# Gazebo Simulator Tutorial

## Introduction to Gazebo

**Gazebo** is a free, open-source 3D robot simulator designed for developing and testing robotic systems. It provides:
- Accurate physics simulation
- Support for multiple physics engines (ODE, Bullet, DART)
- Extensive sensor library
- ROS 2 integration
- Plugin system for extensibility

### Installation

```bash
# For ROS 2 Humble on Ubuntu 22.04
sudo apt-get update
sudo apt-get install -y ros-humble-gazebo-ros-pkgs

# Verify installation
gazebo --version
```

## Gazebo Architecture

```
┌─────────────────────────────────────┐
│      Gazebo Simulator               │
├─────────────────────────────────────┤
│                                     │
│ ┌─────────────────────────────────┐ │
│ │  Rendering Engine               │ │
│ │  (OpenGL, visualization)        │ │
│ └─────────────────────────────────┘ │
│                                     │
│ ┌─────────────────────────────────┐ │
│ │  Physics Engine                 │ │
│ │  (ODE/Bullet/DART)              │ │
│ └─────────────────────────────────┘ │
│                                     │
│ ┌─────────────────────────────────┐ │
│ │  Sensor Simulators              │ │
│ │  (Camera, LiDAR, IMU, etc)      │ │
│ └─────────────────────────────────┘ │
│                                     │
│ ┌─────────────────────────────────┐ │
│ │  Plugin System                  │ │
│ │  (Custom components)            │ │
│ └─────────────────────────────────┘ │
│                                     │
│ ┌─────────────────────────────────┐ │
│ │  ROS 2 Middleware               │ │
│ │  (Topics, Services)             │ │
│ └─────────────────────────────────┘ │
└─────────────────────────────────────┘
```

## World Files (SDF Format)

Gazebo uses the **Simulation Description Format (SDF)** to define worlds, models, and physics parameters.

### Basic World Structure

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <world name="default">
    <!-- Physics properties -->
    <physics type="ode">
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
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
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
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Include a robot model -->
    <include>
      <uri>model://robot_model</uri>
      <pose>0 0 0 0 0 0</pose>
    </include>
  </world>
</sdf>
```

## Creating a Gazebo Model

### Model Directory Structure

```
robot_model/
├── model.sdf          # Model definition
├── model.config       # Model metadata
└── meshes/
    ├── link1.stl
    ├── link2.stl
    └── link3.stl
```

### Model Configuration File

Create `model.config`:

```xml
<?xml version="1.0"?>
<model>
  <name>mobile_robot</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author>
    <name>Student</name>
    <email>student@example.com</email>
  </author>
  <description>Mobile robot for simulation</description>
</model>
```

### Model SDF File

Create `model.sdf`:

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <model name="mobile_robot">
    <!-- Base Link -->
    <link name="base_link">
      <pose>0 0 0.1 0 0 0</pose>
      <inertial>
        <mass>10.0</mass>
        <inertia>
          <ixx>0.1</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.1</iyy>
          <iyz>0</iyz>
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
          <bounce>
            <restitution_coefficient>0.1</restitution_coefficient>
          </bounce>
        </surface>
      </collision>
    </link>

    <!-- Left Wheel -->
    <link name="left_wheel">
      <pose>0 0.15 0.1 1.5708 0 0</pose>
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.01</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.01</iyy>
          <iyz>0</iyz>
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
          <diffuse>0 0 0 1</diffuse>
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

    <!-- Right Wheel (similar to left wheel) -->
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
          <diffuse>0 0 0 1</diffuse>
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

    <!-- Gazebo plugins for ROS 2 integration -->
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

## Gazebo Plugins

Plugins extend Gazebo functionality with custom behavior.

### System Plugin for Motor Control

Create `src/motor_control_plugin.cpp`:

```cpp
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <geometry_msgs/msg/twist.hpp>

using namespace gz;
using namespace sim;

class MotorControlPlugin: public System,
    public ISystemPreUpdate,
    public ISystemPostUpdate
{
public:
    MotorControlPlugin() = default;

    void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) override
    {
        // Called before physics update
    }

    void PostUpdate(const UpdateInfo &_info, const EntityComponentManager &_ecm) override
    {
        // Called after physics update
    }

private:
    gz::transport::Node node;
};

GZ_ADD_PLUGIN(MotorControlPlugin, gz::sim::System,
    MotorControlPlugin::ISystemPreUpdate,
    MotorControlPlugin::ISystemPostUpdate)
```

## Sensor Simulation in Gazebo

### Camera Sensor

```xml
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
      <lens>
        <type>standard</type>
      </lens>
    </camera>
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <visualize>true</visualize>
    <topic>camera/image</topic>
  </sensor>
</link>
```

### LiDAR Sensor

```xml
<link name="lidar_link">
  <pose>0 0 0.2 0 0 0</pose>
  <inertial>
    <mass>0.2</mass>
  </inertial>

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
          <resolution>1</resolution>
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
    <visualize>false</visualize>
    <topic>lidar/scan</topic>
  </sensor>
</link>
```

### IMU Sensor

```xml
<link name="imu_link">
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
            <mean>0</mean>
            <stddev>0.009</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0</mean>
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
            <mean>0</mean>
            <stddev>0.021</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0</mean>
            <stddev>0.021</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
    <always_on>true</always_on>
    <update_rate>250</update_rate>
    <topic>imu/data</topic>
  </sensor>
</link>
```

## Running Gazebo Simulations

### Launch Gazebo with a World

```bash
# Simple launch
gazebo /path/to/world.sdf

# Without GUI (headless)
gazebo -s libgazebo_ros_init.so -s libgazebo_ros_factory.so /path/to/world.sdf

# With specific physics engine
gazebo --physics bullet /path/to/world.sdf
```

### Programmatic Launch (Python)

```python
import subprocess
import os

def launch_gazebo(world_file, headless=False):
    """Launch Gazebo simulation"""

    cmd = ['gazebo']

    if headless:
        cmd.extend(['-s', 'libgazebo_ros_init.so',
                   '-s', 'libgazebo_ros_factory.so'])

    cmd.append(world_file)

    process = subprocess.Popen(cmd)
    return process
```

### ROS 2 Launch File Integration

Create `launch/gazebo.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='gazebo',
            output='screen',
            arguments=['path/to/world.sdf']
        ),
    ])
```

## Debugging and Visualization

### Gazebo GUI Tools

- **Scene Graph**: View model hierarchy
- **Rendering Sensors**: Visualize camera and lidar data
- **Physics Inspector**: Monitor forces and velocities
- **Entity Inspector**: Edit properties in real-time

### Monitoring Simulation Data

```bash
# List available topics
gazebo topic list

# Echo sensor data
gazebo topic echo /sensor/camera/image

# Publish commands
gazebo topic pub -1 /cmd_vel geometry_msgs.Twist "{linear: {x: 0.5}}"
```

## Performance Optimization

### Tips for Faster Simulation

1. **Reduce visual complexity**: Remove unnecessary meshes
2. **Use simpler collision shapes**: Primitives faster than meshes
3. **Adjust physics parameters**:
   - Increase max_step_size (less accurate but faster)
   - Reduce solver iterations (fewer constraints)
4. **Disable visualization**: Use headless mode
5. **Adjust sensor update rates**: Don't publish faster than needed

### Real-Time Factor

```
Real-Time Factor = Simulation Speed / Real-World Speed

Example:
If sim runs 10x faster than real-time, factor = 10
If sim runs 2x slower, factor = 0.5
```

## Common Issues and Solutions

| Issue | Cause | Solution |
|-------|-------|----------|
| Model falls through ground | Physics error | Increase solver iterations, reduce time step |
| Unstable simulation | Large time step | Reduce max_step_size |
| Slow simulation | High complexity | Simplify models, reduce update rates |
| Sensor data not published | ROS integration | Verify plugin configuration |
| Jerky motion | Time discretization | Use smaller time step |

## Best Practices

1. **Start Simple**: Begin with basic shapes before complex meshes
2. **Validate Physics**: Compare simulation with real system
3. **Document Models**: Comment all custom components
4. **Version Control**: Track all world and model files
5. **Modular Design**: Separate world, models, and controllers
6. **Test Thoroughly**: Verify all sensors and actuators

## Practical Exercise

Create a complete Gazebo simulation:
1. Design a 2-link robotic arm
2. Add camera and LiDAR sensors
3. Create a world with obstacles
4. Develop ROS 2 controller nodes
5. Verify sensor data publication

## Summary

Gazebo provides a powerful, extensible platform for robot simulation. By mastering SDF files, plugins, and sensor configuration, you can create realistic digital twins that accelerate robot development and testing.

## References

- Gazebo Documentation: https://gazebosim.org/docs
- SDF Format Specification: https://sdformat.org/
- Gazebo ROS Integration: https://github.com/gazebosim/ros_gz

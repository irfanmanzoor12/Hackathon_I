---
sidebar_position: 5
title: "Simulating Sensors: LiDAR, Cameras, and IMUs"
---

# Simulating Sensors: LiDAR, Cameras, and IMUs

## Introduction to Sensor Simulation

Accurate sensor simulation is critical for realistic robot behavior. This chapter covers simulating three fundamental sensors: LiDAR (light detection and ranging), RGB-D cameras, and Inertial Measurement Units (IMUs).

## LiDAR Simulation

### LiDAR Principles

LiDAR measures distance by emitting laser pulses and measuring reflection time:

```
Distance = (speed_of_light × time_to_return) / 2

Typical Specifications:
- Range: 5-100+ meters
- Angular Resolution: 0.1-0.3 degrees
- Scan Frequency: 5-20 Hz
- Points per Scan: 1000s to 100,000s
```

### Ray-Based LiDAR Simulation

```python
import numpy as np
from scipy.spatial.transform import Rotation

class LiDARSimulator:
    def __init__(self, num_rays=360, num_rows=16, max_range=100.0):
        self.num_rays = num_rays
        self.num_rows = num_rows
        self.max_range = max_range

        # Pre-compute ray directions
        self.ray_directions = self._compute_ray_directions()

    def _compute_ray_directions(self):
        """Compute unit direction vectors for all rays"""
        directions = []

        # Horizontal angles (0 to 2π)
        horizontal_angles = np.linspace(0, 2 * np.pi, self.num_rays)

        # Vertical angles (-30° to +30°)
        vertical_angles = np.linspace(-np.pi/6, np.pi/6, self.num_rows)

        for v_angle in vertical_angles:
            for h_angle in horizontal_angles:
                # Compute direction vector
                x = np.cos(v_angle) * np.sin(h_angle)
                y = np.sin(v_angle)
                z = np.cos(v_angle) * np.cos(h_angle)

                directions.append([x, y, z])

        return np.array(directions)

    def scan(self, world, sensor_pose):
        """Generate LiDAR scan from current sensor pose"""
        points = []
        ranges = []

        for direction in self.ray_directions:
            # Transform ray to world coordinates
            ray_origin = sensor_pose[:3]
            rotation = Rotation.from_quat(sensor_pose[3:])
            ray_direction = rotation.apply(direction)

            # Ray-cast against world geometry
            distance = self._raycast(world, ray_origin, ray_direction)

            ranges.append(distance)

            if distance < self.max_range:
                point = ray_origin + ray_direction * distance
                points.append(point)

        return np.array(points), np.array(ranges)

    def _raycast(self, world, origin, direction):
        """Find closest intersection of ray with world objects"""
        min_distance = self.max_range

        for obj in world.objects:
            distance = obj.intersect_ray(origin, direction)
            if distance and distance < min_distance:
                min_distance = distance

        return min_distance

    def add_noise(self, ranges, noise_std=0.01):
        """Add Gaussian noise to range measurements"""
        noise = np.random.normal(0, noise_std, len(ranges))
        noisy_ranges = ranges + noise
        return np.clip(noisy_ranges, 0, self.max_range)

    def to_ros_message(self, ranges, frame_id='lidar_link'):
        """Convert to ROS LaserScan message"""
        from sensor_msgs.msg import LaserScan

        msg = LaserScan()
        msg.header.frame_id = frame_id
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * np.pi
        msg.angle_increment = (2.0 * np.pi) / self.num_rays
        msg.time_increment = 0.0
        msg.scan_time = 0.1  # 10 Hz scan rate
        msg.range_min = 0.1
        msg.range_max = self.max_range
        msg.ranges = ranges.tolist()

        return msg
```

### Point Cloud Simulation

```python
class RGBDLiDARSimulator(LiDARSimulator):
    """LiDAR with RGB and intensity data"""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.intensity_sim = IntensitySimulator()

    def scan_with_intensity(self, world, sensor_pose, lighting):
        """Generate scan with intensity based on surface properties"""
        points, ranges = self.scan(world, sensor_pose)

        intensities = []
        for i, (point, distance) in enumerate(zip(points, ranges)):
            if distance < self.max_range:
                # Compute intensity based on:
                # 1. Surface reflectivity
                # 2. Incidence angle
                # 3. Distance falloff

                obj, surface_point = world.get_intersection(point)
                normal = obj.get_surface_normal(surface_point)

                intensity = self.intensity_sim.compute(
                    distance=distance,
                    normal=normal,
                    reflectivity=obj.reflectivity,
                    lighting=lighting
                )
                intensities.append(intensity)
            else:
                intensities.append(0)

        return points, ranges, np.array(intensities)

    def to_point_cloud_message(self, points, intensities, frame_id='lidar_link'):
        """Convert to ROS PointCloud2 message"""
        from sensor_msgs.msg import PointCloud2, PointField
        import struct

        msg = PointCloud2()
        msg.header.frame_id = frame_id
        msg.height = 1
        msg.width = len(points)
        msg.is_dense = True
        msg.is_bigendian = False

        # Define fields: x, y, z, intensity
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width

        # Pack point data
        data = b''
        for point, intensity in zip(points, intensities):
            data += struct.pack('ffff', point[0], point[1], point[2], intensity)

        msg.data = list(data)
        return msg
```

## Camera Simulation

### RGB Camera

```python
import cv2

class CameraSimulator:
    def __init__(self, width=640, height=480, focal_length=500.0):
        self.width = width
        self.height = height
        self.focal_length = focal_length

        # Camera intrinsic matrix
        self.K = np.array([
            [focal_length, 0, width/2],
            [0, focal_length, height/2],
            [0, 0, 1]
        ])

    def render_scene(self, world, sensor_pose, lighting):
        """Render 3D scene from camera pose"""
        # Create virtual frame buffer
        image = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        # For each pixel
        for y in range(self.height):
            for x in range(self.width):
                # Compute ray from camera through pixel
                ray = self._pixel_to_ray(x, y)

                # Transform to world coordinates
                rotation = Rotation.from_quat(sensor_pose[3:])
                world_ray = rotation.apply(ray)

                # Ray cast and find closest object
                hit_point, hit_obj = world.raycast(
                    sensor_pose[:3],
                    world_ray
                )

                if hit_obj:
                    # Compute pixel color based on material and lighting
                    normal = hit_obj.get_surface_normal(hit_point)
                    color = self._compute_color(
                        hit_obj.material,
                        normal,
                        lighting,
                        hit_point
                    )
                    image[y, x] = color

        return image

    def _pixel_to_ray(self, x, y):
        """Convert pixel coordinates to ray direction"""
        fx = self.K[0, 0]
        fy = self.K[1, 1]
        cx = self.K[0, 2]
        cy = self.K[1, 2]

        # Normalize pixel coordinates
        nx = (x - cx) / fx
        ny = (y - cy) / fy

        # Ray direction
        ray = np.array([nx, ny, 1.0])
        return ray / np.linalg.norm(ray)

    def _compute_color(self, material, normal, lighting, point):
        """Compute RGB color at point using lighting model"""
        # Simplified Phong illumination model
        ambient = np.array(material.ambient) * 0.2

        # Diffuse
        light_dir = lighting.direction
        diffuse_intensity = max(0, np.dot(normal, light_dir))
        diffuse = np.array(material.diffuse) * diffuse_intensity

        # Specular
        view_dir = -np.array(point)  # Simplified
        half_dir = normalize(light_dir + view_dir)
        spec_intensity = max(0, np.dot(normal, half_dir)) ** material.shininess
        specular = np.array(material.specular) * spec_intensity

        # Combine
        color = (ambient + diffuse + specular) * 255
        return np.clip(color, 0, 255).astype(np.uint8)

    def add_noise(self, image, noise_type='gaussian', std=5):
        """Add noise to simulate real camera noise"""
        if noise_type == 'gaussian':
            noise = np.random.normal(0, std, image.shape)
            noisy = image.astype(float) + noise
            return np.clip(noisy, 0, 255).astype(np.uint8)

        elif noise_type == 'salt_pepper':
            noisy = image.copy().astype(float)
            num_pixels = image.size
            num_salt = int(0.01 * num_pixels)
            num_pepper = int(0.01 * num_pixels)

            salt_coords = np.random.randint(0, image.shape[0], num_salt), \
                         np.random.randint(0, image.shape[1], num_salt)
            pepper_coords = np.random.randint(0, image.shape[0], num_pepper), \
                           np.random.randint(0, image.shape[1], num_pepper)

            noisy[salt_coords] = 255
            noisy[pepper_coords] = 0

            return noisy.astype(np.uint8)

    def distort_image(self, image, k1=0.0, k2=0.0):
        """Apply lens distortion"""
        h, w = image.shape[:2]
        distorted = cv2.undistort(
            image,
            self.K,
            np.array([k1, k2, 0, 0]),
            newCameraMatrix=self.K
        )
        return distorted

    def to_ros_message(self, image, frame_id='camera_link'):
        """Convert to ROS Image message"""
        from sensor_msgs.msg import Image

        msg = Image()
        msg.header.frame_id = frame_id
        msg.height = image.shape[0]
        msg.width = image.shape[1]
        msg.encoding = 'bgr8' if image.ndim == 3 else 'mono8'
        msg.is_bigendian = False
        msg.step = image.strides[0]
        msg.data = image.tobytes()

        return msg
```

### Depth Camera (RGB-D)

```python
class DepthCameraSimulator(CameraSimulator):
    """Simulates RGB-D cameras (Kinect, RealSense, etc)"""

    def render_depth(self, world, sensor_pose, max_range=5.0):
        """Render depth map"""
        depth_map = np.full((self.height, self.width), max_range, dtype=np.float32)

        rotation = Rotation.from_quat(sensor_pose[3:])
        sensor_pos = sensor_pose[:3]

        for y in range(self.height):
            for x in range(self.width):
                ray = self._pixel_to_ray(x, y)
                world_ray = rotation.apply(ray)

                hit_point, hit_dist = world.raycast(sensor_pos, world_ray)

                if hit_dist and hit_dist < max_range:
                    depth_map[y, x] = hit_dist

        return depth_map

    def add_depth_noise(self, depth_map, uncertainty=0.01):
        """Add realistic depth sensor noise"""
        # Depth noise increases with distance
        noise = np.random.normal(0, uncertainty * depth_map, depth_map.shape)
        noisy_depth = depth_map + noise
        return np.clip(noisy_depth, 0, np.max(depth_map))

    def rgbd_to_point_cloud(self, rgb_image, depth_map):
        """Convert RGB-D image to point cloud"""
        h, w = depth_map.shape
        points = []
        colors = []

        for y in range(h):
            for x in range(w):
                depth = depth_map[y, x]
                if depth < np.inf:
                    # Back-project to 3D
                    ray = self._pixel_to_ray(x, y)
                    point_3d = ray * depth
                    points.append(point_3d)
                    colors.append(rgb_image[y, x])

        return np.array(points), np.array(colors)
```

## IMU Simulation

### Accelerometer and Gyroscope

```python
class IMUSimulator:
    def __init__(self):
        self.last_velocity = np.zeros(3)
        self.last_angular_velocity = np.zeros(3)
        self.gravity = np.array([0, 0, 9.81])

    def simulate(self, state, dt=0.01):
        """Simulate IMU readings given robot state"""
        # Current state
        position = state['position']
        velocity = state['velocity']
        rotation = Rotation.from_quat(state['orientation'])
        angular_velocity = state['angular_velocity']

        # Compute acceleration in inertial frame
        linear_accel = (velocity - self.last_velocity) / dt
        self.last_velocity = velocity.copy()

        # Compute angular acceleration
        angular_accel = (angular_velocity - self.last_angular_velocity) / dt
        self.last_angular_velocity = angular_velocity.copy()

        # Transform to body frame
        linear_accel_body = rotation.inv().apply(linear_accel)
        angular_accel_body = rotation.inv().apply(angular_accel)

        # Add gravity (always acts downward in body frame)
        gravity_body = rotation.inv().apply(self.gravity)
        linear_accel_with_gravity = linear_accel_body + gravity_body

        # Transform angular velocity to body frame
        angular_vel_body = rotation.inv().apply(angular_velocity)

        return {
            'linear_acceleration': linear_accel_with_gravity,
            'angular_velocity': angular_vel_body,
            'orientation': state['orientation']
        }

    def add_noise(self, imu_data):
        """Add realistic IMU noise"""
        # Accelerometer noise
        accel_noise = np.random.normal(0, 0.021, 3)  # m/s²
        noisy_accel = imu_data['linear_acceleration'] + accel_noise

        # Gyroscope noise
        gyro_noise = np.random.normal(0, 0.009, 3)  # rad/s
        noisy_gyro = imu_data['angular_velocity'] + gyro_noise

        # Bias drift (slow changes)
        self.accel_bias += np.random.normal(0, 0.0001, 3)
        self.gyro_bias += np.random.normal(0, 0.00001, 3)

        noisy_accel += self.accel_bias
        noisy_gyro += self.gyro_bias

        return {
            'linear_acceleration': noisy_accel,
            'angular_velocity': noisy_gyro,
            'orientation': imu_data['orientation']
        }

    def to_ros_message(self, imu_data, frame_id='imu_link'):
        """Convert to ROS Imu message"""
        from sensor_msgs.msg import Imu

        msg = Imu()
        msg.header.frame_id = frame_id

        msg.linear_acceleration.x = imu_data['linear_acceleration'][0]
        msg.linear_acceleration.y = imu_data['linear_acceleration'][1]
        msg.linear_acceleration.z = imu_data['linear_acceleration'][2]

        msg.angular_velocity.x = imu_data['angular_velocity'][0]
        msg.angular_velocity.y = imu_data['angular_velocity'][1]
        msg.angular_velocity.z = imu_data['angular_velocity'][2]

        msg.orientation.x = imu_data['orientation'][0]
        msg.orientation.y = imu_data['orientation'][1]
        msg.orientation.z = imu_data['orientation'][2]
        msg.orientation.w = imu_data['orientation'][3]

        # Covariance matrices
        msg.linear_acceleration_covariance = [0.021**2] * 9
        msg.angular_velocity_covariance = [0.009**2] * 9
        msg.orientation_covariance = [0.1**2] * 9

        return msg
```

## Integrated Sensor System

```python
class SensorSuite:
    """Manages multiple sensors for complete robot perception"""

    def __init__(self, config):
        self.lidar = LiDARSimulator(**config['lidar'])
        self.camera = CameraSimulator(**config['camera'])
        self.depth_camera = DepthCameraSimulator(**config['depth_camera'])
        self.imu = IMUSimulator()

        self.publishers = {}

    def setup_publishers(self, node):
        """Setup ROS 2 publishers"""
        self.publishers['lidar'] = node.create_publisher(
            LaserScan, '/sensor/lidar', 10
        )
        self.publishers['camera'] = node.create_publisher(
            Image, '/camera/rgb', 10
        )
        self.publishers['depth'] = node.create_publisher(
            Image, '/camera/depth', 10
        )
        self.publishers['imu'] = node.create_publisher(
            Imu, '/sensor/imu', 10
        )

    def update(self, world, robot_state, time_step):
        """Update all sensors and publish data"""

        # LiDAR scan
        lidar_points, lidar_ranges = self.lidar.scan(world, robot_state['pose'])
        lidar_msg = self.lidar.to_ros_message(lidar_ranges)
        self.publishers['lidar'].publish(lidar_msg)

        # RGB image
        rgb_image = self.camera.render_scene(world, robot_state['pose'], world.lighting)
        camera_msg = self.camera.to_ros_message(rgb_image)
        self.publishers['camera'].publish(camera_msg)

        # Depth image
        depth_map = self.depth_camera.render_depth(world, robot_state['pose'])
        depth_map = self.depth_camera.add_depth_noise(depth_map)
        depth_msg = self.depth_camera.to_ros_message(depth_map)
        self.publishers['depth'].publish(depth_msg)

        # IMU data
        imu_data = self.imu.simulate(robot_state, time_step)
        imu_data = self.imu.add_noise(imu_data)
        imu_msg = self.imu.to_ros_message(imu_data)
        self.publishers['imu'].publish(imu_msg)
```

## Validation and Calibration

### Comparing Simulated vs Real Sensors

```python
def validate_sensor_sim(sim_data, real_data, metric='mse'):
    """Compare simulated sensor data with real measurements"""

    error = {
        'mse': np.mean((sim_data - real_data)**2),
        'rmse': np.sqrt(np.mean((sim_data - real_data)**2)),
        'mae': np.mean(np.abs(sim_data - real_data)),
        'correlation': np.corrcoef(sim_data, real_data)[0, 1]
    }

    return error
```

## Summary

Accurate sensor simulation is essential for realistic digital twins. By simulating LiDAR point clouds, camera images, and IMU data with appropriate noise models, you create realistic training environments for robot perception and control algorithms.

## References

- LiDAR Simulation: https://docs.ros.org/en/humble/Tutorials/Ros2Gazebo.html
- Camera Calibration: http://docs.opencv.org/master/d9/d0c/group__calib3d.html
- IMU Noise Modeling: https://arxiv.org/abs/1502.07674

---
title: Simulating the Robot's Senses
---

# Simulating the Robot's Senses in Gazebo

With the physical model of our robot in place, the next step is to give it senses. In this section, we'll learn how to add and configure common robotic sensors in Gazebo. We will focus on three key sensors: a LiDAR, a depth camera, and an Inertial Measurement Unit (IMU).

We will use Gazebo's sensor plugins to simulate these devices and publish their data to ROS 2 topics.

## Adding a LiDAR Sensor

A LiDAR (Light Detection and Ranging) sensor is crucial for mapping and localization. It measures distances by illuminating a target with a laser and analyzing the reflected light.

To add a LiDAR to our robot, we need to add a `<sensor>` tag to our robot's SDF file, within one of the links.

```xml
<!-- Example of a LiDAR sensor in an SDF file -->
<link name='head'>
  ...
  <sensor name='gpu_lidar' type='gpu_lidar'>
    <pose>0 0 0.1 0 0 0</pose>
    <topic>/laser_scan</topic>
    <update_rate>10</update_rate>
    <lidar>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.14</min_angle>
          <max_angle>3.14</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>10.0</max>
        <resolution>0.01</resolution>
      </range>
    </lidar>
    <plugin
      filename="libgazebo_ros_gpu_lidar.so"
      name="gazebo_ros_gpu_lidar">
      <topic_name>/laser_scan</topic_name>
      <frame_name>lidar_link</frame_name>
    </plugin>
  </sensor>
</link>
```

The `libgazebo_ros_gpu_lidar.so` plugin does the work of simulating the laser scan and publishing it to the `/laser_scan` ROS 2 topic as a `sensor_msgs/LaserScan` message.

## Adding a Depth Camera

A depth camera provides a 2D image where each pixel's value represents the distance to the object. This is essential for 3D perception and obstacle avoidance.

```xml
<!-- Example of a Depth Camera sensor in an SDF file -->
<link name='head'>
  ...
  <sensor name='depth_camera' type='depth_camera'>
    <pose>0.1 0 0 0 0 0</pose>
    <topic>/depth_camera</topic>
    <update_rate>30</update_rate>
    <camera>
      <horizontal_fov>1.0</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
      </image>
      <clip>
        <near>0.1</near>
        <far>10</far>
      </clip>
    </camera>
    <plugin
      filename="libgazebo_ros_camera.so"
      name="camera_controller">
      <topic_name>/depth_camera</topic_name>
      <frame_name>camera_link</frame_name>
    </plugin>
  </sensor>
</link>
```
The `libgazebo_ros_camera.so` plugin will publish the depth image data to the `/depth_camera` topic as a `sensor_msgs/Image`.

## Adding an IMU

An Inertial Measurement Unit (IMU) measures and reports a body's specific force, angular rate, and sometimes the orientation of the body. It's fundamental for robot stabilization and orientation tracking.

```xml
<!-- Example of an IMU sensor in an SDF file -->
<link name='torso'>
  ...
  <sensor name='imu_sensor' type='imu'>
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <topic>/imu</topic>
    <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
      <topic_name>/imu</topic_name>
      <frame_name>imu_link</frame_name>
    </plugin>
  </sensor>
</link>
```

The `libgazebo_ros_imu_sensor.so` plugin publishes the orientation, angular velocity, and linear acceleration to the `/imu` topic as a `sensor_msgs/Imu` message.

## Verifying Sensor Output

Once you have added these sensors to your robot's SDF file and launched the simulation, you can verify that the data is being published correctly using the ROS 2 command line:

```bash
# Check the list of active topics
ros2 topic list

# Echo the output of the laser scanner
ros2 topic echo /laser_scan

# Echo the output of the IMU
ros2 topic echo /imu
```

With sensors providing data to our ROS 2 system, our robot can now perceive its world. In the next section, we'll build the visual counterpart to this simulation in Unity.

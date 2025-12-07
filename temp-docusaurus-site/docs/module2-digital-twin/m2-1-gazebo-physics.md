---
title: Building the Physics Core in Gazebo
---

# Building the Physics Core in Gazebo

In this section, we will lay the foundation of our digital twin by creating the physical representation of our robot and its environment in Gazebo. This involves defining the robot's structure and physical properties using the Simulation Description Format (SDF) or Unified Robot Description Format (URDF).

## SDF vs. URDF

-   **URDF (Unified Robot Description Format)** is an XML format used in ROS to describe the kinematics and dynamics of a robot. It's great for single-robot descriptions but lacks tags for describing the world or more complex simulations.
-   **SDF (Simulation Description Format)** is the native format for Gazebo. It's a more comprehensive XML format that can describe everything from robots and sensors to the environment, lighting, and physics.

For this chapter, we will primarily use **SDF**, as it allows us to define the entire simulation in one place.

## Creating the Robot Model

Let's start by defining a simple humanoid robot. We will create an SDF file that describes its links (the rigid parts) and joints (the connections between links).

Here is a snippet of what the SDF file for a single leg might look like.

```xml
<!-- Example SDF for a robot leg -->
<sdf version='1.7'>
  <model name='humanoid_leg'>
    <link name='thigh'>
      <pose>0 0 0.5 0 0 0</pose>
      <inertial>
        <mass>5.0</mass>
        <inertia>
          <ixx>0.1</ixx>
          <iyy>0.1</iyy>
          <izz>0.1</izz>
        </inertia>
      </inertial>
      <collision name='collision'>
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.4</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name='visual'>
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.4</length>
          </cylinder>
        </geometry>
      </visual>
    </link>

    <link name='shin'>
      <pose>0 0 0.2 0 0 0</pose>
      <inertial>
        <mass>3.0</mass>
        <inertia>
          <ixx>0.05</ixx>
          <iyy>0.05</iyy>
          <izz>0.05</izz>
        </inertia>
      </inertial>
      <collision name='collision'>
        <geometry>
          <cylinder>
            <radius>0.04</radius>
            <length>0.3</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name='visual'>
        <geometry>
          <cylinder>
            <radius>0.04</radius>
            <length>0.3</length>
          </cylinder>
        </geometry>
      </visual>
    </link>

    <joint name='knee' type='revolute'>
      <parent>thigh</parent>
      <child>shin</child>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>0</lower>
          <upper>2.5</upper>
        </limit>
      </axis>
    </joint>
  </model>
</sdf>
```

**Key tags to note:**
-   `<link>`: A rigid body in the simulation.
-   `<inertial>`: Defines the mass and moment of inertia.
-   `<collision>`: The geometry used for physics calculations.
-   `<visual>`: The geometry that is rendered in Gazebo's GUI.
-   `<joint>`: Connects two links and defines their motion.

## Setting up the Gazebo World

Next, we need to create a `.world` file for Gazebo. This file defines the environment, including lighting, physics properties, and any static objects.

```xml
<!-- Example world file -->
<sdf version='1.7'>
  <world name='default'>
    <light name='sun' type='directional'>
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
    </light>

    <model name='ground_plane'>
      <static>true</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name='visual'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </visual>
      </link>
    </model>

    <physics type='ode'>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <gravity>0 0 -9.8</gravity>

  </world>
</sdf>
```
In this world file, we've defined a ground plane, a light source, the gravity vector, and the physics engine properties.

By creating these two files, you have a complete physical simulation of your robot in a simple world. In the next section, we'll add sensors to our robot.

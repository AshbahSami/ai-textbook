---
title: Troubleshooting and Common Issues
---

# Troubleshooting and Common Issues

Building a digital twin involves several complex pieces of software working together. It's common to run into issues. This section provides a list of common problems and their solutions.

## Connection Issues with ROS-TCP-Connector

**Problem**: Unity is not connecting to the ROS 2 network. You see "Connection timed out" errors in the Unity console.

**Solutions**:
-   **Check IP Addresses**: Ensure that the `ROS IP Address` in the Unity ROS Settings is correct. If ROS is running on the same machine, use `127.0.0.1`. If it's on a different machine, make sure it's the correct IP and that the machines are on the same network.
-   **Firewall**: A firewall on either the ROS machine or the Unity machine might be blocking the TCP connection. Make sure that the port used by the ROS-TCP-Connector (default is 10000) is open.
-   **Run the ROS side first**: Make sure you have started the `ros_tcp_endpoint` on the ROS 2 side *before* you press the "Play" button in the Unity editor.

## Incorrect Visualization in Unity

**Problem**: The robot model in Unity is not moving, or it's moving incorrectly (e.g., parts are flying off, orientation is wrong).

**Solutions**:
-   **Coordinate System**: Remember that Gazebo and Unity use different coordinate systems. You need to convert positions and rotations when passing them between the two.
    -   **Position**: Swap Y and Z axes.
    -   **Rotation**: Swap Y and Z components, and negate all of X, Y, and Z.
-   **Topic Names**: Double-check that the topic names you are subscribing to in your Unity C# scripts exactly match the topic names being published from ROS 2. Use `ros2 topic list` to be sure.
-   **Link Names**: In the `PoseSubscriber.cs` script, make sure the name of the robot link you are looking for (`humanoid::base_link` in our example) exactly matches the name of the link in your SDF file.

## Gazebo Simulation is Slow or Unstable

**Problem**: The Gazebo simulation runs very slowly or crashes.

**Solutions**:
-   **Physics Properties**: The `max_step_size` in your `.world` file can have a big impact. A smaller step size is more accurate but computationally more expensive. Try adjusting this value.
-   **Collision Geometry**: Complex collision meshes can significantly slow down the physics calculations. Use simplified collision geometries (like spheres, cylinders, and boxes) whenever possible, even if your visual models are complex.
-   **Hardware Acceleration**: Ensure that your system is using hardware-accelerated graphics for Gazebo.

## General Debugging Tips

-   **ROS 2 Tools**: Use `ros2 topic echo`, `ros2 topic list`, `ros2 node info`, and `rqt_graph` to visualize the ROS 2 communication graph and inspect messages.
-   **Unity Console**: Keep the Unity console open to see any error messages from your C# scripts. Use `Debug.Log()` statements to print variable values and trace the execution of your code.
-   **Gazebo GUI**: The Gazebo GUI provides a lot of useful information. You can inspect the properties of models, see collision meshes, and view sensor data visualizations.

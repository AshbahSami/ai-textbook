---
title: Bridging the Physical and Visual Worlds
---

# Bridging the Physical and Visual Worlds with ROS 2

This is the final and most critical step in creating our digital twin: connecting all the components. We need to establish a robust, bi-directional communication bridge so that our ROS 2 control system can talk to both the Gazebo physics simulation and the Unity visualization.

For this chapter, we will use the **ROS-TCP-Connector** package for Unity, which provides a reliable way to send and receive ROS messages over a TCP connection.

## Setting up the ROS-TCP-Connector in Unity

1.  **Install the package**: In your Unity project, open the Package Manager (`Window -> Package Manager`). Click the `+` icon, select `Add package from git URL...`, and enter `https://github.com/Unity-Technologies/ROS-TCP-Connector.git`.
2.  **Configure the ROS Connection**: After the package is installed, a new menu item `Robotics` will appear. Go to `Robotics -> ROS Settings`. Here, you will set the `ROS IP Address` to the IP address of the machine running the ROS 2 master (or `127.0.0.1` if it's the same machine).

## Subscribing to ROS 2 Topics in Unity

Let's create a script that subscribes to the robot's pose from Gazebo and updates the visual model in Unity.

Create a C# script named `PoseSubscriber.cs`.

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Gazebo;

public class PoseSubscriber : MonoBehaviour
{
    public GameObject robot; // Assign your robot prefab in the inspector

    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<LinkStatesMsg>("/gazebo/link_states", UpdatePose);
    }

    void UpdatePose(LinkStatesMsg msg)
    {
        // Find the index of the base link of your robot
        int baseLinkIndex = -1;
        for (int i = 0; i < msg.name.Length; i++)
        {
            if (msg.name[i] == "humanoid::base_link") // Adjust the name to match your robot's base link
            {
                baseLinkIndex = i;
                break;
            }
        }

        if (baseLinkIndex != -1)
        {
            // The position and orientation from Gazebo need to be converted
            // from a left-handed to a right-handed coordinate system.
            Vector3 position = new Vector3(
                (float)msg.pose[baseLinkIndex].position.x,
                (float)msg.pose[baseLinkIndex].position.z, // Swap Y and Z
                (float)msg.pose[baseLinkIndex].position.y
            );
            Quaternion rotation = new Quaternion(
                -(float)msg.pose[baseLinkIndex].orientation.x,
                -(float)msg.pose[baseLinkIndex].orientation.z, // Swap Y and Z and negate
                -(float)msg.pose[baseLinkIndex].orientation.y,
                (float)msg.pose[baseLinkIndex].orientation.w
            );

            robot.transform.position = position;
            robot.transform.rotation = rotation;
        }
    }
}
```

Attach this script to a `GameObject` in your scene and assign your robot's prefab to the `robot` field in the inspector. This script subscribes to the `/gazebo/link_states` topic, which is published by Gazebo and contains the pose of all links in the simulation. It then updates the robot's position and rotation in Unity.

## The ROS 2 Launch File

To tie everything together, we can create a ROS 2 launch file that starts Gazebo, the Unity bridge, and any other necessary nodes. The ROS-TCP-Connector package provides a `ros_service.py` script that acts as the bridge on the ROS 2 side.

Here's an example of a simple launch file:

```python
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', 'worlds/my_world.world'],
            output='screen'
        ),

        # Start the ROS-TCP-Connector bridge
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_tcp_endpoint', 'default_server_endpoint'],
            output='screen'
        ),

        # Your robot's control node
        Node(
            package='my_robot_control',
            executable='control_node',
            name='control_node'
        ),
    ])
```

Now, you can launch the entire digital twin ecosystem with a single command:
`ros2 launch my_robot_bringup digital_twin.launch.py`

With this final piece, your digital twin is complete! You have a physics simulation, a high-fidelity visualization, and a ROS 2 control system all working in harmony.

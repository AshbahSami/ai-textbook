---
title: Creating the Visual Twin in Unity
---

# Creating the Visual Twin in Unity

Now that we have a fully functional physics simulation in Gazebo, it's time to build the other half of our digital twin: the high-fidelity visual representation in Unity. This is where we can leverage Unity's advanced graphics capabilities to create a visually stunning and interactive experience.

## Setting up a New Unity Project

First, you'll need to create a new Unity project.

1.  Open the Unity Hub and create a new project.
2.  Select the **3D (URP)** or **3D (HDRP)** template for high-fidelity rendering. The Universal Render Pipeline (URP) is a good balance of performance and quality, while the High Definition Render Pipeline (HDRP) is for top-tier visuals. We will use URP for this chapter.
3.  Give your project a name and save it.

## Importing Assets

Once your project is created, you need to import the visual models for your robot and the environment. Unlike the SDF files which contained collision geometry, these should be high-polygon models suitable for rendering (e.g., `.fbx` or `.obj` files).

You can create these assets yourself or find them on the Unity Asset Store. For our humanoid robot, you would import the model and create a "prefab" - a reusable asset that bundles the model, materials, and any associated scripts.

## Scripting Human-Robot Interaction (HRI)

Unity's power shines when creating interactive experiences. We can easily create UI elements and script simple HRI scenarios.

As an example, let's create a simple UI button that, when clicked, sends a command to our robot via ROS 2.

First, create a C# script named `HRIController.cs`.

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class HRIController : MonoBehaviour
{
    // ROS Connector
    private ROSConnection ros;

    // ROS Topic
    private readonly string topicName = "/hri_command";

    void Start()
    {
        // Get ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<StringMsg>(topicName);
    }

    public void SendWaveCommand()
    {
        StringMsg msg = new StringMsg("wave");
        ros.Publish(topicName, msg);
        Debug.Log("Sent 'wave' command");
    }
}
```

This script uses the `ROS-TCP-Connector` package (which we will set up in the next section) to publish a simple string message to a ROS 2 topic.

Next, in the Unity editor:
1.  Create a new UI Button (`GameObject -> UI -> Button`).
2.  Create an empty `GameObject` and attach the `HRIController.cs` script to it.
3.  Select the button and in the `On Click()` event handler in the Inspector window, drag the `GameObject` with the `HRIController` script into the object field.
4.  From the function dropdown, select `HRIController -> SendWaveCommand()`.

Now, when you run the simulation in Unity and click the button, it will publish a message to the `/hri_command` topic. A ROS 2 node can then subscribe to this topic to make the robot wave its arm.

This is a simple example, but it demonstrates the power of using Unity for creating rich, interactive control interfaces for your robot. In the next section, we will set up the bridge that makes this communication possible.

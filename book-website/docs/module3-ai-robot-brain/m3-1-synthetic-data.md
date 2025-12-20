---
title: Synthetic Data Generation in Isaac Sim
---

# Synthetic Data Generation in Isaac Sim

In this section, we'll explore how NVIDIA Isaac Sim, built on the Omniverse platform, allows us to generate vast amounts of high-quality synthetic data. This data is crucial for training robust AI vision models, especially when real-world data is scarce, expensive, or difficult to acquire.

## Setting up an Isaac Sim Scene

First, launch Isaac Sim. You can either use a pre-built scene or create a new one. For our humanoid robot, we'll assume a medium-sized indoor environment with dynamic lighting and various obstacles, as per our specifications.

1.  **Import Humanoid Model**: Import your humanoid robot's USD (Universal Scene Description) model into the scene. Isaac Sim works natively with USD, allowing for rich asset descriptions and composition.
2.  **Populate Environment**: Add static and simple dynamic obstacles (e.g., conveyor belts, moving doors) to increase the complexity and realism of the scene. Vary textures and materials to ensure visual diversity.
3.  **Configure Lighting**: Set up dynamic lighting conditions, including different times of day, varying intensities, and artificial lights. This helps generalize vision models to different illumination scenarios.

## Configuring the Realsense Camera Extension

Isaac Sim provides extensions to simulate various sensors. For synthetic data generation, the **Realsense Camera extension** is particularly useful as it can output RGB, depth, and segmentation maps.

1.  **Enable Extension**: Go to `Window -> Extensions` and enable the `omni.isaac.realsense` extension.
2.  **Add Realsense Camera**: Select your robot's head (or a suitable link) and add a `Realsense Camera` component to it. Position and orient the camera as you would a physical sensor.
3.  **Configure Outputs**: In the Realsense Camera properties, you can specify which data streams to generate:
    -   **RGB**: Photorealistic color images.
    -   **Depth**: Per-pixel distance information.
    -   **Instance Segmentation**: Assigns a unique ID to each object instance, crucial for training object detection and segmentation models.
    -   **Semantic Segmentation**: Labels pixels based on object class (e.g., "chair", "table").

## Generating Annotated Synthetic Data

Once the scene and camera are set up, you can generate synthetic data programmatically using Isaac Sim's Python API. This allows for precise control over the data generation process, including varying camera poses, object positions, and environmental conditions.

Here's a simplified Python script snippet demonstrating how to capture data:

```python
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import open_stage_as_stream
from omni.isaac.sensor import _sensor
import omni.timeline

# Assume stage is already open and robot/environment loaded
# Get the Realsense camera sensor
camera_prim = "/World/your_robot/head/realsense_camera" # Adjust path
realsense_sensor = _sensor.acquire_sensor_interface(camera_prim)

# Set the timeline to play
timeline = omni.timeline.get_timeline_interface()
timeline.play()

# Loop to capture frames
for i in range(100): # Capture 100 frames as an example
    # Advance simulation
    omni.usd.UsdContext().get : (
        omni.usd.UsdContext().get_stage_next_time_step()
    )

    # Capture data
    rgb_data = realsense_sensor.get_image_data()
    depth_data = realsense_sensor.get_depth_data()
    segmentation_data = realsense_sensor.get_instance_segmentation_data()

    # Save data (implement your own saving logic)
    # E.g., save as PNG, EXR, or numpy arrays with associated metadata
    print(f"Captured frame {i}: RGB {rgb_data.shape}, Depth {depth_data.shape}")

# Stop timeline
timeline.stop()
```

This script allows you to automate the data collection, varying parameters to create a diverse dataset. The generated data (RGB, depth, segmentation maps) can then be used to train your vision models, which we will deploy using Isaac ROS in the next section.

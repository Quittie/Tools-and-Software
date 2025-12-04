# Tools-and-Software## Stage 2 – Custom ROS2 Camera Node

We created a custom ROS2 node (`camera_node.py`) that:
- subscribes to `/image_raw`,
- converts ROS Image → OpenCV using CvBridge,
- displays live video in an OpenCV window,
- runs in real-time.

Screenshot:
![Stage 2](docs/stage2_camera_node_screenshot.png)

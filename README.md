## Stage 1 — Camera Setup Complete

We connected a USB camera and verified the video stream using ROS2:

- Installed `usb_cam` package
- Installed `rqt_image_view`
- Confirmed the driver publishes `/image_raw`
- Preview visible in `rqt_image_view`

Screenshot:

https://github.com/Quittie/Tools-and-Software/raw/refs/heads/main/ETAP%201%20PHOTO


## Stage 2 – Custom ROS2 Camera Node

We created a custom ROS2 node (`camera_node.py`) that:
- subscribes to `/image_raw`,
- converts ROS Image → OpenCV using CvBridge,
- displays live video in an OpenCV window,
- runs in real-time.

Screenshot:
![Stage 2](docs/stage2_camera_node_screenshot.png)

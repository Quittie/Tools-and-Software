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

## Stage 3 – Mouse-based interaction

We extended `camera_node.py` to:
- handle left mouse button clicks in the camera window,
- compare the click position with the vertical middle of the image,
- log a decision:
  - click above center → FORWARD
  - click below center → BACKWARD

## Stage 4 – Robot control (turtlesim)

- Robot: `turtlesim_node`
- Topic sterujący: `/turtle1/cmd_vel` (`geometry_msgs/Twist`)
- Nasz node `camera_node.py`:
  - subskrybuje `/image_raw`,
  - obsługuje kliknięcia myszy w oknie kamery,
  - decyzja:
    - klik powyżej środka → `FORWARD` → `linear.x = 1.0`
    - klik poniżej środka → `BACKWARD` → `linear.x = -1.0`
  - publikuje komendy Twist na `/turtle1/cmd_vel`, przez co żółw rusza do przodu / cofa się.
- Dowód działania: screeny w `docs/` (kamera + turtlesim + logi node’a).


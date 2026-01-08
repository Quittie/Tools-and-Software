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

## Stage 5 – Automatic launch (+0.5)

In this stage we prepared an automatic startup of the whole system using a ROS 2 launch file and a simple shell script.

The script:
- ustawia poprawnie zmienną `ROS_DOMAIN_ID=0`,
- ładuje środowisko ROS 2 Humble,
- ładuje nasz workspace `camera_project`,
- uruchamia launch file `camera_turtle.launch.py`, który startuje:
  - driver kamery (`usb_cam_node_exe`),
  - nasz node `camera_node`,
  - symulator robota `turtlesim_node`.

Thanks to this, the whole demo can be started with a **single command**:

```bash
./run_camera_project.sh

---------------------------------------------------


Stage 6 – ArUco-based robot control (+0.5)

As an extension of the mouse-based interface, we implemented a vision-based robot control mode using ArUco markers.

Camera input:

image stream provided on /image_raw.

ArUco detection:

implemented using OpenCV ArUco module (DICT_4X4_50),

the center of the detected marker is computed based on its corner coordinates.

Control logic implemented in camera_node.py:

if the center of the ArUco marker is above the vertical center of the image → FORWARD → linear.x = 1.0,

if the center of the ArUco marker is below the vertical center of the image → BACKWARD → linear.x = -1.0,

if no marker is detected → robot stops (linear.x = 0.0).

Velocity commands (geometry_msgs/Twist) are published on /turtle1/cmd_vel, enabling control of the robot in the turtlesim environment.

This stage demonstrates an alternative human–robot interaction method based solely on visual feedback, without the use of mouse input.

Proof of operation: screenshots in docs/ (ArUco marker visible in camera view, robot motion in turtlesim, node logs).

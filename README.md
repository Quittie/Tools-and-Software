## Stage 1 — Camera Setup Complete

We connected a USB camera and verified the ROS 2 video stream.

- Installed `usb_cam`
- Installed `rqt_image_view`
- Verified that the camera driver publishes `/image_raw`
- Confirmed the live preview in `rqt_image_view`

**Screenshot (Stage 1):**  
![Stage 1 — usb_cam + rqt_image_view](stage1_camera_setup.png)



## Stage 2 — Custom ROS 2 Camera Node

We implemented a custom ROS 2 node (`camera_node.py`) that:

- subscribes to `/image_raw`,
- converts `sensor_msgs/Image` to OpenCV format using `CvBridge`,
- displays the live camera stream in an OpenCV window,
- operates in real time.

**Screenshot (Stage 2):**  
![Stage 2 — custom camera node](stage2_camera_node.png)

## Stage 3 — Mouse-based interaction

We extended `camera_node.py` to support mouse-based interaction in the camera window.

Functionality:
- left mouse button clicks are captured in the OpenCV window,
- the click position is compared with the vertical center of the image,
- a control decision is generated:
  - click above the image center → **FORWARD**
  - click below the image center → **BACKWARD**

**Screenshots (Stage 3):**

*Click above image center — FORWARD*  
![Stage 3 — mouse forward](stage3_mouse_forward.png)

*Click below image center — BACKWARD*  
![Stage 3 — mouse backward](stage3_mouse_backward.png)


## Stage 4 — Robot control (turtlesim)

In this stage we connected the camera-based decision logic with robot control in `turtlesim`.

- Robot: `turtlesim_node`
- Control topic: `/turtle1/cmd_vel` (`geometry_msgs/Twist`)
- Our node `camera_node.py`:
  - subscribes to `/image_raw`,
  - processes mouse clicks in the camera window,
  - generates a decision:
    - click above image center → **FORWARD** → `linear.x = 1.0`
    - click below image center → **BACKWARD** → `linear.x = -1.0`
  - publishes `Twist` commands to `/turtle1/cmd_vel`, which makes the turtle move forward / backward.

**Screenshots (Stage 4):**

*FORWARD (linear.x > 0)*  
![Stage 4 — turtlesim forward](stage4_turtlesim_forward.png)

*BACKWARD (linear.x < 0)*  
![Stage 4 — turtlesim backward](stage4_turtlesim_backward.png)

## Stage 5 – Automatic launch (+0.5)

In this stage we prepared an automatic startup of the whole system using a ROS 2 launch file and a simple shell script.

The script:
- sets the required environment variable `ROS_DOMAIN_ID=0`,
- sources the ROS 2 Humble environment,
- sources the project workspace,
- launches `camera_turtle.launch.py`, which starts:
  - the USB camera driver (`usb_cam_node_exe`),
  - the custom control node `camera_node`,
  - the `turtlesim_node`.

Thanks to this, the whole demo can be started with a **single command**:  
`./run_camera_project.sh`

**Screenshot (Stage 5):**  
![Stage 5 — automatic launch](stage5_auto_launch.png)

## Stage 6 – ArUco-based robot control (+0.5)

As an extension of the mouse-based interface, we implemented a vision-based robot control mode using ArUco markers.

Camera input:
image stream provided on /image_raw.

ArUco detection:
implemented using OpenCV ArUco module (DICT_4X4_50),
the center of the detected marker is computed based on its corner coordinates.
Control logic implemented in camera_node.py:
marker above the vertical center of the image → FORWARD → linear.x = 1.0,
marker below the vertical center of the image → BACKWARD → linear.x = -1.0,
no marker detected → robot stops (linear.x = 0.0).

Velocity commands (geometry_msgs/Twist) are published on /turtle1/cmd_vel, enabling control of the robot in turtlesim.

This stage demonstrates an alternative human–robot interaction method based solely on visual feedback, without the use of mouse input.

Proof of operation: screenshots in docs/ (ArUCo marker visible in camera view, robot motion in turtlesim, node logs).

## Stage 7 – Dockerized application (+0.5)

As an additional extension, the entire ROS 2 application was containerized using Docker, allowing the system to be executed without local installation of ROS 2 and project dependencies. The Docker image is built from the root of the repository using the following command:

docker build -t camera_project:humble .

To run the application inside the container with access to the camera and graphical interface, GUI access must be enabled first:

xhost +local:root

The container is then started with:

docker run --rm -it \
--net=host \
--device=/dev/video0 \
-e DISPLAY=$DISPLAY \
-v /tmp/.X11-unix:/tmp/.X11-unix \
camera_project:humble

After finishing the demonstration, GUI access is revoked using:

xhost -local:root

This setup launches the complete system automatically inside Docker, including the camera driver, custom control node, and turtlesim. Both mouse-based control and ArUco marker control work correctly inside the container.

Proof of operation: screenshots in docs/ (Docker container running, camera view, turtlesim window).



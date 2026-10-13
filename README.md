# ros2-aruco-world

A Gazebo room with ArUco markers in different places and poses, a small robot with a camera, and a
ROS 2 node that detects the markers live and places them around the robot in RViz. Built to see
what a robot camera actually gets from a marker as distance and viewing angle change.

ROS 2 Jazzy, Gazebo Harmonic, Ubuntu 24.04 (tested under WSL2 on Windows 11).

## The problem

A fiducial marker is easy to read straight on and close up. A robot rarely sees it that way: the
marker is far, turned away, on the floor, or above the camera. Testing that on real hardware is
slow; a simulated camera shows the same effect in seconds and makes every case repeatable.

## The world

Ten markers (DICT_4X4_50, 15 cm) placed to cover a range of cases rather than the easy one:

- **near and head-on**, on a post in front of the robot;
- **turned away**, on posts turned at increasing angles;
- **flat on the floor**, ahead of the robot;
- **far**, on the far wall;
- **above the camera**, mounted high on the walls and tilted down;
- **out of the starting view**, on the side walls, the back wall and the side of a box, so the
  robot has to turn to find them.

The exact position and facing of each marker live in one place, `aruco_world/scripts/build_world.py`,
which prints how each one is seen from the starting pose every time it regenerates the world.

## The robot and its frames

The robot is described once, in `aruco_world/urdf/camera_bot.urdf.xacro`. `robot_state_publisher`
publishes that description and its TF, and Gazebo spawns the robot from the same description.

```
odom -> base_footprint -> base_link -> camera_link -> camera_link_optical
                                    -> left_wheel, right_wheel
```

`odom -> base_footprint` is the wheel odometry from Gazebo's DiffDrive system; the wheels turn with
the joint states Gazebo publishes; the rest is fixed. Images and detections are stamped in
`camera_link_optical`, so TF can place any detection relative to the robot or in `odom`.

## The detector

`aruco_detector` subscribes to a camera image and its calibration, and for every frame publishes:

- `/aruco/image`: the frame with each marker outlined, its axes drawn, and its id, distance and
  viewing angle;
- `/aruco/detections` (`aruco_interfaces/MarkerDetectionArray`): id, pose, distance and viewing
  angle of every marker, with the image's stamp and frame;
- `/aruco/markers` (`visualization_msgs/MarkerArray`): each detected marker as a panel with its id,
  for RViz.

It knows nothing about the simulation: remap `image` and `camera_info` to any camera.

## Architecture

```
camera_bot.urdf.xacro -> robot_state_publisher -> /robot_description, TF
                                |
                                v
Gazebo (spawns the robot) -> ros_gz_bridge -> /camera/image, /camera/camera_info, /odom,
                                              /tf (odom -> base_footprint), /joint_states, /clock
                                                     |
                                                     v
                                              aruco_detector -> /aruco/image, /aruco/detections,
                                                                /aruco/markers -> RViz
```

| Package | Build type | Holds |
|---|---|---|
| `aruco_world` | ament_cmake | world generator, robot URDF, Gazebo bridge, launch, RViz view |
| `aruco_detector` | ament_python | the detection node, its core without ROS, and tests |
| `aruco_interfaces` | ament_cmake | the `MarkerDetection` and `MarkerDetectionArray` messages |

## Key technical decisions

- **The world is generated, not hand-written.** Each marker is described by its position and the
  direction it faces; the script computes the rotation (keeping wall markers upright and away
  from the Euler angle singularity) and writes both the marker models and the world. Moving a
  marker is a one-line change.
- **One model per marker**, a thin panel with the marker and its white quiet zone as a texture,
  so the same id can be placed anywhere with `<include>`.
- **One description of the robot.** The URDF drives both Gazebo and TF, so the camera that renders
  the images and the frame that places the detections cannot drift apart. The frames follow
  REP 105 and REP 103, and Gazebo stamps the camera's images in the optical frame. The SDF in
  `models/camera_bot/` is kept for reference only.
- **One source for what the world defines.** `build_world.py` writes the detector's parameters and
  the robot's start pose next to the markers. The node's parameters have no defaults: a missing
  size stops it at start-up instead of silently scaling every distance.
- **The detector is its own package** and does not depend on the simulation, so the same node runs
  on a real camera topic. The simulation's launch starts it and remaps its inputs.
- **A detection core without ROS** (`aruco_detector/aruco_detector/detection.py`), tested on
  synthetic images rendered from known poses, on OpenCV 4.6 (Ubuntu 24.04) and on the current
  release (CI). Both ArUco APIs are supported, chosen once at start-up.
- **Subpixel corners and SQPnP.** The pose comes from four corners, so they are refined to subpixel
  precision; SQPnP is globally optimal, so it returns the pose that best fits them and never flips
  a marker to face away from the camera.
- **Simulated time where the clock is read.** `robot_state_publisher` and RViz run on Gazebo's
  clock; the detector only copies each image's stamp, so it stays off `/clock`, which arrives at
  every physics step.
- **RViz shows only what is in view.** Each marker message starts by deleting the previous frame's
  markers, rather than letting them expire after a guessed time.
- **Gazebo's server and window run apart**, so losing the window never stops the simulation or
  the camera.
- **Messages follow REP 103** (metres and radians). The subscriptions use the sensor QoS profile
  (late frames are dropped, not queued); the publishers keep the default reliable profile, so any
  viewer can subscribe.

## Limitations

- Rendered images are clean: no sensor noise, motion blur or lens distortion, so detection here is
  a best case.
- Lighting is fixed: one sun that never changes.
- A marker's tilt shows only as perspective, which shrinks with the square of the distance: for a
  far, nearly head-on marker the viewing angle and orientation are poorly determined, while the
  distance still is.
- Detection uses OpenCV's default parameters on purpose, so small, oblique markers near the
  reading limit come and go, as they would with a stock detector.
- Simulated wheels do not slip, so odometry hardly drifts and markers placed in `odom` stay put;
  a real robot's odometry drifts.
- On modest hardware the simulation runs slower than real time, and the camera's rate drops with
  it.

## Quickstart

Under WSL2, set `GALLIUM_DRIVER=d3d12` so Gazebo renders on the GPU, and on a machine with more
than one GPU, `MESA_D3D12_DEFAULT_ADAPTER_NAME` (for example `NVIDIA`) to pick which one.

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/KevDP/ros2-aruco-world.git   # or ln -s /path/to/this/repo
cd ~/ros2_ws && colcon build --symlink-install
source install/setup.bash
ros2 launch aruco_world world.launch.py
```

It opens Gazebo and RViz. Launch arguments: `gz_gui:=false` (RViz only, lighter), `rviz:=false`,
`image_view:=true` (a separate camera window), `detector:=false` (raw camera only).

Drive it from another terminal (keys `i`, `j`, `l`, `,`, `k`), and watch the detections from a third:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

```bash
ros2 topic echo /aruco/detections
```

Look at the frames:

```bash
ros2 run tf2_ros tf2_echo odom camera_link_optical
```

```bash
ros2 run tf2_tools view_frames
```

Run the detector on any other camera:

```bash
ros2 run aruco_detector detector --ros-args -p dictionary:=DICT_4X4_50 -p marker_length:=0.15 -r image:=/my_camera/image -r camera_info:=/my_camera/camera_info
```

Save a frame, raw and annotated:

```bash
ros2 run aruco_world camera_snapshot.py --out /tmp/aruco
```

After editing a placement in `build_world.py`, regenerate and rebuild (from the repository root):

```bash
cd aruco_world && python3 scripts/build_world.py && cd ~/ros2_ws && colcon build --symlink-install
```

Lint and test without ROS:

```bash
pip install ruff pytest numpy opencv-python-headless
ruff check . && pytest -q aruco_detector/test
```

## Repository structure

```
aruco_world/           simulation package
  config/              Gazebo bridge; detector parameters and robot start (generated)
  launch/              world.launch.py
  models/              aruco_<id> (generated); camera_bot, the robot's SDF, for reference
  rviz/                aruco_world.rviz
  scripts/             build_world.py (generator), camera_snapshot.py
  urdf/                camera_bot.urdf.xacro, the robot
  worlds/              aruco_room.sdf (generated)
aruco_detector/        detection package
  aruco_detector/      detection.py (no ROS), node.py
  test/                detection tests on synthetic images
aruco_interfaces/      MarkerDetection and MarkerDetectionArray
.github/workflows/     CI: ruff and the detection tests
```

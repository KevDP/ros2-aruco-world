# ros2-aruco-world

A Gazebo room with ArUco markers in different places and poses, a small robot with a camera, and a
ROS 2 node that detects the markers live. Built to see what a robot camera actually gets from a
marker as distance and viewing angle change.

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

## The detector

`aruco_detector` subscribes to a camera image and its calibration, and for every frame publishes:

- `/aruco/image`: the frame with each marker outlined, its axes drawn, and its id, distance and
  viewing angle;
- `/aruco/detections` (`aruco_interfaces/MarkerDetectionArray`): id, pose, distance and viewing
  angle of every marker, with the image's stamp and frame.

It knows nothing about the simulation: remap `image` and `camera_info` to any camera.

## Architecture

```
Gazebo camera -> ros_gz_bridge -> /camera/image, /camera/camera_info
                                        |
                                        v
                                 aruco_detector -> /aruco/image -> rqt_image_view
                                        |
                                        +-------> /aruco/detections
```

| Package | Build type | Holds |
|---|---|---|
| `aruco_world` | ament_cmake | world generator, robot, Gazebo bridge, launch |
| `aruco_detector` | ament_python | the detection node, its core without ROS, and tests |
| `aruco_interfaces` | ament_cmake | the `MarkerDetection` and `MarkerDetectionArray` messages |

## Key technical decisions

- **The world is generated, not hand-written.** Each marker is described by its position and the
  direction it faces; the script computes the rotation (keeping wall markers upright and away
  from the Euler angle singularity) and writes both the marker models and the world. Moving a
  marker is a one-line change.
- **One model per marker**, a thin panel with the marker and its white quiet zone as a texture,
  so the same id can be placed anywhere with `<include>`.
- **The detector is its own package** and does not depend on the simulation, so the same node runs
  on a real camera topic. The simulation's launch starts it and remaps its inputs.
- **One source for the marker size.** `build_world.py` writes the detector's parameters
  (`aruco_world/config/aruco_markers.yaml`) next to the markers. The node's parameters have no
  defaults: a missing size stops it at start-up instead of silently scaling every distance.
- **A detection core without ROS** (`aruco_detector/aruco_detector/detection.py`), tested on
  synthetic images rendered from known poses, on OpenCV 4.6 (Ubuntu 24.04) and on the current
  release (CI). Both ArUco APIs are supported, chosen once at start-up.
- **Subpixel corners and SQPnP.** The pose comes from four corners, so they are refined to subpixel
  precision; SQPnP is globally optimal, so it returns the pose that best fits them and never flips
  a marker to face away from the camera.
- **Messages follow REP 103** (metres and radians), with poses in the camera optical frame.
- **QoS:** the subscriptions use the sensor profile (late frames are dropped, not queued); the
  publishers keep the default reliable profile, so any viewer can subscribe.
- **Plain SDF for the robot**, with Gazebo's DiffDrive system and camera sensor, bridged to
  standard ROS 2 topics. A URDF with `robot_state_publisher` is the natural next step.

## Limitations

- Rendered images are clean: no sensor noise, motion blur or lens distortion, so detection here is
  a best case.
- Lighting is fixed: one sun that never changes.
- A marker's tilt shows only as perspective, which shrinks with the square of the distance: for a
  far, nearly head-on marker the viewing angle and orientation are poorly determined, while the
  distance still is.
- Detection uses OpenCV's default parameters on purpose, so small, oblique markers near the
  reading limit come and go, as they would with a stock detector.
- Poses are in the camera optical frame; without a TF tree they are not placed in the room yet.

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

`detector:=false` shows the raw camera instead; `image_view:=false` opens no window.

Drive it from another terminal (keys `i`, `j`, `l`, `,`, `k`), and watch the detections from a third:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

```bash
ros2 topic echo /aruco/detections
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
  config/              Gazebo bridge; detector parameters (generated)
  launch/              world.launch.py
  models/              camera_bot (hand-written) and aruco_<id> (generated)
  scripts/             build_world.py (generator), camera_snapshot.py
  worlds/              aruco_room.sdf (generated)
aruco_detector/        detection package
  aruco_detector/      detection.py (no ROS), node.py
  test/                detection tests on synthetic images
aruco_interfaces/      MarkerDetection and MarkerDetectionArray
.github/workflows/     CI: ruff and the detection tests
```

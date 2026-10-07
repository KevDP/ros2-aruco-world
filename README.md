# aruco_world

A Gazebo room with ArUco markers in different places and poses, and a small robot with a camera
to look at them. Built to see, in simulation, what a robot camera actually gets from a marker as
distance and viewing angle change.

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

The exact position and facing of each marker live in one place, `scripts/build_world.py`, which
prints how each one is seen from the starting pose every time it regenerates the world.

## Architecture

```
scripts/build_world.py   marker placements -> models/aruco_<id>/ + worlds/aruco_room.sdf
models/camera_bot/        differential drive robot, forward camera (gz DiffDrive + camera sensor)
config/bridge.yaml        Gazebo <-> ROS 2: /cmd_vel, /odom, /camera/image, /camera/camera_info, /clock
launch/world.launch.py    Gazebo + bridge + rqt_image_view on /camera/image
scripts/camera_snapshot.py one frame -> snapshot.png + snapshot_detected.png
```

## Key technical decisions

- **The world is generated, not hand-written.** Each marker is described by its position and the
  direction it faces; the script computes the rotation (keeping wall markers upright and away
  from the Euler angle singularity) and writes both the marker models and the world. Moving a
  marker is a one-line change.
- **One model per marker**, a thin panel with the marker and its white quiet zone as a texture,
  so the same id can be placed anywhere with `<include>`.
- **Plain SDF for the robot**, with Gazebo's DiffDrive system and camera sensor, bridged to
  standard ROS 2 topics. A URDF with `robot_state_publisher` is the natural next step.
- **Detection runs on the bridged image**, the same way it would on a real camera topic.

## Limitations

- Rendered images are clean: no sensor noise, motion blur or lens distortion, so detection here is
  a best case.
- Lighting is fixed: one sun that never changes.

## Quickstart

Under WSL2, set `GALLIUM_DRIVER=d3d12` so Gazebo renders on the GPU, and on a machine with more
than one GPU, `MESA_D3D12_DEFAULT_ADAPTER_NAME` (for example `NVIDIA`) to pick which one.

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
ln -s /path/to/this/repo aruco_world      # or clone it here
cd ~/ros2_ws && colcon build --symlink-install --packages-select aruco_world
source install/setup.bash
ros2 launch aruco_world world.launch.py
```

Drive it from another terminal (keys `i`, `j`, `l`, `,`, `k`):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Save a frame with detections:

```bash
ros2 run aruco_world camera_snapshot.py --out /tmp/aruco
```

After editing a placement in `scripts/build_world.py`, regenerate and rebuild:

```bash
python3 scripts/build_world.py && cd ~/ros2_ws && colcon build --symlink-install --packages-select aruco_world
```

## Repository structure

```
config/      ROS 2 <-> Gazebo bridge
launch/      world.launch.py
models/      camera_bot (hand-written) and aruco_<id> (generated)
scripts/     build_world.py (generator), camera_snapshot.py (one frame + detection)
worlds/      aruco_room.sdf (generated)
```

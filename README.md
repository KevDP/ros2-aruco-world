# ros2-aruco-world

A Gazebo room with ArUco markers in different places and poses, a small robot with a camera and a
2D lidar, a ROS 2 node that detects the markers live and places them around the robot in RViz, a
node that places the robot in the map from the markers it sees, and a guard that stops the robot
before it drives into what the lidar sees. Built to see what a robot camera actually gets from a
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

## The robot and its frames

The robot is described once, in `aruco_world/urdf/camera_bot.urdf.xacro`. `robot_state_publisher`
publishes that description and its TF, and Gazebo spawns the robot from the same description.

```
map -> odom -> base_footprint -> base_link -> camera_link -> camera_link_optical
                                           -> lidar_link
                                           -> left_wheel, right_wheel
```

`map` is Gazebo's world frame, the one the markers are placed in. `map -> odom` is the robot's
start pose, published once: the simulated odometry barely drifts, so that holds until a filter
estimates it. `odom -> base_footprint` is the wheel odometry from Gazebo's DiffDrive system; the
wheels turn with the joint states Gazebo publishes; the rest is fixed. Images and detections are
stamped in `camera_link_optical` and scans in `lidar_link`, so TF can place any of them relative to
the robot or in the map.

For checking estimates, Gazebo also publishes the robot's true pose in the map on `/ground_truth`
(`nav_msgs/Odometry`). It never reaches `/tf`, so no node can use it as a sensor.

## The detector

`aruco_detector` subscribes to a camera image and its calibration, and for every frame publishes:

- `/aruco/image`: the frame with each marker outlined, its axes drawn, and its id, distance and
  viewing angle;
- `/aruco/detections` (`aruco_interfaces/MarkerDetectionArray`): id, pose, distance and viewing
  angle of every marker, with the image's stamp and frame;
- `/aruco/markers` (`visualization_msgs/MarkerArray`): each detected marker as a panel with its id,
  for RViz.

It knows nothing about the simulation: remap `image` and `camera_info` to any camera.

## Where the markers put the robot

`aruco_localization` turns each detected marker whose place is known into a fix of the robot's pose
in the map: the marker's pose in the map, composed with the inverse of where the camera saw it and
the inverse of where the camera sits on the robot. The robot drives on the floor, so a fix keeps x,
y and heading.

Each fix carries a covariance that grows with distance, unevenly. The distance to a marker is
measured well and its orientation less so, and an orientation error swings the camera around the
marker: the fix spreads across the line of sight by the distance times the angle. Markers beyond a
distance limit give no fix at all, because a small marker seen nearly head-on has an ambiguous
orientation and its fix can land metres away.

It publishes:

- `/aruco/robot_pose` (`geometry_msgs/PoseWithCovarianceStamped`): one fix per marker in range,
  stamped like its image, ready for a filter to fuse;
- `/aruco/robot_pose_markers` (`visualization_msgs/MarkerArray`): the fixes as arrows with their
  2-sigma ellipses, for RViz;
- `/aruco/marker_map` (`visualization_msgs/MarkerArray`): where the map says each marker is,
  latched.

The marker map is generated with the world, in `aruco_world/config/marker_map.yaml`; the distance
limit and the spread are in `aruco_world/config/aruco_localizer.yaml`. Each fix stands alone here:
fusing them with the odometry is the next step.

## The lidar and the guard

A 360-degree 2D lidar sits on a mast above the camera, so nothing on the robot blocks its view.
Gazebo renders it with the datasheet numbers of two common low-cost time-of-flight units, taking
the less favourable one where they differ: 500 rays per turn, 10 turns per second, 0.05 to 12 m,
about 1 cm of range noise.

`scan_guard` sits between whoever drives and the robot. It reads the driver's command on
`/cmd_vel_teleop` and the scan on `/scan`, and it owns `/cmd_vel`:

- an obstacle in the **stop zone**, just past the robot's leading edge, stops the robot, and the
  command is dropped, so the robot stays put after the obstacle leaves until it is driven again;
- an obstacle in the **slow zone**, further out, caps the speed;
- both zones are as wide as the robot plus a margin, and only the direction of travel is checked,
  so a robot blocked ahead can still back away;
- with no recent scan, it sends zero velocity.

It also publishes the zones on `/scan_guard/zones` for RViz, faint while clear and solid while
something is in them.

## Architecture

```
camera_bot.urdf.xacro -> robot_state_publisher -> /robot_description, TF
                                |
                                v
Gazebo (spawns the robot) -> ros_gz_bridge -> /camera/image, /camera/camera_info, /scan, /odom,
                                              /tf (odom -> base_footprint), /joint_states, /clock,
                                              /ground_truth (checking only)
                                                     |
                                                     v
                                              aruco_detector -> /aruco/image, /aruco/markers -> RViz
                                                     |
                                                     v /aruco/detections
                                              aruco_localizer (marker map, TF)
                                                     -> /aruco/robot_pose
                                                     -> /aruco/robot_pose_markers, /aruco/marker_map -> RViz

static map -> odom (the start pose)

teleop -> /cmd_vel_teleop -> scan_guard (with /scan) -> /cmd_vel -> ros_gz_bridge -> Gazebo
                                        |
                                        v
                                  /scan_guard/zones -> RViz
```

| Package | Build type | Holds |
|---|---|---|
| `aruco_world` | ament_cmake | world generator, robot URDF, Gazebo bridge, launch, RViz view |
| `aruco_detector` | ament_python | the detection node, its core without ROS, and tests |
| `aruco_interfaces` | ament_cmake | the `MarkerDetection` and `MarkerDetectionArray` messages |
| `aruco_localization` | ament_python | the localizer node, its pose and covariance core without ROS, and tests |
| `scan_guard` | ament_python | the lidar guard node, its decision core without ROS, and tests |

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
- **One source for what the world defines.** `build_world.py` writes the detector's parameters,
  the marker map and the robot's start pose next to the markers. The nodes' parameters have no
  defaults: a missing size stops the detector at start-up instead of silently scaling every
  distance, and a missing marker stops the localizer.
- **Wheels touch the floor at one point.** Their collision shape is a sphere, not the cylinder they
  are drawn as: a cylinder touches the floor along its rim's edges, and turning on the inner edges
  shrinks the wheel separation the odometry assumes.
- **The truth stays out of `/tf`.** Gazebo's true pose of the robot goes to `/ground_truth`, which
  only checks estimates; nothing the robot runs reads it.
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
- **The guard owns `/cmd_vel`.** The robot's base listens on `/cmd_vel`, and anything that can veto
  a command sits just before it; drivers publish on their own topic. Adding a planner later means
  remapping its output, not touching the guard or the bridge.
- **No data, no motion.** Without a recent scan, or before TF says where the lidar sits, the guard
  sends zero velocity. It measures scan age in wall time, so a paused simulation holds the robot
  too.
- **A stop is latched; a slowdown is not.** A robot stopped by something in its way does not set
  off again by itself when the way clears.
- **One source for the robot's size.** The launch reads the guard's outline of the robot from the
  URDF's properties; only the zone sizes live in `config/scan_guard.yaml`.
- **A guard core without ROS** (`scan_guard/scan_guard/guard.py`), tested on synthetic scans with
  obstacles at known positions; the node only feeds it and publishes what it decides.
- **A localizer core without ROS** (`aruco_localization/aruco_localization/localization.py`),
  tested on detections computed from known robot poses. Each fix is a planar pose with its own
  covariance, so a filter can weigh near and far markers without knowing where they came from.
- **One fix per marker, not an average.** Fusing belongs to a filter that also knows the odometry;
  averaging here would hide which marker said what.

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
- On modest hardware the simulation runs slower than real time, and the camera's and lidar's rates
  drop with it.
- The simulated lidar is ideal apart from range noise: dark or glossy surfaces return as well as
  any other, glass and mirrors behave like walls, sunlight never blinds it, and each scan is taken
  at one instant instead of over a turn while the robot moves. Those are what a real unit has to be
  tested on.
- A 2D lidar sees one plane, 0.30 m above the floor here: anything lower, like the floor marker, or
  overhanging is invisible to it.
- The guard is a teaching example, not a certified safety function. Turning in place is not
  checked, and an obstacle must return at least `min_points` rays to count, which a thin object
  far away may not (rays are 1.3 cm apart at 1 m).
- Each marker fix stands alone: with no marker in range the robot has no fix, and nothing smooths
  the fixes over time.
- A fix's covariance leaves out the link between its sideways spread and its heading error, which
  come from the same orientation error, so it counts one marker as a little more informative than
  it is.
- Distances from the detector carry a small bias here, larger on oblique markers; the localizer's
  covariance is wide enough to cover it instead of correcting it.

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
`image_view:=true` (a separate camera window), `detector:=false` (raw camera only), `guard:=false`
(no lidar guard).

Drive it from another terminal (keys `i`, `j`, `l`, `,`, `k`). The guard listens on
`/cmd_vel_teleop`, so teleop is remapped there:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_teleop
```

With `guard:=false`, drop the remap and teleop drives `/cmd_vel` directly.

Watch the detections, the marker fixes and the lidar from other terminals:

```bash
ros2 topic echo /aruco/detections
```

```bash
ros2 topic echo /aruco/robot_pose --field pose.pose.position
```

```bash
ros2 topic hz /scan
```

Compare a fix with where the robot really is:

```bash
ros2 topic echo /ground_truth --field pose.pose.position
```

Look at the frames:

```bash
ros2 run tf2_ros tf2_echo map camera_link_optical
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

Lint and test without ROS (one pytest run per package, so each finds its own settings):

```bash
pip install ruff pytest numpy opencv-python-headless
ruff check . && pytest -q aruco_detector/test && pytest -q aruco_localization/test && pytest -q scan_guard/test
```

## Repository structure

```
aruco_world/           simulation package
  config/              Gazebo bridge, guard zones, localizer limits; detector parameters, marker
                       map and robot start (generated)
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
aruco_localization/    localization package
  aruco_localization/  localization.py (no ROS), node.py
  test/                localization tests on detections from known poses
scan_guard/            lidar guard package
  scan_guard/          guard.py (no ROS), node.py
  test/                guard tests on synthetic scans
.github/workflows/     CI: ruff and the detection, localization and guard tests
```

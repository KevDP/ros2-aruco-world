# ros2-aruco-world

A ROS 2 and Gazebo setup for studying what a robot's camera gets from fiducial markers as distance
and viewing angle change, and what the robot can do with it.

A small differential robot with a camera, a 2D lidar and an IMU moves around a room with ArUco
markers in different places and poses. On top of it run a live marker detector, a localizer that
works out the robot's place in the map from each marker it sees, two `robot_localization` filters
that fuse those fixes with drifting wheel odometry and a gyro, and a guard that stops the robot
before it drives into whatever the lidar sees.

ROS 2 Jazzy, Gazebo Harmonic, Ubuntu 24.04 (tested under WSL2 on Windows 11).

## The problem

A fiducial marker is easy to read straight on and close up. A robot rarely gets that view. The
marker is far away, turned, on the floor or above the camera. Testing those cases on real hardware
is slow. A simulated camera shows the same effects in seconds and makes every case repeatable.

## The world

Ten markers (DICT_4X4_50, 15 cm) cover a range of cases, from easy to hard:

- **near and head-on**, on a post in front of the robot
- **turned away**, on posts turned at increasing angles
- **flat on the floor**, ahead of the robot
- **far**, on the far wall
- **above the camera**, mounted high on the walls and tilted down
- **out of the starting view**, on the side walls, the back wall and the side of a box, where the
  robot has to turn to find them

Every marker's position and facing is set in `aruco_world/scripts/build_world.py`. Each time the
script regenerates the world, it also prints how each marker looks from the robot's starting pose.

## The robot and its frames

The robot is described once, in `aruco_world/urdf/camera_bot.urdf.xacro`. `robot_state_publisher`
turns that description into TF, and Gazebo builds the robot from it too.

```
map -> odom -> base_footprint -> base_link -> camera_link -> camera_link_optical
                                           -> lidar_link
                                           -> imu_link
                                           -> left_wheel, right_wheel
```

`map` is Gazebo's world frame, the one the markers are placed in. The local filter (wheel odometry
and gyro) publishes `odom -> base_footprint`, which is smooth but drifts. The global filter adds the
marker fixes and publishes `map -> odom`, the drift those fixes have found. The wheel frames follow
the joint states Gazebo publishes and every other link is fixed. Images and detections are stamped
in `camera_link_optical`, scans in `lidar_link` and IMU readings in `imu_link`. TF can then place
any of them relative to the robot or in the map.

Gazebo also publishes where the robot actually is on `/ground_truth` (`nav_msgs/Odometry`), for
checking estimates. That topic never reaches `/tf`, and none of the robot's nodes subscribe to
it.

## The detector

For every camera frame, `aruco_detector` publishes:

- `/aruco/image`: the camera frame annotated with every marker's outline, axes, id, distance and
  viewing angle
- `/aruco/detections` (`aruco_interfaces/MarkerDetectionArray`): id, pose, distance and viewing
  angle of every marker, with the image's stamp and frame
- `/aruco/markers` (`visualization_msgs/MarkerArray`): a panel and an id for each detected marker,
  for RViz

It subscribes to an image and its calibration and has no tie to the simulation. Remapping `image`
and `camera_info` is enough to run it on any camera.

## Where the markers put the robot

For each detected marker whose place is known, `aruco_localization` works out where the robot must
be for the camera to see the marker the way it does. It chains three transforms: the marker's pose
in the map, the inverse of the marker's pose as the camera saw it, and the inverse of the camera's
pose on the robot. The robot moves on the floor, and a fix keeps only x, y and heading.

Each fix comes with a covariance that grows with distance, and not equally in every direction. The
detector measures the distance to a marker well and its orientation less well. A small error in the
orientation rotates the estimated camera position around the marker, and the fix spreads sideways by
about the distance times the angle. Past a distance limit no fix is published at all. A small marker
seen almost head-on has an ambiguous orientation, and its fix can land metres off.

It publishes:

- `/aruco/robot_pose` (`geometry_msgs/PoseWithCovarianceStamped`): one fix per marker in range,
  stamped with its image's time, ready for a filter
- `/aruco/robot_pose_markers` (`visualization_msgs/MarkerArray`): the fixes as arrows with 2-sigma
  ellipses, for RViz
- `/aruco/marker_map` (`visualization_msgs/MarkerArray`): each marker where the map puts it, latched

The marker map is generated with the world, into `aruco_world/config/marker_map.yaml`. The distance
limit and the spread live in `aruco_world/config/aruco_localizer.yaml`.

## Fusing odometry, gyro and markers

Gazebo's wheel odometry is close to perfect, and no real robot's is. `noisy_odometry` publishes a
drifting copy of it. Each step of motion gets a random, zero-mean error sized by the distance
travelled and the angle turned, and the errors build up as a random walk. The robot's nodes only
ever see that copy, on `/wheel/odometry`.

Two `robot_localization` filters then estimate the robot's pose, following the split REP 105
describes:

- **`ekf_local`** combines the wheel odometry (forward speed and turn rate, with sideways speed held
  at zero) and the gyro's turn rate. Its pose in `odom` moves smoothly and never jumps, at the cost
  of drifting. It publishes `odom -> base_footprint` and `/odometry/local`.
- **`ekf_global`** takes the same inputs plus the marker fixes. Its pose in `map` does not drift,
  but it moves when a fix corrects it. It publishes `map -> odom` and `/odometry/global`.

A fix reaches the filter after a camera frame and a detection, later than the image it came from.
The global filter rewinds to the image's time to apply it. Fixes that disagree too much with the
odometry and the gyro are rejected. Both filters are configured in `aruco_world/config/ekf.yaml`,
and the drift in `aruco_world/config/noisy_odometry.yaml`.

## The lidar and the guard

A 360-degree 2D lidar sits on a mast above the camera, where nothing on the robot blocks it. Its
simulated specs come from the datasheets of two common low-cost time-of-flight units, taking the
worse value where they differ: 500 rays per turn, 10 turns per second, a range of 0.05 to 12 m and
about 1 cm of noise.

`scan_guard` sits between whoever drives and the robot. It reads the driver's command on
`/cmd_vel_teleop` and the scan on `/scan`, and it is the only node that publishes on `/cmd_vel`.

- An obstacle in the **stop zone**, just past the robot's leading edge, stops the robot. The guard
  keeps the command, and the robot goes on with it once the obstacle is gone.
- An obstacle in the **slow zone**, further out, caps the speed.
- Both zones match the robot's width plus a margin. Only the direction of travel is checked, and a
  robot blocked ahead can still back away.
- Without a recent scan, it sends zero velocity and drops the command. A sensor that went quiet
  needs someone to look at it, and the robot waits for a new command.

It also draws the zones on `/scan_guard/zones` for RViz, faint while clear and solid while
something is in them.

## Architecture

```
camera_bot.urdf.xacro -> robot_state_publisher -> /robot_description, TF
                                |
                                v
Gazebo (spawns the robot) -> ros_gz_bridge -> /camera/image, /camera/camera_info, /scan, /imu,
                                              /sim/wheel_odometry, /joint_states, /clock,
                                              /ground_truth (checking only)
                                                     |
                                                     v
                                              aruco_detector -> /aruco/image, /aruco/markers -> RViz
                                                     |
                                                     v /aruco/detections
                                              aruco_localizer (marker map, TF)
                                                     -> /aruco/robot_pose
                                                     -> /aruco/robot_pose_markers, /aruco/marker_map -> RViz

/sim/wheel_odometry -> noisy_odometry -> /wheel/odometry
/wheel/odometry + /imu                     -> ekf_local  -> odom -> base_footprint, /odometry/local
/wheel/odometry + /imu + /aruco/robot_pose -> ekf_global -> map -> odom, /odometry/global -> RViz

teleop -> /cmd_vel_teleop -> scan_guard (with /scan) -> /cmd_vel -> ros_gz_bridge -> Gazebo
                                        |
                                        v
                                  /scan_guard/zones -> RViz
```

| Package | Build type | Holds |
|---|---|---|
| `aruco_world` | ament_cmake | world generator, robot URDF, Gazebo bridge, filter settings, launch, RViz view |
| `aruco_detector` | ament_python | the detection node, its core without ROS, and tests |
| `aruco_interfaces` | ament_cmake | the `MarkerDetection` and `MarkerDetectionArray` messages |
| `aruco_localization` | ament_python | the localizer node, its pose and covariance core without ROS, and tests |
| `noisy_odometry` | ament_python | the node that makes the wheel odometry drift, its model without ROS, and tests |
| `scan_guard` | ament_python | the lidar guard node, its decision core without ROS, and tests |

## Key technical decisions

### World

- The world is generated by `build_world.py`. Each marker is given by its position and the
  direction it faces, and the script works out the rotation, keeping wall markers upright and away
  from the Euler angle singularity. Moving a marker is a one-line change.
- The same script writes the detector's parameters, the marker map and the robot's start pose.
  Nothing that depends on a marker's place is copied by hand. The nodes have no defaults for those
  parameters. A missing marker size stops the detector at start-up, before it can scale every
  distance by the wrong factor, and a missing marker stops the localizer.
- Each marker is its own model, a thin plain panel with the marker on a one-sided plane just in
  front of it. A printed marker can only be read from the front. One readable from behind would put
  the robot on the wrong side of it.

### Robot

- One URDF drives both Gazebo and TF. The camera that renders the images and the frame that places
  the detections cannot drift apart. Frame names and axes follow REP 105 and REP 103, and Gazebo
  stamps camera images in the optical frame. The SDF in `models/camera_bot/` is kept for reference
  only.
- The wheels are drawn as cylinders and collide as spheres. A cylinder rests on
  the edges of its rim, and turning on the inner edges shortens the wheel separation the odometry
  assumes. A sphere touches the floor right under the wheel's centre. The cylinder stays available
  as a launch argument for comparison.
- Every speed and acceleration limit of the base is set in both directions. Gazebo's speed limiter
  leaves any bound it is not given at infinity. With only the maximums set, the robot would stop
  instantly and could turn right or reverse with no limit.
- Simulated time is used only where a node reads the clock. `/clock` arrives at every physics step,
  and the detector, which only copies each image's stamp, stays off it.
- Gazebo's server and window run as separate processes. Losing the window never stops the
  simulation or the camera.

### Detection

- The detector is its own package and knows nothing about the simulation. The same node runs on a
  real camera topic, and the simulation's launch only starts it and remaps its inputs.
- Its core (`aruco_detector/aruco_detector/detection.py`) has no ROS dependency. It is tested on
  synthetic images rendered from known poses and on markers cut out of simulated camera frames,
  with OpenCV 4.6 (Ubuntu 24.04) locally and the current release in CI. Both ArUco APIs are
  supported, chosen once at start-up.
- Corners are refined to subpixel precision and the pose comes from SQPnP. Being globally optimal,
  it returns the pose that best fits the four corners and never flips a marker to face away from
  the camera.
- Bits are read from the middle of each cell of the marker's grid. Read whole, as OpenCV does by
  default, a cell a few pixels wide takes in the blur of its neighbours, and one wrong bit drops a
  `DICT_4X4_50` marker. Reading the middle keeps far markers readable at no cost in wrong ids.
  Correcting a misread bit was left off, because codes in this dictionary differ in as few as 4
  bits and a correction can turn one marker into another. `experiments/perception.py` measures
  both readings.
- The subscriptions use the sensor QoS profile, which drops late frames. Publishers keep the
  default reliable profile that any viewer can subscribe to. Each RViz marker message starts by
  deleting the previous frame's markers, which leaves no lifetime to guess.

### Localization and fusion

- The localizer's core (`aruco_localization/aruco_localization/localization.py`) has no ROS
  dependency and is tested on detections computed from known robot poses. Each fix is a planar pose
  with its own covariance. A filter can weigh near and far markers without knowing where the fixes
  came from.
- The localizer publishes one fix per marker. Fusing them is the job of a filter that also knows
  the odometry, and separate fixes show which marker said what.
- There are two filters, as in REP 105. Anything that needs smooth motion, like a controller or the
  guard's zones, reads `odom`, which never jumps. Anything that needs the robot's place in the room
  reads `map`, which does not drift.
- The odometry's drift is scaled to the motion, and publishing faster does not make it drift
  more.
- The simulated IMU's own orientation is switched off and the filters use only the gyro's rate.
  Gazebo's orientation would be the true heading, which a real low-cost IMU does not give.
- The global filter keeps a second of history and applies each fix at its image's stamp, however
  late the fix arrives.

### Guard

- The base listens on `/cmd_vel` and drivers publish on a topic of their own, which leaves room for
  anything that can veto a command to sit in between. A planner added later only needs its output
  remapped, with no change to the guard or the bridge.
- The guard holds the robot when it lacks data: no recent scan, or no TF yet for the lidar. Scan age
  and the command timer use a clock that never runs backwards. A paused simulation still holds the
  robot. A wall clock set back by NTP or by a virtual machine's time sync does not interrupt the
  commands.
- After a stop, the robot moves on with the same command once the way has been clear for a moment,
  long enough for whoever stepped in front to step away. The way also has to be clear a little past
  the stop zone, because lidar noise at the edge would otherwise release the robot. A slowdown lifts
  as soon as the way clears.
- The robot's outline comes from the URDF's properties, and only the zone sizes live in
  `config/scan_guard.yaml`. The guard's core (`scan_guard/scan_guard/guard.py`) has no ROS
  dependency and is tested on synthetic scans with obstacles at known positions.

### Experiments

- Each experiment in `experiments/` runs a fixed scenario on a fresh simulation, compares the result
  with Gazebo's true pose and keeps its raw numbers. Tables and figures are regenerated from them.

## Limitations

- Rendered images are clean, with no sensor noise, motion blur or lens distortion. Detection here is
  a best case.
- The lighting comes from a single sun that never moves.
- A marker's tilt only shows through perspective, which fades with the square of the distance. For a
  far marker seen almost head-on, the viewing angle and orientation are poorly determined even
  though the distance is not.
- A marker far enough away that its cells are only a few pixels wide is read part of the time.
  Reading it reliably would take bigger markers, a camera with more pixels, or a dictionary with
  codes far enough apart to correct a bit.
- The odometry's drift is only random and zero-mean. A real robot's also has a part that always
  pulls the same way, which is left out here.
- On modest hardware the simulation runs slower than real time, and the camera and lidar rates drop
  with it.
- Apart from range noise, the simulated lidar is ideal. Dark or glossy surfaces return like any
  other, glass and mirrors behave like walls, sunlight never blinds it, and a whole scan is captured
  at one instant. A real unit takes a tenth of a second per turn while the robot keeps moving. Those
  are the things to test on real hardware.
- A 2D lidar sees a single plane, 0.30 m above the floor here. Anything lower, such as the floor
  marker, or anything overhanging is invisible to it.
- The guard is a teaching example with no safety certification. It does not check turning in
  place, and an obstacle has to return at least `min_points` rays to count, which a thin object far
  away may not do (rays are 1.3 cm apart at 1 m).
- With no marker in range, the global estimate relies on odometry and the gyro alone and drifts
  until the next fix.
- A fix's covariance ignores that its sideways spread and its heading error come from the same
  orientation error, and it overstates how much a single marker tells the filter.
- The detector's distances come out slightly long, more on far and oblique markers. The
  localizer's covariance is wide enough to absorb it, and that puts a floor on how close the global
  estimate gets.
- The filters assume a flat floor and a level robot.

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
(no lidar guard), `wheel_collision:=cylinder` (the wheels' drawn shape as their collision, to
compare with the default sphere).

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

Compare a fix, and the global filter's estimate, with where the robot really is:

```bash
ros2 topic echo /ground_truth --field pose.pose.position
```

```bash
ros2 topic echo /odometry/global --field pose.pose.position
```

In RViz, the green trail is the truth and the blue one is the global estimate, drawn with its
uncertainty. The gap between the `map` and `odom` axes is the drift the markers have corrected.

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

Run an experiment against a fresh simulation and build its tables and figures (see
`experiments/README.md` for each one):

```bash
experiments/sim.sh -- python3 experiments/visibility.py
python3 experiments/visibility_report.py
```

Lint and test without ROS (one pytest run per package, so each finds its own settings):

```bash
pip install ruff pytest numpy opencv-python-headless
ruff check .
for p in aruco_detector aruco_localization noisy_odometry scan_guard; do pytest -q $p/test; done
```

## Repository structure

```
aruco_world/           simulation package
  config/              Gazebo bridge, guard zones, localizer limits, filters, odometry drift;
                       detector parameters, marker map and robot start (generated)
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
noisy_odometry/        odometry drift package
  noisy_odometry/      drift.py (no ROS), node.py
  test/                drift tests on synthetic motion
scan_guard/            lidar guard package
  scan_guard/          guard.py (no ROS), node.py
  test/                guard tests on synthetic scans
experiments/           scenarios checked against Gazebo's truth, their reports and results
.github/workflows/     CI: ruff and every package's tests
```

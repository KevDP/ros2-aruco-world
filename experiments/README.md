# Experiments

Each experiment runs the simulation with no windows, takes the robot through a fixed scenario,
compares what the robot measures with Gazebo's truth, and writes its numbers to
`results/<experiment>/`. A report script turns those numbers into tables and figures. Reports
run without ROS, on any machine with NumPy, Matplotlib and PyYAML.

| Experiment | Question | Scenario |
| --- | --- | --- |
| `visibility` | Which markers does the robot read from each spot in the room? | Gazebo places the robot on a 0.5 m grid over the free floor, facing 12 headings, and the detector runs on one frame per pose |
| `turning` | How far does the wheel odometry stray from the truth, per wheel collision shape? | Two turns in place at three rates and in both directions, and 2 m straight at two speeds, three times each |
| `camera_views` | Which markers in the camera's view does the detector read? | Gazebo places the robot at its start pose and turned toward the box. The markers in view come from projecting the world with the camera's true pose. `--stock` detects with OpenCV's defaults |
| `perception` | How often is a marker in view read, with whole cells and with cell centres, and how far off is the pose measured? | The `visibility` grid, with 12 headings turned by a random offset at each point. Every frame is read both ways |
| `guard_view` | What does the guard see ahead, and which zone does it apply? | Gazebo places the robot with the box ahead and then close to a post. One scan from above, with the corridor and the room to the nearest return in it, in the colour of the zone the guard reports |
| `stopping` | How much room does the lidar guard leave, and how much does the robot need when something appears in its way? | Through the guard, at four speeds: drives at the back wall from six angles, past or into a post at four offsets, and at the box's face and corner. Then across open floor while Gazebo puts a box ahead of it, at eight distances. Approaches three times each, appearances five |
| `localization` | How far from the truth does each marker's fix land, and does the localizer's covariance cover it? | The `visibility` grid, with 12 headings turned by offsets of its own. Every frame is read with four corner windows, and the localizer's code turns each marker read into a fix |
| `fusion` | How far from the truth does each pose estimate drift along a route, with and without markers, with the gyro calibrated or not, and with the fixes weighed in different ways? | Three laps of a 5 x 3.2 m rectangle, steered by the true pose, once per simulation with its own drift seed, 30 times. Extra filters run next to the robot's own: a pair on the uncalibrated gyro, and global ones with a 5-sigma gate, with the fixes' covariance times 4 and 16, and with at most two fixes a second |

## Running

From the repository root, with the workspace built and sourced (and, under WSL2, the GPU
variables from the main README set):

```bash
experiments/sim.sh -- python3 experiments/visibility.py
python3 experiments/visibility_report.py
```

```bash
experiments/sim.sh wheel_collision:=sphere -- python3 experiments/turning.py --label sphere
experiments/sim.sh wheel_collision:=cylinder -- python3 experiments/turning.py --label cylinder
python3 experiments/turning_report.py
```

```bash
experiments/sim.sh -- python3 experiments/camera_views.py
experiments/sim.sh -- python3 experiments/camera_views.py --stock
```

```bash
experiments/sim.sh -- python3 experiments/perception.py
python3 experiments/perception_report.py
```

```bash
experiments/sim.sh guard:=true -- python3 experiments/guard_view.py
experiments/sim.sh guard:=true -- python3 experiments/stopping.py
python3 experiments/stopping_report.py
```

```bash
experiments/sim.sh -- python3 experiments/localization.py
python3 experiments/localization_report.py
```

```bash
for run in $(seq 1 30); do
  experiments/sim.sh detector:=true drift_seed:=$run -- python3 experiments/fusion.py --run $run
done
python3 experiments/fusion_report.py
```

`sim.sh` starts a headless simulation, waits until Gazebo publishes the robot's true pose, and
runs the command. Whether the command finishes, fails or is interrupted, the simulation is shut
down afterwards. Each run uses its own Gazebo partition, out of reach of a server left over from an
earlier run. Launch arguments before `--` go to the launch.

## Layout

```
sim.sh                 runs one experiment against a fresh simulation
sim_tools.py           placing the robot and an obstacle through Gazebo, message conversions (ROS)
world.py               the world and the robot, read from the files that define them (no ROS)
camera_model.py        which markers the camera has in view, from the true pose (no ROS)
plot_style.py          one look for every figure
visibility.py          which markers the robot reads from each spot (report: visibility_report.py)
turning.py             the odometry against the truth while turning (report: turning_report.py)
camera_views.py        the camera's view from two poses, read and unread markers marked
perception.py          read rate and pose error per marker in view (report: perception_report.py)
guard_view.py          one scan and the room the guard measures ahead, from above, in two poses
stopping.py            the guard's approaches and stops, on the true pose (report: stopping_report.py)
localization.py        each marker's fix against the true pose (report: localization_report.py)
fusion.py              every pose estimate along a route against the truth (report: fusion_report.py)
results/<experiment>/  the raw numbers (CSV), summary.md and the figures
```

## Decisions

- The raw results are versioned and the figures and tables are regenerated from them. Any number
  in a summary can be traced back to the CSV it came from.
- Where the path does not matter, Gazebo's own service places the robot pose by pose. That covers
  a grid evenly and keeps every sample independent. No node on the robot can do this: it is the
  experiment that talks to Gazebo.
- The world's settings are read from the files the launch uses (`build_world.py`, the generated
  configuration and the URDF). The experiments keep no copies of them.
- Timing follows the simulation's clock, read from message stamps. A slower machine takes longer but
  measures the same thing.
- A marker counts as in view when the true pose puts it inside the frame and nothing hides it. A
  marker the detector never even finds as a candidate still counts as missed.

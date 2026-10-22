# Experiments

Each experiment runs the simulation with no windows, takes the robot through a fixed scenario,
compares what the robot measures with Gazebo's truth, and writes its numbers to
`results/<experiment>/`. A report script turns those numbers into tables and figures. Reports
run without ROS, on any machine with NumPy, Matplotlib and PyYAML.

| Experiment | Question | Scenario |
| --- | --- | --- |
| `visibility` | Which markers does the robot read from each spot in the room? | Gazebo places the robot on a 0.5 m grid over the free floor, facing 12 headings, and the detector runs on one frame per pose |
| `turning` | How far does the wheel odometry stray from the truth, per wheel collision shape? | Two turns in place at three rates and in both directions, and 2 m straight at two speeds, three times each |
| `camera_views` | Which markers in the camera's view does the detector read? | Gazebo places the robot at its start pose and turned toward the box. The markers in view come from projecting the world with the camera's true pose. `--stock` reads whole cells, OpenCV's default |
| `perception` | How often is a marker in view read, with whole cells and with cell centres, and how far off is the pose measured? | The `visibility` grid, with 12 headings turned by a random offset at each point. Every frame is read both ways |

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

`sim.sh` starts a headless simulation, waits until Gazebo publishes the robot's true pose, and
runs the command. Whether the command finishes, fails or is interrupted, the simulation is shut
down afterwards. Each run uses its own Gazebo partition, out of reach of a server left over from an
earlier run. Launch arguments before `--` go to the launch.

## Layout

```
sim.sh                 runs one experiment against a fresh simulation
sim_tools.py           placing the robot through Gazebo, and message conversions (needs ROS)
world.py               the world and the robot, read from the files that define them (no ROS)
camera_model.py        which markers the camera has in view, from the true pose (no ROS)
plot_style.py          one look for every figure
visibility.py          which markers the robot reads from each spot (report: visibility_report.py)
turning.py             the odometry against the truth while turning (report: turning_report.py)
camera_views.py        the camera's view from two poses, read and unread markers marked
perception.py          read rate and pose error per marker in view (report: perception_report.py)
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

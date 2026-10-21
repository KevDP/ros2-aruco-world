#!/usr/bin/env bash
# Run one experiment against a simulation with no windows, then shut the simulation down.
#
# Usage, from the repository root, with the workspace built and sourced:
#   experiments/sim.sh [launch arguments...] -- <command...>
# Example:
#   experiments/sim.sh wheel_collision:=cylinder -- python3 experiments/turning.py --label cylinder
#
# The launch runs without Gazebo's window, RViz, the detector or the guard; the experiment brings
# what it needs. Launch arguments given here come after those and override them.
set -euo pipefail

launch_args=()
while [[ $# -gt 0 && "$1" != "--" ]]; do
  launch_args+=("$1")
  shift
done
if [[ "${1:-}" != "--" || $# -lt 2 ]]; then
  echo "usage: $0 [launch arguments...] -- <command...>" >&2
  exit 2
fi
shift

log=$(mktemp -t sim_launch.XXXXXX.log)
# Each run talks to Gazebo on its own partition, so a server left over from another run can never
# answer this one's requests.
export GZ_PARTITION="${GZ_PARTITION:-experiment_$$}"
# The simulation runs in its own process group, so one signal reaches Gazebo and every node.
# Signalling only the launch leaves Gazebo running: it is started through a shell that does not
# pass signals on.
set -m
ros2 launch aruco_world world.launch.py gz_gui:=false rviz:=false detector:=false guard:=false \
  "${launch_args[@]}" > "$log" 2>&1 &
sim=$!
set +m

stop() {
  kill -INT -- "-$sim" 2> /dev/null || true
  wait "$sim" 2> /dev/null || true
  # Gazebo's server sometimes outlives the signal; give the group a few seconds, then end it.
  for _ in $(seq 1 10); do
    pgrep -g "$sim" > /dev/null || return 0
    sleep 1
  done
  echo "the simulation did not stop on its own; killing what is left of it" >&2
  kill -KILL -- "-$sim" 2> /dev/null || true
}
trap stop EXIT

# With its type given, echo waits for the topic to appear instead of failing at once.
if ! timeout 60 ros2 topic echo --once /ground_truth nav_msgs/msg/Odometry > /dev/null; then
  echo "the simulation did not start in 60 s; its log: $log" >&2
  exit 1
fi
echo "simulation up (log: $log)"
"$@"

#!/usr/bin/env bash
# Run one experiment against a simulation with no windows, then shut the simulation down.
#
# Usage, from the repository root, with the workspace built and sourced:
#   experiments/sim.sh [launch arguments...] -- <command...>
# Example:
#   experiments/sim.sh wheel_collision:=cylinder -- python3 experiments/turning.py --label cylinder
#
# The launch runs without Gazebo's window, RViz, the detector or the guard. Each experiment starts
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
# One Gazebo partition per run. A server left over from an earlier run never sees this one's
# requests.
export GZ_PARTITION="${GZ_PARTITION:-experiment_$$}"
# The simulation gets its own process group, where one signal reaches Gazebo and every node.
# Signalling only the launch would leave Gazebo running, because launch starts it through a shell
# that does not pass signals on.
set -m
ros2 launch aruco_world world.launch.py gz_gui:=false rviz:=false detector:=false guard:=false \
  "${launch_args[@]}" > "$log" 2>&1 &
sim=$!
set +m

stop() {
  kill -INT -- "-$sim" 2> /dev/null || true
  wait "$sim" 2> /dev/null || true
  # Gazebo's server sometimes outlives the signal. The group gets ten seconds before it is killed.
  for _ in $(seq 1 10); do
    pgrep -g "$sim" > /dev/null || return 0
    sleep 1
  done
  echo "the simulation did not stop on its own; killing what is left of it" >&2
  kill -KILL -- "-$sim" 2> /dev/null || true
}
trap stop EXIT

# Given the message type, echo waits until the topic appears.
if ! timeout 60 ros2 topic echo --once /ground_truth nav_msgs/msg/Odometry > /dev/null; then
  echo "the simulation did not start in 60 s; its log: $log" >&2
  exit 1
fi
echo "simulation up (log: $log)"
"$@"

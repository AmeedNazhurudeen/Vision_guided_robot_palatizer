# Source this in EVERY terminal:  source <this folder>/env.sh
# ROS 2 Humble + this workspace (franka/ and robot/, built into ./install), ROS kept on this computer, and the shell moved to the
# repository root (the commands use paths relative to it). ROS_LOCALHOST_ONLY / ROS_DOMAIN_ID must be the same in every terminal.
source /opt/ros/humble/setup.bash
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Optional: a franka_ros2 workspace that was built somewhere else. export FRANKA_WS=<its folder> BEFORE sourcing this file.
if [ -n "${FRANKA_WS:-}" ]; then
  if [ -f "$FRANKA_WS/install/setup.bash" ]; then
    source "$FRANKA_WS/install/setup.bash"
  else
    echo "WARNING: no built workspace at FRANKA_WS=$FRANKA_WS"
  fi
fi
[ -f "$ROOT/install/setup.bash" ] && source "$ROOT/install/setup.bash" || echo "WARNING: not built yet: run ./build.sh"
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=77
cd "$ROOT"
echo "ready: $(pwd)  ROS_DOMAIN_ID=$ROS_DOMAIN_ID  fr3_pose_pick -> $(ros2 pkg prefix fr3_pose_pick 2>/dev/null || echo 'not built')"

#!/usr/bin/env bash
# Build the ROS 2 workspace: the Franka packages in franka/ and our package in robot/.
# --symlink-install: the installed scripts are links into robot/, so a change to a script or the config needs no rebuild,
# and the scripts find perception/ and the config through them.
#   ./build.sh                                   everything (libfranka is compiled from source: the first build takes a while)
#   ./build.sh --packages-up-to fr3_pose_pick    only what the application needs
set -e
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
# A franka_ros2 workspace built elsewhere can be reused: export FRANKA_WS=<its folder>, then only robot/ is built.
if [ -n "${FRANKA_WS:-}" ] && [ -f "$FRANKA_WS/install/setup.bash" ]; then
  source "$FRANKA_WS/install/setup.bash"
  BASE_PATHS=(robot)
else
  BASE_PATHS=(franka robot)
fi
# BUILD_TESTS=OFF: libfranka's own C++ tests need googletest/gmock, which is not needed to run the robot.
colcon build --base-paths "${BASE_PATHS[@]}" --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF "$@"

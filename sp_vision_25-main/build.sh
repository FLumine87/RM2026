#!/bin/bash

source /opt/ros/humble/setup.bash
colcon build --packages-select auto_aim_interfaces --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
cmake -B build
cmake --build build --target auto_aim_debug_mpc_rosdebug -j
make -C build/ -j`nproc`
make -C build/ -j4
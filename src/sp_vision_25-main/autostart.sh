#!/bin/bash

sudo chmod 777 /dev/ttyACM0
source install/setup.bash
./build/auto_aim_debug_mpc_ros2 configs/standard3.yaml 
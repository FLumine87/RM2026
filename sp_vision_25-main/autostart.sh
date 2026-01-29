sleep 5
cd ~/Desktop/sp_vision_25/
screen \
    -L \
    -Logfile logs/$(date "+%Y-%m-%d_%H-%M-%S").screenlog \
    -d \
    -m \
    bash -c "./watchdog.sh"

# ./build/auto_aim_debug_mpc_ros2 configs/standard3.yaml 
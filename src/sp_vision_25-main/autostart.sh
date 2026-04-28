#!/bin/bash

sleep 5
cd "$(dirname "$0")"

# 使用 xterm 打开终端窗口运行 watchdog
# -hold 保持终端窗口不关闭
# -e 指定要执行的命令
xterm -hold -e "bash watchdog.sh"

# sed -i 's/\r$//' /home/a/sp_vision_25-main_2/sp_vision_25-main/autostart.sh
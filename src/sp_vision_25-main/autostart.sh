#!/bin/bash

sleep 5
cd ~/sp_vision_25-main/

# 使用 xterm 打开终端窗口运行 watchdog
# -hold 保持终端窗口不关闭
# -e 指定要执行的命令
xterm -hold -e "bash watchdog.sh"

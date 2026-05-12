#!/bin/bash

sleep 3

# 终端1：watchdog
WATCHDOG_COMMANDS="
cd \"$(dirname "$0")\"
./watchdog.sh
bash
"

# 启动终端
gnome-terminal --window --title="Watchdog" -- bash -c "$WATCHDOG_COMMANDS"

echo "已启动 Watchdog 终端"

# sed -i 's/\r$//' ./autostart.sh
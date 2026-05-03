# UDP Plotter 可视化工具

用于实时可视化 RM2025 Auto Aim MPC 调试数据的 Python 脚本。

## 功能特性

- 实时接收并显示 MPC 调试数据
- 多窗口同时显示：
  - Yaw/Pitch 角度对比（当前值、目标值、规划值）
  - Yaw/Pitch 速度和加速度
  - 目标距离和速度
  - 开火状态和目标角速度

## 依赖安装

```bash
# 安装 matplotlib 和其他依赖
pip install matplotlib
```

## 使用方法

### 方法一：直接运行（推荐）

1. **终端1 - 运行主程序**：
```bash
cd build
./auto_aim_debug_mpc configs/sentry.yaml
```

2. **终端2 - 运行可视化脚本**：
```bash
cd tools/python_scripts
python3 udp_plotter.py
```

### 方法二：使用参数运行

```bash
python3 udp_plotter.py --ip 127.0.0.1 --port 9870 --maxlen 500
```

参数说明：
- `--ip`: UDP 绑定地址（默认：127.0.0.1）
- `--port`: UDP 端口（默认：9870）
- `--maxlen`: 最大数据点数（默认：500）

## 数据格式

脚本接收的 JSON 数据包含以下字段：

| 字段 | 说明 | 单位 |
|------|------|------|
| t | 时间戳 | s |
| gimbal_yaw | 云台当前 yaw 角度 | rad |
| gimbal_yaw_vel | 云台 yaw 角速度 | rad/s |
| gimbal_pitch | 云台当前 pitch 角度 | rad |
| gimbal_pitch_vel | 云台 pitch 角速度 | rad/s |
| target_yaw | 目标 yaw 角度 | rad |
| target_pitch | 目标 pitch 角度 | rad |
| plan_yaw | MPC 规划 yaw 角度 | rad |
| plan_yaw_vel | MPC 规划 yaw 角速度 | rad/s |
| plan_yaw_acc | MPC 规划 yaw 角加速度 | rad/s² |
| plan_pitch | MPC 规划 pitch 角度 | rad |
| plan_pitch_vel | MPC 规划 pitch 角速度 | rad/s |
| plan_pitch_acc | MPC 规划 pitch 角加速度 | rad/s² |
| fire | 是否开火 | 0/1 |
| fired | 是否已发射 | 0/1 |
| target_z | 目标距离 | m |
| target_vz | 目标距离方向速度 | m/s |
| w | 目标角速度 | rad/s |

## 注意事项

1. **防火墙设置**：确保 UDP 端口 9870 已开放
```bash
sudo ufw allow 9870/udp
```

2. **网络配置**：如果需要远程查看，修改 `--ip` 参数为对应地址

3. **性能优化**：如果数据量过大，减少 `--maxlen` 参数值

## 文件结构

```
tools/python_scripts/
├── udp_plotter.py    # 主可视化脚本
└── README.md         # 使用说明
```

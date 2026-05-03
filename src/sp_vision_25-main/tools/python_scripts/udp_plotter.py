#!/usr/bin/env python3
"""
UDP Plotter for RM2025 Auto Aim Debug MPC
实时可视化 MPC 调试数据

使用方法:
1. 运行主程序: ./auto_aim_debug_mpc configs/sentry.yaml
2. 运行此脚本: python3 udp_plotter.py

数据格式:
- t: 时间戳 (s)
- gimbal_yaw/pitch/vel: 云台状态
- target_yaw/pitch: 目标角度
- plan_yaw/pitch/vel/acc: MPC 规划结果
- fire/fired: 开火状态
- target_z/vz: 目标距离和速度
- w: 目标角速度
"""

import socket
import json
import threading
import numpy as np
import matplotlib
matplotlib.use('TkAgg')  # 使用 TkAgg 后端
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import argparse

# 全局数据缓冲区
data_buffer = {
    't': [],
    'gimbal_yaw': [],
    'gimbal_yaw_vel': [],
    'gimbal_pitch': [],
    'gimbal_pitch_vel': [],
    'target_yaw': [],
    'target_pitch': [],
    'plan_yaw': [],
    'plan_yaw_vel': [],
    'plan_yaw_acc': [],
    'plan_pitch': [],
    'plan_pitch_vel': [],
    'plan_pitch_acc': [],
    'fire': [],
    'fired': [],
    'target_z': [],
    'target_vz': [],
    'w': []
}

stop_event = threading.Event()
data_mutex = threading.Lock()
new_data_event = threading.Event()

def udp_receiver(ip, port):
    """UDP 数据接收线程"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((ip, port))
    sock.settimeout(1.0)
    
    while not stop_event.is_set():
        try:
            data, addr = sock.recvfrom(4096)
            json_data = json.loads(data.decode())
            
            with data_mutex:
                # 更新数据缓冲区，缺失字段用 NaN 填充
                data_buffer['t'].append(json_data.get('t', np.nan))
                data_buffer['gimbal_yaw'].append(json_data.get('gimbal_yaw', np.nan))
                data_buffer['gimbal_yaw_vel'].append(json_data.get('gimbal_yaw_vel', np.nan))
                data_buffer['gimbal_pitch'].append(json_data.get('gimbal_pitch', np.nan))
                data_buffer['gimbal_pitch_vel'].append(json_data.get('gimbal_pitch_vel', np.nan))
                data_buffer['target_yaw'].append(json_data.get('target_yaw', np.nan))
                data_buffer['target_pitch'].append(json_data.get('target_pitch', np.nan))
                data_buffer['plan_yaw'].append(json_data.get('plan_yaw', np.nan))
                data_buffer['plan_yaw_vel'].append(json_data.get('plan_yaw_vel', np.nan))
                data_buffer['plan_yaw_acc'].append(json_data.get('plan_yaw_acc', np.nan))
                data_buffer['plan_pitch'].append(json_data.get('plan_pitch', np.nan))
                data_buffer['plan_pitch_vel'].append(json_data.get('plan_pitch_vel', np.nan))
                data_buffer['plan_pitch_acc'].append(json_data.get('plan_pitch_acc', np.nan))
                data_buffer['fire'].append(json_data.get('fire', np.nan))
                data_buffer['fired'].append(json_data.get('fired', np.nan))
                data_buffer['target_z'].append(json_data.get('target_z', np.nan))
                data_buffer['target_vz'].append(json_data.get('target_vz', np.nan))
                data_buffer['w'].append(json_data.get('w', np.nan))
            
            # 限制缓冲区大小
            max_len = 500
            with data_mutex:
                for key in data_buffer:
                    if len(data_buffer[key]) > max_len:
                        data_buffer[key] = data_buffer[key][-max_len:]
            
            new_data_event.set()
        except socket.timeout:
            continue
        except json.JSONDecodeError:
            print("JSON 解析错误")
        except Exception as e:
            print(f"接收数据错误: {e}")
    
    sock.close()

def main():
    parser = argparse.ArgumentParser(description='UDP Plotter for RM2025 Auto Aim')
    parser.add_argument('--ip', default='127.0.0.1', help='UDP 绑定地址')
    parser.add_argument('--port', type=int, default=9870, help='UDP 端口')
    parser.add_argument('--maxlen', type=int, default=500, help='最大数据点数')
    args = parser.parse_args()
    
    # 启动接收线程
    receiver_thread = threading.Thread(target=udp_receiver, args=(args.ip, args.port))
    receiver_thread.daemon = True
    receiver_thread.start()
    
    print(f"UDP Plotter 启动，监听 {args.ip}:{args.port}")
    print("等待接收数据...")
    
    # 等待首次数据
    success = new_data_event.wait(timeout=10)
    if not success:
        print("超时未收到数据，检查主程序是否正常运行")
        stop_event.set()
        receiver_thread.join()
        return
    
    # 初始化图形
    fig = plt.figure(figsize=(14, 10))
    fig.suptitle('RM2025 Auto Aim MPC Debug Data', fontsize=14)
    
    # 子图布局: 3行2列
    gs = fig.add_gridspec(3, 2)
    
    # 子图1: Yaw 角度
    ax1 = fig.add_subplot(gs[0, 0])
    line1_1, = ax1.plot([], [], 'b-', label='gimbal_yaw', linewidth=1.5)
    line1_2, = ax1.plot([], [], 'r--', label='target_yaw', linewidth=1.5)
    line1_3, = ax1.plot([], [], 'g-.', label='plan_yaw', linewidth=1.5)
    ax1.set_ylabel('Yaw (rad)')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # 子图2: Pitch 角度
    ax2 = fig.add_subplot(gs[0, 1])
    line2_1, = ax2.plot([], [], 'b-', label='gimbal_pitch', linewidth=1.5)
    line2_2, = ax2.plot([], [], 'r--', label='target_pitch', linewidth=1.5)
    line2_3, = ax2.plot([], [], 'g-.', label='plan_pitch', linewidth=1.5)
    ax2.set_ylabel('Pitch (rad)')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # 子图3: Yaw 速度和加速度
    ax3 = fig.add_subplot(gs[1, 0])
    line3_1, = ax3.plot([], [], 'b-', label='gimbal_yaw_vel', linewidth=1.5)
    line3_2, = ax3.plot([], [], 'r--', label='plan_yaw_vel', linewidth=1.5)
    line3_3, = ax3.plot([], [], 'g-.', label='plan_yaw_acc', linewidth=1.5)
    ax3.set_ylabel('Yaw Vel/Acc')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    # 子图4: Pitch 速度和加速度
    ax4 = fig.add_subplot(gs[1, 1])
    line4_1, = ax4.plot([], [], 'b-', label='gimbal_pitch_vel', linewidth=1.5)
    line4_2, = ax4.plot([], [], 'r--', label='plan_pitch_vel', linewidth=1.5)
    line4_3, = ax4.plot([], [], 'g-.', label='plan_pitch_acc', linewidth=1.5)
    ax4.set_ylabel('Pitch Vel/Acc')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    
    # 子图5: 目标信息
    ax5 = fig.add_subplot(gs[2, 0])
    line5_1, = ax5.plot([], [], 'b-', label='target_z', linewidth=1.5)
    line5_2, = ax5.plot([], [], 'r--', label='target_vz', linewidth=1.5)
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Target Z (m)')
    ax5.legend()
    ax5.grid(True, alpha=0.3)
    
    # 子图6: 开火状态和角速度
    ax6 = fig.add_subplot(gs[2, 1])
    line6_1, = ax6.plot([], [], 'r-', label='fire', linewidth=1.5)
    line6_2, = ax6.plot([], [], 'g--', label='w', linewidth=1.5)
    ax6.set_xlabel('Time (s)')
    ax6.set_ylabel('Fire / Angular Vel')
    ax6.legend()
    ax6.grid(True, alpha=0.3)
    
    def update(frame):
        # 复制数据以避免线程冲突
        with data_mutex:
            t = np.array(data_buffer['t']).copy()
            gimbal_yaw = np.array(data_buffer['gimbal_yaw']).copy()
            gimbal_yaw_vel = np.array(data_buffer['gimbal_yaw_vel']).copy()
            gimbal_pitch = np.array(data_buffer['gimbal_pitch']).copy()
            gimbal_pitch_vel = np.array(data_buffer['gimbal_pitch_vel']).copy()
            target_yaw = np.array(data_buffer['target_yaw']).copy()
            target_pitch = np.array(data_buffer['target_pitch']).copy()
            plan_yaw = np.array(data_buffer['plan_yaw']).copy()
            plan_yaw_vel = np.array(data_buffer['plan_yaw_vel']).copy()
            plan_yaw_acc = np.array(data_buffer['plan_yaw_acc']).copy()
            plan_pitch = np.array(data_buffer['plan_pitch']).copy()
            plan_pitch_vel = np.array(data_buffer['plan_pitch_vel']).copy()
            plan_pitch_acc = np.array(data_buffer['plan_pitch_acc']).copy()
            fire = np.array(data_buffer['fire']).copy()
            target_z = np.array(data_buffer['target_z']).copy()
            target_vz = np.array(data_buffer['target_vz']).copy()
            w = np.array(data_buffer['w']).copy()
        
        # 过滤有效数据（去除 NaN）
        mask = ~np.isnan(t)
        t = t[mask]
        
        # 更新 Yaw 角度
        line1_1.set_data(t, gimbal_yaw[mask])
        line1_2.set_data(t, target_yaw[mask])
        line1_3.set_data(t, plan_yaw[mask])
        
        # 更新 Pitch 角度
        line2_1.set_data(t, gimbal_pitch[mask])
        line2_2.set_data(t, target_pitch[mask])
        line2_3.set_data(t, plan_pitch[mask])
        
        # 更新 Yaw 速度和加速度
        line3_1.set_data(t, gimbal_yaw_vel[mask])
        line3_2.set_data(t, plan_yaw_vel[mask])
        line3_3.set_data(t, plan_yaw_acc[mask])
        
        # 更新 Pitch 速度和加速度
        line4_1.set_data(t, gimbal_pitch_vel[mask])
        line4_2.set_data(t, plan_pitch_vel[mask])
        line4_3.set_data(t, plan_pitch_acc[mask])
        
        # 更新目标信息
        line5_1.set_data(t, target_z[mask])
        line5_2.set_data(t, target_vz[mask])
        
        # 更新开火状态
        line6_1.set_data(t, fire[mask])
        line6_2.set_data(t, w[mask])
        
        # 自动调整坐标轴（仅在有数据时）
        if len(t) > 0:
            for ax in [ax1, ax2, ax3, ax4, ax5, ax6]:
                ax.relim()
                ax.autoscale_view(scalex=True, scaley=True)
        
        new_data_event.clear()
        return line1_1, line1_2, line1_3, line2_1, line2_2, line2_3, \
               line3_1, line3_2, line3_3, line4_1, line4_2, line4_3, \
               line5_1, line5_2, line6_1, line6_2
    
    # 启动动画
    ani = FuncAnimation(fig, update, interval=50, blit=True)
    
    plt.tight_layout()
    plt.show()
    
    stop_event.set()
    receiver_thread.join()

if __name__ == '__main__':
    main()

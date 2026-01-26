# standard_mpc.cpp 算法运行逻辑分析

本文档分析了 `standard_mpc.cpp` 文件的算法运行逻辑，包括其执行流程和每一步涉及的代码文件。

## 代码运行调用关系结构

### 主程序与模块调用关系

```
sp_vision_25-main/
├── src/                      # 源代码目录
│   └── standard_mpc.cpp      # 主程序文件，实现了自瞄和打符的主要逻辑
├── io/                       # 输入输出模块
│   ├── camera.hpp            # 相机基类定义，提供相机图像读取接口
│   └── gimbal/               # 云台模块
│       ├── gimbal.hpp        # 云台类定义，实现与云台的通信
│       └── gimbal.cpp        # 云台类实现
├── tasks/                    # 任务模块
│   ├── auto_aim/             # 自瞄模块
│   │   ├── yolo.hpp          # YOLO 目标检测定义
│   │   ├── yolo.cpp          # YOLO 目标检测实现
│   │   ├── armor.hpp         # 装甲板类定义
│   │   ├── solver.hpp        # 目标位置解算器定义
│   │   ├── solver.cpp        # 目标位置解算器实现
│   │   ├── tracker.hpp       # 目标跟踪类定义
│   │   ├── tracker.cpp       # 目标跟踪类实现
│   │   ├── target.hpp        # 目标类定义
│   │   └── planner/          # 规划器模块
│   │       ├── planner.hpp   # 规划器类定义，使用 MPC 算法
│   │       ├── planner.cpp   # 规划器类实现
│   │       └── tinympc/      # 小型 MPC 实现
│   │           └── tiny_api.hpp # MPC 算法接口
│   └── auto_buff/            # 打符模块
│       ├── buff_detector.hpp # 能量机关检测类定义
│       ├── buff_detector.cpp # 能量机关检测类实现
│       ├── buff_solver.hpp   # 能量机关解算器定义
│       ├── buff_solver.cpp   # 能量机关解算器实现
│       ├── buff_aimer.hpp    # 能量机关瞄准类定义
│       ├── buff_aimer.cpp    # 能量机关瞄准类实现
│       ├── buff_target.hpp   # 能量机关目标类定义
│       ├── buff_type.hpp     # 能量机关类型定义
│       └── yolo11_buff.hpp   # YOLOv11 能量机关检测定义
├── tools/                    # 工具模块
│   ├── thread_safe_queue.hpp # 线程安全队列，用于线程间数据传输
│   ├── logger.hpp            # 日志工具，用于记录运行信息
│   ├── exiter.hpp            # 程序退出管理工具
│   ├── plotter.hpp           # 数据可视化工具
│   ├── recorder.hpp          # 数据记录工具
│   ├── img_tools.hpp         # 图像处理工具
│   └── math_tools.hpp        # 数学工具
└── configs/                  # 配置文件目录
    └── standard3.yaml        # 标准配置文件
```

### 自瞄模式调用流程

```
standard_mpc.cpp:主循环
├── io/Camera::read           # 读取相机图像
├── auto_aim::YOLO::detect    # 检测装甲板
│   ├── tasks/auto_aim/yolo.hpp
│   └── tasks/auto_aim/armor.hpp
├── auto_aim::Tracker::track  # 跟踪目标
│   ├── tasks/auto_aim/tracker.hpp
│   ├── tasks/auto_aim/solver.hpp
│   └── tasks/auto_aim/target.hpp
├── tools/ThreadSafeQueue::push # 推送目标到队列
│   └── tools/thread_safe_queue.hpp
├── 规划线程
│   ├── auto_aim::Planner::plan # 规划轨迹
│   │   ├── tasks/auto_aim/planner/planner.hpp
│   │   └── tasks/auto_aim/planner/tinympc/tiny_api.hpp
│   └── io/Gimbal::send        # 发送控制命令
│       └── io/gimbal/gimbal.hpp
```

### 打符模式调用流程

```
standard_mpc.cpp:主循环
├── io/Camera::read           # 读取相机图像
├── auto_buff::Buff_Detector::detect # 检测能量机关
│   ├── tasks/auto_buff/buff_detector.hpp
│   └── tasks/auto_buff/yolo11_buff.hpp
├── auto_buff::Solver::solve  # 解算能量机关位置
│   ├── tasks/auto_buff/buff_solver.hpp
│   └── tools/math_tools.hpp
├── auto_buff::SmallTarget::get_target/auto_buff::BigTarget::get_target # 获取目标
│   └── tasks/auto_buff/buff_target.hpp
├── auto_buff::Aimer::mpc_aim # 瞄准能量机关
│   ├── tasks/auto_buff/buff_aimer.hpp
│   └── tasks/auto_aim/planner/planner.hpp
└── io/Gimbal::send           # 发送控制命令
    └── io/gimbal/gimbal.hpp
```

## 算法运行逻辑

### 1. 初始化阶段

**代码位置**：`standard_mpc.cpp:31-60`

**执行流程**：
1. **解析命令行参数**：获取配置文件路径
   - `standard_mpc.cpp:33-38`
2. **初始化工具类**：
   - `tools/exiter.hpp`：程序退出管理
   - `tools/plotter.hpp`：数据可视化
   - `tools/recorder.hpp`：数据记录
   - `standard_mpc.cpp:40-42`
3. **初始化输入输出模块**：
   - `io/Gimbal`：云台控制
     - `io/gimbal/gimbal.hpp`
   - `io/Camera`：相机图像获取
     - `io/camera.hpp`
   - `standard_mpc.cpp:44-45`
4. **初始化自瞄模块**：
   - `auto_aim::YOLO`：目标检测
     - `tasks/auto_aim/yolo.hpp`
   - `auto_aim::Solver`：目标位置解算
     - `tasks/auto_aim/solver.hpp`
   - `auto_aim::Tracker`：目标跟踪
     - `tasks/auto_aim/tracker.hpp`
   - `auto_aim::Planner`：云台运动规划
     - `tasks/auto_aim/planner/planner.hpp`
   - `standard_mpc.cpp:47-51`
5. **初始化打符模块**：
   - `auto_buff::Buff_Detector`：能量机关检测
     - `tasks/auto_buff/buff_detector.hpp`
   - `auto_buff::Solver`：能量机关位置解算
     - `tasks/auto_buff/buff_solver.hpp`
   - `auto_buff::SmallTarget`、`auto_buff::BigTarget`：能量机关目标
     - `tasks/auto_buff/buff_target.hpp`
   - `auto_buff::Aimer`：能量机关瞄准
     - `tasks/auto_buff/buff_aimer.hpp`
   - `standard_mpc.cpp:55-59`
6. **初始化数据结构**：
   - 目标队列：`tools/ThreadSafeQueue`
   - `standard_mpc.cpp:52-53`

### 2. 规划线程启动

**代码位置**：`standard_mpc.cpp:70-88`

**执行流程**：
1. **创建规划线程**：
   - `standard_mpc.cpp:70`
2. **规划线程主循环**：
   - 检查目标队列是否非空且当前模式为自瞄模式
     - `standard_mpc.cpp:75`
   - 获取目标和云台状态
     - `standard_mpc.cpp:76-77`
   - 使用规划器规划云台运动轨迹
     - `auto_aim::Planner::plan`
     - `tasks/auto_aim/planner/planner.hpp`
     - `standard_mpc.cpp:78`
   - 发送控制命令到云台
     - `io/Gimbal::send`
     - `io/gimbal/gimbal.hpp`
     - `standard_mpc.cpp:80-82`
   - 线程休眠
     - `standard_mpc.cpp:84`

### 3. 主循环

**代码位置**：`standard_mpc.cpp:90-138`

**执行流程**：
1. **检查退出信号**：
   - `tools/Exiter::exit`
   - `standard_mpc.cpp:90`
2. **获取当前模式**：
   - `io/Gimbal::mode`
   - `standard_mpc.cpp:91`
3. **模式切换日志**：
   - `tools/logger`
   - `standard_mpc.cpp:93-96`
4. **读取相机图像**：
   - `io/Camera::read`
   - `standard_mpc.cpp:98`
5. **获取云台状态**：
   - `io/Gimbal::q`：获取云台姿态
   - `io/Gimbal::state`：获取云台状态
   - `standard_mpc.cpp:99-100`
6. **记录数据**：
   - `tools/Recorder::record`
   - `standard_mpc.cpp:101`
7. **设置解算器姿态**：
   - `auto_aim::Solver::set_R_gimbal2world`
   - `standard_mpc.cpp:102`

### 4. 自瞄模式执行

**代码位置**：`standard_mpc.cpp:105-112`

**执行流程**：
1. **检测装甲板**：
   - `auto_aim::YOLO::detect`
   - `tasks/auto_aim/yolo.hpp`
   - `standard_mpc.cpp:106`
2. **跟踪目标**：
   - `auto_aim::Tracker::track`
   - `tasks/auto_aim/tracker.hpp`
   - `standard_mpc.cpp:107`
3. **推送目标到队列**：
   - `tools/ThreadSafeQueue::push`
   - `standard_mpc.cpp:109-111`

### 5. 打符模式执行

**代码位置**：`standard_mpc.cpp:115-136`

**执行流程**：
1. **设置解算器姿态**：
   - `auto_buff::Solver::set_R_gimbal2world`
   - `standard_mpc.cpp:116`
2. **检测能量机关**：
   - `auto_buff::Buff_Detector::detect`
   - `tasks/auto_buff/buff_detector.hpp`
   - `standard_mpc.cpp:118`
3. **解算能量机关位置**：
   - `auto_buff::Solver::solve`
   - `tasks/auto_buff/buff_solver.hpp`
   - `standard_mpc.cpp:120`
4. **根据模式获取目标**：
   - 小符模式：`auto_buff::SmallTarget::get_target`
   - 大符模式：`auto_buff::BigTarget::get_target`
   - `tasks/auto_buff/buff_target.hpp`
   - `standard_mpc.cpp:124-129`
5. **瞄准能量机关**：
   - `auto_buff::Aimer::mpc_aim`
   - `tasks/auto_buff/buff_aimer.hpp`
   - `standard_mpc.cpp:126-130`
6. **发送控制命令到云台**：
   - `io/Gimbal::send`
   - `standard_mpc.cpp:132-134`

### 6. 退出阶段

**代码位置**：`standard_mpc.cpp:140-144`

**执行流程**：
1. **设置退出标志**：
   - `standard_mpc.cpp:140`
2. **等待规划线程结束**：
   - `standard_mpc.cpp:141`
3. **发送停止命令到云台**：
   - `io/Gimbal::send`
   - `standard_mpc.cpp:142`
4. **退出程序**：
   - `standard_mpc.cpp:143`

## 详细流程分析

### 自瞄模式详细流程

1. **目标检测**：
   - **代码**：`auto_aim::YOLO::detect`
     - `tasks/auto_aim/yolo.cpp`
   - **流程**：
     - 加载 YOLO 模型（根据配置选择 yolov5、yolov8 或 yolo11）
     - 对相机图像进行预处理
     - 使用 YOLO 模型进行推理
     - 后处理检测结果，生成装甲板对象

2. **目标跟踪**：
   - **代码**：`auto_aim::Tracker::track`
     - `tasks/auto_aim/tracker.cpp`
   - **流程**：
     - 对检测到的装甲板进行筛选
     - 使用状态机管理目标跟踪状态
     - 计算目标与历史目标的匹配度
     - 更新目标状态和丢失计数
     - 选择最优目标

3. **目标解算**：
   - **代码**：`auto_aim::Solver::solve`
     - `tasks/auto_aim/solver.cpp`
   - **流程**：
     - 使用相机内参和外参
     - 根据装甲板的像素坐标计算三维位置
     - 优化目标姿态角

4. **运动规划**：
   - **代码**：`auto_aim::Planner::plan`
     - `tasks/auto_aim/planner/planner.cpp`
   - **流程**：
     - 根据目标位置和子弹速度计算瞄准点
     - 使用 MPC（模型预测控制）算法规划云台运动轨迹
     - 计算云台的角度、速度和加速度指令

5. **云台控制**：
   - **代码**：`io/Gimbal::send`
     - `io/gimbal/gimbal.cpp`
   - **流程**：
     - 将规划结果转换为云台控制命令
     - 通过串口发送控制命令到云台

### 打符模式详细流程

1. **能量机关检测**：
   - **代码**：`auto_buff::Buff_Detector::detect`
     - `tasks/auto_buff/buff_detector.cpp`
   - **流程**：
     - 使用 YOLOv11 模型检测能量机关
     - 处理检测结果，提取扇叶信息
     - 计算能量机关中心位置

2. **能量机关解算**：
   - **代码**：`auto_buff::Solver::solve`
     - `tasks/auto_buff/buff_solver.cpp`
   - **流程**：
     - 使用相机内参和外参
     - 根据能量机关的像素坐标计算三维位置

3. **能量机关瞄准**：
   - **代码**：`auto_buff::Aimer::mpc_aim`
     - `tasks/auto_buff/buff_aimer.cpp`
   - **流程**：
     - 预测能量机关的运动轨迹
     - 根据子弹速度计算提前量
     - 使用 MPC 算法规划云台运动轨迹

4. **云台控制**：
   - **代码**：`io/Gimbal::send`
     - `io/gimbal/gimbal.cpp`
   - **流程**：
     - 将瞄准结果转换为云台控制命令
     - 通过串口发送控制命令到云台

## 文件功能说明

### 主程序文件

- **standard_mpc.cpp**：主程序文件，实现了自瞄和打符的主要逻辑，包括相机图像读取、目标检测、跟踪、解算和云台控制。

### 输入输出模块

- **io/camera.hpp**：相机基类定义，提供了相机图像读取的接口。
- **io/gimbal/gimbal.hpp**：云台类定义，实现了与云台的通信，包括获取云台状态和发送控制命令。

### 自瞄模块

- **tasks/auto_aim/yolo.hpp**：YOLO 目标检测类定义，实现了基于 YOLO 模型的目标检测功能。
- **tasks/auto_aim/solver.hpp**：解算器类定义，实现了目标三维位置的解算功能。
- **tasks/auto_aim/tracker.hpp**：目标跟踪类定义，实现了目标的跟踪功能，包括状态机和目标切换逻辑。
- **tasks/auto_aim/planner/planner.hpp**：规划器类定义，实现了云台运动轨迹的规划功能，使用 MPC（模型预测控制）算法。

### 打符模块

- **tasks/auto_buff/buff_detector.hpp**：能量机关检测类定义，实现了能量机关的检测功能。
- **tasks/auto_buff/buff_solver.hpp**：能量机关解算器类定义，实现了能量机关三维位置的解算功能。
- **tasks/auto_buff/buff_aimer.hpp**：能量机关瞄准类定义，实现了能量机关的瞄准功能，包括预测和控制。

### 工具模块

- **tools/thread_safe_queue.hpp**：线程安全队列类定义，实现了多线程间的数据传输功能。
- **tools/logger.hpp**：日志工具类定义，实现了日志记录功能。
- **tools/exiter.hpp**：退出工具类定义，实现了程序的优雅退出功能。
- **tools/plotter.hpp**：绘图工具类定义，实现了数据可视化功能。
- **tools/recorder.hpp**：记录工具类定义，实现了图像和数据的记录功能。

## 配置文件说明

`standard3.yaml` 是 `standard_mpc.cpp` 使用的配置文件，包含了以下主要配置项：

- **enemy_color**：敌方颜色，可设置为 "red" 或 "blue"。
- **yolo_name**：使用的 YOLO 模型名称，如 "yolov5"。
- **classify_model**：分类模型路径。
- **yolo11_model_path**、**yolov8_model_path**、**yolov5_model_path**：不同 YOLO 模型的路径。
- **device**：运行设备，如 "CPU"。
- **min_confidence**：检测置信度阈值。
- **use_traditional**：是否使用传统方法。
- **roi**：感兴趣区域的坐标和大小。
- **use_roi**：是否使用感兴趣区域。
- **threshold**：传统方法的阈值。
- **max_angle_error**：最大角度误差。
- **min_lightbar_ratio**、**max_lightbar_ratio**：灯条比例范围。
- **min_lightbar_length**：最小灯条长度。
- **min_armor_ratio**、**max_armor_ratio**：装甲板比例范围。
- **max_side_ratio**：最大边比例。
- **max_rectangular_error**：最大矩形误差。
- **min_detect_count**：最小检测次数。
- **max_temp_lost_count**：最大临时丢失次数。
- **outpost_max_temp_lost_count**：前哨站最大临时丢失次数。
- **yaw_offset**：偏航偏移量。
- **pitch_offset**：俯仰偏移量。
- **comming_angle**：接近角度。
- **leaving_angle**：离开角度。
- **decision_speed**：决策速度。
- **high_speed_delay_time**、**low_speed_delay_time**：高速和低速延迟时间。
- **first_tolerance**：近距离射击容差。
- **second_tolerance**：远距离射击容差。
- **judge_distance**：距离判断阈值。
- **auto_fire**：是否由自瞄控制射击。
- **camera_name**：相机名称。
- **exposure_ms**：曝光时间。
- **gain**：增益。
- **vid_pid**：相机的 VID 和 PID。
- **R_gimbal2imubody**：云台到 IMU 本体的旋转矩阵。
- **camera_matrix**：相机内参矩阵。
- **distort_coeffs**：相机畸变系数。
- **R_camera2gimbal**：相机到云台的旋转矩阵。
- **t_camera2gimbal**：相机到云台的平移向量。
- **quaternion_canid**：四元数 CAN ID。
- **bullet_speed_canid**：子弹速度 CAN ID。
- **send_canid**：发送 CAN ID。
- **can_interface**：CAN 接口名称。
- **com_port**：串口端口。
- **yaw_kp**、**yaw_kd**、**pitch_kp**、**pitch_kd**：云台控制的 PID 参数。
- **fire_thresh**：开火阈值。
- **max_yaw_acc**：最大偏航加速度。
- **Q_yaw**、**R_yaw**：偏航控制器的 Q 和 R 矩阵。
- **max_pitch_acc**：最大俯仰加速度。
- **Q_pitch**、**R_pitch**：俯仰控制器的 Q 和 R 矩阵。
- **model**：能量机关模型路径。
- **fire_gap_time**：开火间隔时间。
- **predict_time**：预测时间。

## 运行说明

要运行 `standard_mpc.cpp`，需要指定配置文件路径：

```bash
./standard_mpc <config_path>
```

例如：

```bash
./standard_mpc configs/standard3.yaml
```

## 注意事项

1. 确保配置文件中的路径正确，特别是模型文件路径。
2. 确保相机和云台设备已正确连接。
3. 根据实际情况调整配置文件中的参数，以获得最佳效果。
4. 在运行前，确保已安装所有必要的依赖项，如 OpenCV、Eigen、serial 等。
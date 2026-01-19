
# RM2026 -VANGUARD

华南师范大学 RoboMaster 2026 赛季代码库

## 📁 项目架构

本项目采用**多版本并行管理**策略

### 🌿 分支结构

```
main ────────────────────────────────────────────────────────────────┐
├── develop ──────────────── 主要开发分支 (不稳定)                    │
├── release/v1.0 ─────────── 2025赛季基础版本 (永久存档)             │
└── release/v2.0 ─────────── 新火控系统版本 (永久存档)               │
```

### 🏷️ 版本历史

| 版本 | 分支 | 状态 | 主要特性 |
|------|------|------|----------|
| v1.0 | `release/v1.0` | ✅ 稳定 | 2025赛季基础代码 |
| v2.0 | `release/v2.0` | ✅ 稳定 | 新火控系统 |
| 开发版 | `develop` | 🚧 开发中 | 持续集成新功能 |


### 环境要求

- ROS 2 (Humble)
- OpenCV 4.5+
- CMake 3.16+
- C++17 兼容编译器

## 📦 项目结构

```
RM2026/
├── ros_ws/                 # ROS 工作空间
│   ├── src/
│   │   ├── rm_auto_aim/    # 自动瞄准模块
│   │   ├── rm_buff/        # 增益道具识别
│   │   ├── rm_fire_control/# 火控系统
│   │   ├── rm_gimbal_description/ # 云台描述
│   │   ├── rm_serial_driver/     # 串口驱动
│   │   ├── rm_vision/      # 视觉处理
│   │   └── ros2_hik_camera/# 海康相机驱动
│   └── package.xml
├── docs/                   # 项目文档
└── README.md
```

## 🚀 快速开始

### 获取特定版本

```bash
# 获取 v1.0 稳定版本
git checkout release/v1.0

# 获取 v2.0 稳定版本  
git checkout release/v2.0

# 获取最新开发版本（不稳定）
git checkout develop
```

## 🔧 编译与运行

### 编译项目

```bash
cd ros_ws
colcon build --symlink-install
source install/setup.bash
```
<!-- 
### 运行核心模块

```bash
# 启动视觉识别
ros2 launch rm_vision vision.launch.py

# 启动火控系统
ros2 launch rm_fire_control fire_control.launch.py

# 启动自动瞄准
ros2 launch rm_auto_aim auto_aim.launch.py
``` -->

## 📋 版本管理指南

### 对于使用者

- **生产环境**：使用 `release/*` 分支的稳定版本
- **测试新功能**：使用 `develop` 分支
- **特定版本**：通过标签直接检出，如 `git checkout v1.0`

### 对于开发者

#### 日常开发
```bash
git checkout develop
# 进行功能开发...
git add .
git commit -m "功能: 描述变更"
git push origin develop
```

#### 发布新版本
1. 确保 `develop` 分支稳定
2. 合并到 `main`: `git checkout main && git merge develop`
3. 创建发布分支: `git checkout -b release/v3.0`
4. 打上标签: `git tag v3.0`
5. 推送到远程: `git push --all origin && git push --tags`

#### 维护旧版本
```bash
# 为 v1.0 修复 bug
git checkout release/v1.0
git checkout -b hotfix/v1.0.1
# 修复问题...
git checkout release/v1.0
git merge hotfix/v1.0.1
git tag v1.0.1
```



## 📄 许可证

本项目采用 MIT 许可证。详见 [LICENSE](LICENSE) 文件。

## 👥 开发团队

- **团队名称**: vanguard 
- **竞赛**: RoboMaster 2026
- **维护者**: oyyy , gh , pyy 

---

## 💡 使用提示

- 使用 `git graph` 或 `git log --oneline --graph --all` 可视化分支结构
- 定期从 GitHub 拉取更新: `git fetch --all`
- 查看可用标签: `git tag -l`

## 待开发功能
- [ ] 增添弹道闭环控制
- [ ] pnp以降低自由度代价优化（可能要分兵种）
- [ ] 用球面坐标系优化扩展卡尔曼滤波器
- [ ] 优化视觉ui，包括不限于：弹道校正信息，全装甲板预测ui

## 开发日志
- 2025-？？-？？: ？？

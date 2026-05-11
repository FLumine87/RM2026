# 通信修改指南

## 数据流向总览

| 方向 | 数据流 |
|------|--------|
| **上行** | 下位机 → Gimbal (串口) → `state()` → `ROS2Publisher` → ROS Topic → 导航 |
| **下行** | 导航 → ROS Topic → `ROS2Publisher` (回调) → `plan_thread`组包 → `VisionToGimbal` → Gimbal (串口) → 下位机 |

---

## 一、添加/删除「下位机 → 视觉」的数据

### 1.1 修改 `GimbalToVision` 结构体
**文件**: `io/gimbal/gimbal.hpp`

```cpp
struct __attribute__((packed)) GimbalToVision
{
  // ... 现有字段 ...
  uint16_t crc16;
};
static_assert(sizeof(GimbalToVision) <= 64);  // 确保不超限
```

### 1.2 更新 `GimbalState` 结构体
**文件**: `io/gimbal/gimbal.hpp`

```cpp
struct GimbalState
{
  // ... 现有字段 ...
};
```

### 1.3 更新 `read_thread` 解析逻辑
**文件**: `io/gimbal/gimbal.cpp`

在 `read_thread()` 中，CRC校验成功后添加/删除：

```cpp
std::lock_guard<std::mutex> lock(mutex_);
// 添加/删除字段赋值
state_.xxx = rx_data_.xxx;
```

### 1.4 (可选) 发布到 ROS Topic
**文件**: `src/sentry_debug_mpc_ros.cpp`

如果需要转发给导航，在对应的发布函数中添加逻辑（例如 `publish_gimbal_feedback()` / `publish_sentry_status()`）。

---

## 二、添加/删除「视觉 → 下位机」的数据

> **重要规范**：**禁止**使用单独参数处理方法（如 `GetGimbalPackage()`, `GetFireMode()`, `SetGimbalPackage()`），也**禁止**添加多参数的 `send` 重载。所有数据必须由主程序统一组包后，调用单一的 `send(VisionToGimbal)` 方法发送。

### 2.1 修改 `VisionToGimbal` 结构体
**文件**: `io/gimbal/gimbal.hpp`

```cpp
struct __attribute__((packed)) VisionToGimbal
{
  // ... 现有字段 ...
  uint16_t crc16;
};
static_assert(sizeof(VisionToGimbal) <= 64);  // 确保不超限
```

### 2.2 更新 `send` 方法（唯一入口）
**文件**: `io/gimbal/gimbal.hpp`

`Gimbal` 类**只保留**以下单一 `send` 方法：

```cpp
void send(io::VisionToGimbal VisionToGimbal);
```

**禁止**添加任何其他 `send` 重载形式，例如：
```cpp
// ❌ 禁止：不允许多参数重载
void send(bool control, bool fire, float yaw, ...);
void send(bool control, bool fire, float yaw, ..., VisionToGimbal recv_data);
```

**禁止**添加以下单独参数处理方法：
```cpp
// ❌ 禁止：不允许单独处理参数
GimbalToVision GetGimbalPackage() const;
uint8_t GetFireMode() const;
void SetGimbalPackage(VisionToGimbal recv_data);
```

**文件**: `io/gimbal/gimbal.cpp`

在 `send(io::VisionToGimbal VisionToGimbal)` 方法实现中，需要将 `VisionToGimbal` 参数的字段复制到 `tx_data_`：

```cpp
void Gimbal::send(io::VisionToGimbal VisionToGimbal)
{
  tx_data_.mode = VisionToGimbal.mode;
  tx_data_.yaw = VisionToGimbal.yaw;
  tx_data_.yaw_vel = VisionToGimbal.yaw_vel;
  tx_data_.yaw_acc = VisionToGimbal.yaw_acc;
  tx_data_.pitch = VisionToGimbal.pitch;
  tx_data_.pitch_vel = VisionToGimbal.pitch_vel;
  tx_data_.pitch_acc = VisionToGimbal.pitch_acc;
  tx_data_.vx = VisionToGimbal.vx;
  tx_data_.vy = VisionToGimbal.vy;
  tx_data_.posture = VisionToGimbal.posture;
  tx_data_.spin_flag = VisionToGimbal.spin_flag;
  tx_data_.scan = VisionToGimbal.scan;
  tx_data_.reverse = VisionToGimbal.reverse;
  // ↑ 新增字段也必须添加在此处，例如：
  // tx_data_.terrain_flag = VisionToGimbal.terrain_flag;
  tx_data_.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));
  // ... 日志输出和串口发送 ...
}
```

### 2.3 在 `plan_thread` 中统一组包
**文件**: `src/sentry_debug_mpc_ros.cpp`

所有下行数据必须在主程序的 `plan_thread` 中统一组包，然后调用 `send(VisionToGimbal)` 发送：

```cpp
auto plan_thread = std::thread([&]() {
  while (!quit) {
    // 1. 获取视觉规划结果
    auto target = target_queue.front();
    auto gs = gimbal.state();
    auto plan = planner.plan(target, gs.bullet_speed);

    // 2. 获取导航数据（通过 ROS 订阅回调）
    float vx = ros2_publisher->get_nav_vx();
    float vy = ros2_publisher->get_nav_vy();
    uint8_t posture = ros2_publisher->get_nav_posture();
    // ... 其他导航数据 ...

    // 3. 统一组包（唯一入口）
    io::VisionToGimbal msg;
    msg.mode = plan.control ? (plan.fire ? 2 : 1) : 0;
    msg.yaw = plan.yaw;
    msg.yaw_vel = plan.yaw_vel;
    msg.yaw_acc = plan.yaw_acc;
    msg.pitch = plan.pitch;
    msg.pitch_vel = plan.pitch_vel;
    msg.pitch_acc = plan.pitch_acc;
    msg.vx = vx;
    msg.vy = vy;
    msg.posture = posture;
    // ... 其他字段（如新增的 terrain_flag） ...
    // msg.terrain_flag = ros2_publisher->get_nav_terrain_flag();

    // 4. 调用唯一的 send 方法发送
    gimbal.send(msg);

    std::this_thread::sleep_for(10ms);
  }
});
```

**设计原则**：
- **单一入口**：所有下行数据必须通过 `send(VisionToGimbal)` 发送
- **集中处理**：组包逻辑集中在 `plan_thread` 中，便于维护和调试
- **解耦设计**：`Gimbal` 类只负责串口收发，不参与业务逻辑

---

## 三、添加/删除「视觉 → 导航」的 ROS Topic

### 3.1 创建/修改 `.msg` 消息类型
**文件**: `auto_aim_interfaces/msg/NewTopicName.msg`

```msg
# 示例消息格式
std_msgs/Header header  # 时间戳+坐标系
float32 field1          # 数值字段1
uint8 field2            # 数值字段2
bool flag               # 布尔标志
geometry_msgs/Point point  # 引用其他消息
```

### 3.2 修改 `CMakeLists.txt` 注册新消息
**文件**: `auto_aim_interfaces/CMakeLists.txt`

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Armor.msg"
  "msg/Armors.msg"
  "msg/Target.msg"
  "msg/TargetSentry.msg"
  "msg/Gimbal.msg"
  "msg/GimbalFeedback.msg"
  "msg/SentryStatus.msg"
  "msg/FromDecision.msg"
  "msg/NewTopicName.msg"  # 新增：在这里添加新的 .msg 文件名
  DEPENDENCIES std_msgs geometry_msgs
)
```

### 3.3 添加发布者和发布函数
**文件**: `src/sentry_debug_mpc_ros.cpp`

```cpp
class ROS2Publisher : public rclcpp::Node
{
public:
  ROS2Publisher(const std::string & config_path)
    : Node("sentry_debug_publisher")
    // ... 现有初始化 ...
    , new_topic_pub_(this->create_publisher<auto_aim_interfaces::msg::NewTopicName>("/new_topic_name", 10))  // 新增

  // ... 现有函数 ...

  // 新增：发布函数
  void publish_new_topic_name(/* 参数 */)
  {
    auto msg = std::make_shared<auto_aim_interfaces::msg::NewTopicName>();
    msg->header.stamp = this->now();
    msg->header.frame_id = "odom";  // 根据实际情况设置坐标系
    msg->field1 = /* 值 */;
    msg->field2 = /* 值 */;
    new_topic_pub_->publish(*msg);
  }

private:
  // ... 现有成员 ...
  rclcpp::Publisher<auto_aim_interfaces::msg::NewTopicName>::SharedPtr new_topic_pub_;  // 新增
};
```

### 3.4 在主循环中调用发布
**文件**: `src/sentry_debug_mpc_ros.cpp`

在主循环适当位置调用：

```cpp
ros2_publisher->publish_new_topic_name(/* 参数 */);
```

### 3.5 重新编译
修改完 `.msg` 和 `CMakeLists.txt` 后，需要重新编译 `auto_aim_interfaces` 包。

---

## 四、添加/删除「导航 → 视觉」的 ROS Topic

### 4.1 创建/修改 `.msg` 消息类型
**文件**: `auto_aim_interfaces/msg/NewTopicName.msg`

```msg
# 示例消息格式
std_msgs/Header header
float32 nav_field1
uint8 nav_field2
```

或者修改现有 `FromDecision.msg`：

```msg
uint8 posture
uint8 rotation_posture
float32 nav_field1  # 新增字段
```

### 4.2 修改 `CMakeLists.txt` （同 3.2）
**文件**: `auto_aim_interfaces/CMakeLists.txt`

如果创建了新的 `.msg`，需要在 `rosidl_generate_interfaces` 中添加。

### 4.3 添加订阅者和回调
**文件**: `src/sentry_debug_mpc_ros.cpp`

```cpp
class ROS2Publisher : public rclcpp::Node
{
public:
  ROS2Publisher(const std::string & config_path)
    : Node("sentry_debug_publisher")
    // ... 现有初始化 ...
    , new_topic_sub_(this->create_subscription<auto_aim_interfaces::msg::NewTopicName>(
        "/new_topic_name", 10,
        std::bind(&ROS2Publisher::new_topic_callback, this, std::placeholders::_1)))  // 新增

  // ... 现有函数 ...

  // 新增：回调函数
  void new_topic_callback(const auto_aim_interfaces::msg::NewTopicName::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(nav_mutex_);
    nav_field1_ = msg->nav_field1;
    nav_field2_ = msg->nav_field2;
  }

  // 新增：getter（供 plan_thread 使用）
  float get_nav_field1() const
  {
    std::lock_guard<std::mutex> lock(nav_mutex_);
    return nav_field1_;
  }

  uint8_t get_nav_field2() const
  {
    std::lock_guard<std::mutex> lock(nav_mutex_);
    return nav_field2_;
  }

private:
  // ... 现有成员 ...
  mutable std::mutex nav_mutex_;
  float nav_field1_ = 0.0f;  // 新增：存储订阅数据
  uint8_t nav_field2_ = 0;   // 新增
  rclcpp::Subscription<auto_aim_interfaces::msg::NewTopicName>::SharedPtr new_topic_sub_;  // 新增
};
```

### 4.4 在 `plan_thread` 中使用
**文件**: `src/sentry_debug_mpc_ros.cpp`

```cpp
auto plan_thread = std::thread([&]() {
  while (!quit) {
    // 获取导航数据
    float field1 = ros2_publisher->get_nav_field1();
    uint8_t field2 = ros2_publisher->get_nav_field2();

    // 使用数据组包（参考 2.3）
    // ...
  }
});
```

### 4.5 重新编译
如果修改了 `.msg`，需要重新编译 `auto_aim_interfaces` 包。

---

## 快速检查清单

| 操作 | 需要修改 |
|------|----------|
| 下位机 → 视觉 | `GimbalToVision`, `GimbalState`, `read_thread` |
| 视觉 → 下位机 | `VisionToGimbal` (hpp), **`send()` 方法实现** (cpp), `plan_thread`组包 |
| 视觉 → 导航 | `.msg`, `ROS2Publisher`发布者, 主循环调用 |
| 导航 → 视觉 | `.msg`, `ROS2Publisher`订阅+回调+getter, `plan_thread`使用 |

---

## 代码规范检查清单

| 检查项 | 要求 |
|--------|------|
| `Gimbal` 类 `send` 方法 | **仅允许** `void send(io::VisionToGimbal)` |
| 单独参数处理方法 | **禁止** `GetGimbalPackage()`, `GetFireMode()`, `SetGimbalPackage()` |
| 组包位置 | 所有下行数据必须在 `plan_thread` 中组包 |
| 互斥锁 | 多线程访问共享数据必须使用互斥锁保护 |
| `send()` 实现 | 必须将 `VisionToGimbal` 参数的字段复制到 `tx_data_` |

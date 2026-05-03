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

### 2.2 更新 `send` 方法
**文件**: `io/gimbal/gimbal.cpp`

```cpp
void Gimbal::send(io::VisionToGimbal VisionToGimbal)
{
  // ... 现有字段 ...
  tx_data_.crc16 = tools::get_crc16(/* ... */);
  serial_.write(/* ... */);
}
```

### 2.3 在 `plan_thread` 中组包
**文件**: `src/sentry_debug_mpc_ros.cpp`

在 `plan_thread` 循环中添加/删除：

```cpp
io::VisionToGimbal msg;
msg.yaw = plan.yaw;
// ... 现有字段 ...
msg.xxx = xxx;  // 新增字段赋值
gimbal.send(msg);
```

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

    // 使用数据
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
| 视觉 → 下位机 | `VisionToGimbal`, `send()`, `plan_thread`组包 |
| 视觉 → 导航 | `.msg`, `ROS2Publisher`发布者, 主循环调用 |
| 导航 → 视觉 | `.msg`, `ROS2Publisher`订阅+回调+getter, `plan_thread`使用 |

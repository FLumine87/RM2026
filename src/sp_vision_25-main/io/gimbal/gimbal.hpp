#ifndef IO__GIMBAL_HPP
#define IO__GIMBAL_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>

#include "serial/serial.h"
#include "tools/thread_safe_queue.hpp"



namespace io
{
struct __attribute__((packed)) GimbalToVision
{
  uint8_t head[2] = {'S', 'P'};
  uint8_t mode;          // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
  float q[4];            // wxyz顺序
  float yaw;
  float yaw_vel;
  float pitch;
  float pitch_vel;
  float bullet_speed;
  uint16_t bullet_count; // 子弹累计发送次数
  uint8_t is_play;
  uint16_t time_;        // 比赛时间
  float positon_x;       // 机器人自身位置
  float positon_y;
  float tar_positon_x;   // 目标地点位置（半自动模式使用）
  float tar_positon_y;
  uint16_t own_hp_;      // 己方血量
  uint16_t outpost_HP;   // 己方前哨站血量
  uint8_t outpost_status; // 前哨站占领状态
  uint8_t fort_status;    // 堡垒占领状态
  uint8_t reverse;
  uint16_t crc16;        // 校验值
};

static_assert(sizeof(GimbalToVision) <= 128);

struct __attribute__((packed)) VisionToGimbal
{
  uint8_t head[2] = {'S', 'P'};
  uint8_t mode;          // 0: 不控制, 1: 控制云台但不开火，2: 控制云台且开火
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
  float vx;              // X速度
  float vy;              // Y速度
  uint8_t posture;       // 姿态指令
  uint8_t spin_flag;     // 旋转指令
  uint8_t scan;          // 扫描指令
  uint8_t terrain_flag;  // 地形标志
  uint8_t reverse;       // 预留位
  uint16_t crc16;        // 校验值
};

static_assert(sizeof(VisionToGimbal) <= 64);

enum class GimbalMode
{
  IDLE,        // 空闲
  AUTO_AIM,    // 自瞄
  SMALL_BUFF,  // 小符
  BIG_BUFF     // 大符
};

struct GimbalState
{
  float yaw;
  float yaw_vel;
  float pitch;
  float pitch_vel;
  float bullet_speed;
  uint16_t bullet_count;
  uint8_t is_play;
  uint16_t time_;
  float q[4];
  float positon_x;
  float positon_y;
  float tar_positon_x;
  float tar_positon_y;
  uint16_t own_hp_;
  uint16_t outpost_HP;
  uint8_t outpost_status;
  uint8_t fort_status;
  uint8_t reverse;
  uint8_t mode;
};

class Gimbal
{
public:
  Gimbal(const std::string & config_path);

  ~Gimbal();

  GimbalMode mode() const;
  GimbalState state() const;
  std::string str(GimbalMode mode) const;
  Eigen::Quaterniond q(std::chrono::steady_clock::time_point t);

  void send(
    bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
    float pitch_acc);

  void send(io::VisionToGimbal VisionToGimbal);

private:
  serial::Serial serial_;

  std::thread thread_;
  std::atomic<bool> quit_ = false;
  mutable std::mutex mutex_;

  GimbalToVision rx_data_;
  VisionToGimbal tx_data_;

  GimbalMode mode_ = GimbalMode::IDLE;
  GimbalState state_;
  tools::ThreadSafeQueue<std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>>
    queue_{1000};

  bool read(uint8_t * buffer, size_t size);
  void read_thread();
  void reconnect();


};

}  // namespace io

#endif  // IO__GIMBAL_HPP
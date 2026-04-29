#include "gimbal.hpp"

#include <cmath>

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{
Gimbal::Gimbal(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "com_port");

  try {
    serial_.setPort(com_port);
    // serial_.setBaudrate(921600);
    serial_.open();
  } catch (const std::exception & e) {
    tools::logger()->error("[Gimbal] Failed to open serial: {}", e.what());
    exit(1);
  }

  thread_ = std::thread(&Gimbal::read_thread, this);

  queue_.pop();
  tools::logger()->info("[Gimbal] First q received.");

}

Gimbal::~Gimbal()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();

  serial_.close();
}

GimbalMode Gimbal::mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

GimbalState Gimbal::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

GimbalToVision Gimbal::GetGimbalPackage() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return rx_data_;
}

uint8_t Gimbal::GetFireMode() const
{
  return tx_data_.mode;
}

void Gimbal::SetGimbalPackage(VisionToGimbal recv_data)
{
  tx_data_.vx = recv_data.vx;
  tx_data_.vy = recv_data.vy;
  tx_data_.posture = recv_data.posture;
  tx_data_.rotation_posture = recv_data.rotation_posture;
  tx_data_.reverse = recv_data.reverse;
}

std::string Gimbal::str(GimbalMode mode) const
{
  switch (mode) {
    case GimbalMode::IDLE:
      return "IDLE";
    case GimbalMode::AUTO_AIM:
      return "AUTO_AIM";
    case GimbalMode::SMALL_BUFF:
      return "SMALL_BUFF";
    case GimbalMode::BIG_BUFF:
      return "BIG_BUFF";
    default:
      return "INVALID";
  }
}

Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t)
{
  while (true) {
    auto [q_a, t_a] = queue_.pop();
    auto [q_b, t_b] = queue_.front();
    auto t_ab = tools::delta_time(t_a, t_b);

    if (t > t_b) {
      continue;
    }

    while (std::abs(t_ab) < 15e-4) {
      queue_.pop();
      auto [new_q_b, new_t_b] = queue_.front();
      q_b = new_q_b;
      t_b = new_t_b;
      t_ab = tools::delta_time(t_a, t_b);
    }

    auto t_ac = tools::delta_time(t_a, t);
    auto k = t_ac / t_ab;
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();

    return q_c;
  }
}

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
  tx_data_.rotation_posture = VisionToGimbal.rotation_posture;
  tx_data_.reverse = VisionToGimbal.reverse;
  tx_data_.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

void Gimbal::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
  float pitch_acc)
{
  tx_data_.mode = control ? (fire ? 2 : 1) : 0;
  tx_data_.yaw = yaw;
  tx_data_.yaw_vel = yaw_vel;
  tx_data_.yaw_acc = yaw_acc;
  tx_data_.pitch = pitch;
  tx_data_.pitch_vel = pitch_vel;
  tx_data_.pitch_acc = pitch_acc;

  tx_data_.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

bool Gimbal::read(uint8_t * buffer, size_t size)
{
  try {
    return serial_.read(buffer, size) == size;
  } catch (const std::exception & e) {
    // tools::logger()->warn("[Gimbal] Failed to read serial: {}", e.what());
    return false;
  }
}

void Gimbal::read_thread()
{
  tools::logger()->info("[Gimbal] read_thread started.");

  std::vector<uint8_t> buffer;
  buffer.reserve(512);
  uint8_t temp_raw[256];
  int crc_error_cnt = 0;
  int header_error_cnt = 0;

  while (!quit_) {
    int n = serial_.read(temp_raw, sizeof(temp_raw));
    if (n <= 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    buffer.insert(buffer.end(), temp_raw, temp_raw + n);

    while (buffer.size() >= sizeof(rx_data_)) {
      if (buffer[0] == 'S' && buffer[1] == 'P') {
        auto t = std::chrono::steady_clock::now();

        if (tools::check_crc16(buffer.data(), sizeof(rx_data_))) {
          memcpy(&rx_data_, buffer.data(), sizeof(rx_data_));

          Eigen::Quaterniond q(rx_data_.q[0], rx_data_.q[1], rx_data_.q[2], rx_data_.q[3]);
          double q_norm_sq = q.w() * q.w() + q.x() * q.x() + q.y() * q.y() + q.z() * q.z();

          if (q_norm_sq < 1.1 && q_norm_sq > 0.9 && !std::isnan(q_norm_sq)) {
            queue_.push({q, t});
          } else {
            tools::logger()->warn("[Gimbal] 拦截到非法四元数包 (可能 MCU 刚重启)，已丢弃!");
          }

          std::lock_guard<std::mutex> lock(mutex_);

          state_.yaw = rx_data_.yaw;
          state_.yaw_vel = rx_data_.yaw_vel;
          state_.pitch = rx_data_.pitch;
          state_.pitch_vel = rx_data_.pitch_vel;
          state_.bullet_speed = rx_data_.bullet_speed;
          state_.bullet_count = rx_data_.bullet_count;
          state_.timestamp = 0;
          state_.hp = 0;
          state_.time = rx_data_.time_;
          state_.is_play = rx_data_.is_play;
          state_.enemy_score = rx_data_.enemy_score;
          state_.own_score = rx_data_.own_score;
          for (int i = 0; i < 3; ++i) {
            state_.own_hp[i] = rx_data_.own_hp_[i];
          }
          state_.occupy = rx_data_.occupy;
          state_.mode = rx_data_.mode;
          state_.reverse = rx_data_.reverse;

          switch (rx_data_.mode) {
            case 0:
              mode_ = GimbalMode::IDLE;
              break;
            case 1:
              mode_ = GimbalMode::AUTO_AIM;
              break;
            case 2:
              mode_ = GimbalMode::SMALL_BUFF;
              break;
            case 3:
              mode_ = GimbalMode::BIG_BUFF;
              break;
            default:
              mode_ = GimbalMode::IDLE;
              tools::logger()->warn("[Gimbal] Invalid mode: {}", rx_data_.mode);
              break;
          }
        } else {
          crc_error_cnt++;
          tools::logger()->warn("[Gimbal] CRC Failed! Count: {}", crc_error_cnt);
        }

        buffer.erase(buffer.begin(), buffer.begin() + sizeof(rx_data_));
      } else {
        header_error_cnt++;
        buffer.erase(buffer.begin());
      }
    }

    if (buffer.size() > 512) {
      tools::logger()->error("[Gimbal] Buffer overflow (size: {}). Clearing...", buffer.size());
      buffer.clear();
    }
  }

  tools::logger()->info("[Gimbal] read_thread stopped.");
}

void Gimbal::reconnect()
{
  int max_retry_count = 10;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn("[Gimbal] Reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);
    try {
      serial_.close();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } catch (...) {
    }

    try {
      serial_.open();
      queue_.clear();
      tools::logger()->info("[Gimbal] Reconnected serial successfully.");
      break;
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Reconnect failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}



}  // namespace io
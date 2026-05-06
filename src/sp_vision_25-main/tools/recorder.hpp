#ifndef TOOLS__RECORDER_HPP
#define TOOLS__RECORDER_HPP

#include <Eigen/Geometry>
#include <chrono>
#include <fstream>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <queue>
#include <thread>

#include "tools/thread_safe_queue.hpp"
namespace tools
{
class Recorder
{
public:
  Recorder(double fps = 15, bool enabled = true);
  ~Recorder();
  void record(
    const cv::Mat & img, const Eigen::Quaterniond & q,
    const std::chrono::steady_clock::time_point & timestamp);

private:
  struct FrameData
  {
    cv::Mat img;
    bool has_img;
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point timestamp;
  };
  
  bool init_;
  bool enabled_;
  std::atomic<bool> stop_thread_;
  double fps_;
  double text_fps_;  // 云台数据写入帧率（30fps）
  std::string text_path_;
  std::string video_path_;
  std::string folder_path_;
  std::ofstream text_writer_;
  cv::VideoWriter video_writer_;
  cv::Size img_size_;
  int fourcc_;
  std::chrono::steady_clock::time_point start_time_;
  std::chrono::steady_clock::time_point last_time_;
  std::chrono::steady_clock::time_point last_text_time_;
  tools::ThreadSafeQueue<FrameData> queue_;
  std::thread saving_thread_;
  
  // 分块录制相关
  int block_frame_counter_;
  const int block_duration_minutes_ = 1;  // 每1分钟一个块
  const int flush_interval_frames_ = 90;  // 每90帧刷新一次
  int flush_counter_;
  
  void init(const cv::Mat & img);
  void save_to_file();
  void create_new_block();
};

}  // namespace tools

#endif  // TOOLS__RECORDER_HPP
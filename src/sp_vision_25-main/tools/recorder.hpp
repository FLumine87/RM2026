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
  Recorder(double fps = 15, bool enabled = true, bool compress_enabled = true);
  ~Recorder();
  void record(
    const cv::Mat & img, const Eigen::Quaterniond & q,
    const std::chrono::steady_clock::time_point & timestamp);

private:
  struct FrameData
  {
    cv::Mat img;
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point timestamp;
  };
  
  bool init_;
  bool enabled_;
  std::atomic<bool> stop_thread_;
  std::atomic<bool> pause_compression_;
  std::atomic<bool> compressing_;
  double fps_;
  std::string text_path_;
  std::string video_path_;
  std::string folder_path_;
  std::ofstream text_writer_;
  cv::VideoWriter video_writer_;
  cv::Size img_size_;
  int fourcc_;
  std::chrono::steady_clock::time_point start_time_;
  std::chrono::steady_clock::time_point last_time_;
  tools::ThreadSafeQueue<FrameData> queue_;
  std::thread saving_thread_;
  std::thread compress_thread_;
  
  // 分块录制相关
  int block_frame_counter_;
  const int block_duration_minutes_ = 2;  // 每2分钟一个块
  const int flush_interval_frames_ = 100;  // 每100帧刷新一次
  int flush_counter_;
  
  // 压缩队列
  std::queue<std::string> compress_queue_;
  std::mutex queue_mutex_;
  bool compress_enabled_;  // 是否启用压缩
  
  void init(const cv::Mat & img);
  void save_to_file();
  void create_new_block();
  void queue_for_compression(const std::string & file_path);
  void process_compress_queue();
  void compress_block(const std::string & file_path);
  bool is_high_load();
  void set_low_priority();
};

}  // namespace tools

#endif  // TOOLS__RECORDER_HPP
#include "recorder.hpp"

#include <chrono>
#include <fmt/chrono.h>

using namespace std::chrono_literals;

#include <filesystem>
#include <string>

#include "math_tools.hpp"
#include "tools/logger.hpp"

#ifdef _WIN32
#include <windows.h>
#include <psapi.h>
#else
#include <pthread.h>
#include <sched.h>
#include <sys/resource.h>
#endif

namespace tools
{
Recorder::Recorder(double fps, bool enabled, bool compress_enabled) 
    : init_(false), enabled_(enabled), fps_(fps), queue_(1), 
      stop_thread_(false), pause_compression_(false), compressing_(false),
      block_frame_counter_(0), flush_counter_(0), compress_enabled_(compress_enabled)
{
  start_time_ = std::chrono::steady_clock::now();
  last_time_ = start_time_;
  folder_path_ = "records";
  std::filesystem::create_directory(folder_path_);
  
  // 生成初始文件名
  auto file_name = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
  text_path_ = fmt::format("{}/{}.txt", folder_path_, file_name);
  video_path_ = fmt::format("{}/{}.avi", folder_path_, file_name);
}

Recorder::~Recorder()
{
  stop_thread_ = true;
  pause_compression_ = false;  // 允许压缩线程完成
  
  // 退出时给队列中额外推入一个空帧，避免pop一直等待
  queue_.push({cv::Mat::zeros(0, 0, 0), {0, 0, 0, 0}, std::chrono::steady_clock::now()});
  
  if (saving_thread_.joinable()) saving_thread_.join();
  
  // 等待压缩线程完成
  if (compress_thread_.joinable()) compress_thread_.join();

  if (!init_) return;
  text_writer_.close();
  video_writer_.release();
}

void Recorder::save_to_file()
{
  while (!stop_thread_) {
    FrameData frame;
    queue_.pop(frame);
    
    if (frame.img.empty()) {
      tools::logger()->debug("Recorder received empty img. Skip this frame.");
      continue;
    }
    
    // 检查是否需要创建新块
    if (block_frame_counter_ >= static_cast<int>(fps_ * 60 * block_duration_minutes_)) {
      // 关闭当前文件
      if (video_writer_.isOpened()) {
        video_writer_.release();
        text_writer_.close();
        
        // 将完成的块加入压缩队列
        queue_for_compression(video_path_);
        
        // 创建新块
        create_new_block();
        block_frame_counter_ = 0;
      }
    }
    
    // 定期刷新
    flush_counter_++;
    if (flush_counter_ >= flush_interval_frames_) {
      if (video_writer_.isOpened()) {
        video_writer_.release();
        video_writer_.open(video_path_, fourcc_, fps_, img_size_, true);
      }
      // 刷新txt文件缓冲区到磁盘，防止断电丢失
      if (text_writer_.is_open()) {
        text_writer_.flush();
      }
      flush_counter_ = 0;
    }
    
    // 写入视频文件
    if (video_writer_.isOpened()) {
      video_writer_.write(frame.img);
    }
    
    // 写入文本文件（输出顺序为wxyz）
    if (text_writer_.is_open()) {
      Eigen::Vector4d xyzw = frame.q.coeffs();
      auto since_begin = tools::delta_time(frame.timestamp, start_time_);
      text_writer_ << fmt::format(
        "{} {} {} {} {}\n", since_begin, xyzw[3], xyzw[0], xyzw[1], xyzw[2]);
    }
    
    block_frame_counter_++;
  }
}

void Recorder::create_new_block()
{
  auto file_name = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
  video_path_ = fmt::format("{}/{}.avi", folder_path_, file_name);
  text_path_ = fmt::format("{}/{}.txt", folder_path_, file_name);
  
  video_writer_.open(video_path_, fourcc_, fps_, img_size_, true);
  text_writer_.open(text_path_);
  
  tools::logger()->info("Created new recording block: {}", video_path_);
}

void Recorder::queue_for_compression(const std::string & file_path)
{
  // 如果禁用压缩，跳过
  if (!compress_enabled_) return;
  
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    compress_queue_.push(file_path);
  }
  
  // 如果没有压缩线程在运行，启动一个
  if (!compressing_) {
    compressing_ = true;
    compress_thread_ = std::thread(&Recorder::process_compress_queue, this);
  }
}

void Recorder::process_compress_queue()
{
  while (!stop_thread_ || !compress_queue_.empty()) {
    // 检查是否暂停
    while (pause_compression_ && !stop_thread_) {
      std::this_thread::sleep_for(100ms);
    }
    
    // 检查系统负载
    if (is_high_load() && !stop_thread_) {
      std::this_thread::sleep_for(200ms);
      continue;
    }
    
    std::string file_path;
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (compress_queue_.empty()) {
        if (stop_thread_) break;
        std::this_thread::sleep_for(100ms);
        continue;
      }
      file_path = compress_queue_.front();
      compress_queue_.pop();
    }
    
    // 执行压缩
    compress_block(file_path);
  }
  
  compressing_ = false;
}

void Recorder::compress_block(const std::string & file_path)
{
  // 设置低优先级
  set_low_priority();
  
  // 构建压缩命令（使用7z，中等压缩级别）
  std::string output_path = file_path.substr(0, file_path.size() - 4) + ".7z";
  std::string txt_path = file_path.substr(0, file_path.size() - 4) + ".txt";
  std::string cmd = fmt::format("7z a -mx=1 -t7z \"{}\" \"{}\" \"{}\"", 
                                output_path, file_path, txt_path);
  
  tools::logger()->info("Compressing: {}", file_path);
  int result = std::system(cmd.c_str());
  
  if (result == 0) {
    tools::logger()->info("Compression successful: {}", output_path);
    // 删除原始文件
    std::remove(file_path.c_str());
    std::remove(txt_path.c_str());
  } else {
    tools::logger()->warn("Compression failed for: {}", file_path);
  }
}

bool Recorder::is_high_load()
{
  // 简单的负载检测：检查内存使用情况
  // 如果可用内存少于500MB，认为是高负载
  const size_t min_available_memory = 500 * 1024 * 1024;  // 500MB
  
#ifdef _WIN32
  MEMORYSTATUSEX statex;
  statex.dwLength = sizeof(statex);
  GlobalMemoryStatusEx(&statex);
  return statex.ullAvailPhys < min_available_memory;
#else
  // Linux系统读取/proc/meminfo
  std::ifstream meminfo("/proc/meminfo");
  std::string line;
  while (std::getline(meminfo, line)) {
    if (line.find("MemAvailable:") != std::string::npos) {
      size_t available = std::stoull(line.substr(15));
      return (available * 1024) < min_available_memory;
    }
  }
  return false;
#endif
}

void Recorder::set_low_priority()
{
#ifdef _WIN32
  SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_LOWEST);
#else
  pthread_t thread = pthread_self();
  struct sched_param param;
  param.sched_priority = sched_get_priority_min(SCHED_IDLE);
  pthread_setschedparam(thread, SCHED_IDLE, &param);
#endif
}

void Recorder::record(
  const cv::Mat & img, const Eigen::Quaterniond & q,
  const std::chrono::steady_clock::time_point & timestamp)
{
  // 如果录制被禁用或图像为空，直接返回
  if (!enabled_ || img.empty()) return;
  
  // 检查系统资源
  if (is_high_load()) {
    tools::logger()->warn("High system load, skipping frame");
    return;
  }
  
  if (!init_) init(img);

  auto since_last = tools::delta_time(timestamp, last_time_);
  if (since_last < 1.0 / fps_) return;

  last_time_ = timestamp;
  queue_.push({img.clone(), q, timestamp});
}

void Recorder::init(const cv::Mat & img)
{
  img_size_ = img.size();
  
  // 使用H.264编码
  fourcc_ = cv::VideoWriter::fourcc('H', '2', '6', '4');
  video_writer_ = cv::VideoWriter(video_path_, fourcc_, fps_, img_size_);
  
  // 如果H.264不可用，回退到MJPG
  if (!video_writer_.isOpened()) {
    tools::logger()->warn("H.264 codec not available, falling back to MJPG");
    fourcc_ = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    video_writer_ = cv::VideoWriter(video_path_, fourcc_, fps_, img_size_);
  }
  
  text_writer_.open(text_path_);
  saving_thread_ = std::thread(&Recorder::save_to_file, this);
  
  tools::logger()->info("Recorder initialized with {} fps, codec: {}", 
                        fps_, fourcc_ == cv::VideoWriter::fourcc('H', '2', '6', '4') ? "H264" : "MJPG");
  
  init_ = true;
}

}  // namespace tools
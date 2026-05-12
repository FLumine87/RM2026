#include "recorder.hpp"

#include <chrono>
#include <fmt/chrono.h>

using namespace std::chrono_literals;

#include <filesystem>
#include <string>

#include "math_tools.hpp"
#include "tools/logger.hpp"

namespace tools
{
Recorder::Recorder(double fps, bool enabled) 
    : init_(false), enabled_(enabled), fps_(fps), text_fps_(fps * 2), queue_(1), 
      stop_thread_(false), block_frame_counter_(0), flush_counter_(0), tracked_file_count_(0)
{
  start_time_ = std::chrono::steady_clock::now();
  last_time_ = start_time_;
  last_text_time_ = start_time_;
  folder_path_ = "records";
  std::filesystem::create_directory(folder_path_);
  
  // 初始化时统计已有文件数量
  namespace fs = std::filesystem;
  try {
    for (const auto& entry : fs::directory_iterator(folder_path_)) {
      if (entry.is_regular_file() && entry.path().extension() == ".avi") {
        tracked_file_count_++;
      }
    }
    tools::logger()->info("Found {} existing recording files in records folder", tracked_file_count_);
  } catch (const fs::filesystem_error& e) {
    tools::logger()->warn("Error counting existing files: {}", e.what());
    tracked_file_count_ = 0;
  }
  
  // 生成初始文件名
  auto file_name = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
  text_path_ = fmt::format("{}/{}.txt", folder_path_, file_name);
  video_path_ = fmt::format("{}/{}.avi", folder_path_, file_name);
}

Recorder::~Recorder()
{
  stop_thread_ = true;
  
  // 退出时给队列中额外推入一个空帧，避免pop一直等待
  queue_.push({cv::Mat::zeros(0, 0, 0), false, {0, 0, 0, 0}, std::chrono::steady_clock::now()});
  
  if (saving_thread_.joinable()) saving_thread_.join();

  if (!init_) return;
  text_writer_.close();
  video_writer_.release();
}

void Recorder::save_to_file()
{
  while (!stop_thread_) {
    FrameData frame;
    queue_.pop(frame);
    
    // 检查是否需要创建新块（基于图像帧数）
    if (block_frame_counter_ >= static_cast<int>(fps_ * 60 * block_duration_minutes_)) {
      // 关闭当前文件
      if (video_writer_.isOpened()) {
        video_writer_.release();
        text_writer_.close();
        
        // 创建新块
        create_new_block();
        block_frame_counter_ = 0;
      }
    }
    
    // 定期刷新
    flush_counter_++;
    if (flush_counter_ >= flush_interval_frames_) {
      // 刷新txt文件缓冲区到磁盘，防止断电丢失
      if (text_writer_.is_open()) {
        text_writer_.flush();
      }
      flush_counter_ = 0;
    }
    
    // 写入视频文件（只有当 has_img 为 true 时）
    if (frame.has_img && video_writer_.isOpened()) {
      video_writer_.write(frame.img);
      block_frame_counter_++;  // 只有图像帧才计数
    }
    
    // 写入文本文件（每次都写，实现30fps云台数据）
    if (text_writer_.is_open()) {
      Eigen::Vector4d xyzw = frame.q.coeffs();
      auto since_begin = tools::delta_time(frame.timestamp, start_time_);
      text_writer_ << fmt::format(
        "{} {} {} {} {}\n", since_begin, xyzw[3], xyzw[0], xyzw[1], xyzw[2]);
    }
  }
}

void Recorder::create_new_block()
{
  // 创建新块前先清理旧文件
  clean_old_files();
  
  auto file_name = fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now());
  video_path_ = fmt::format("{}/{}.avi", folder_path_, file_name);
  text_path_ = fmt::format("{}/{}.txt", folder_path_, file_name);
  
  // 使用FFmpeg管道方式确保码率控制（仅对H.264）
  if (fourcc_ == cv::VideoWriter::fourcc('H', '2', '6', '4')) {
    std::string ffmpeg_cmd = fmt::format(
      "ffmpeg -y -f rawvideo -vcodec rawvideo -pix_fmt bgr24 -s {}x{} -r {} -i - -c:v libx264 -b:v 5M -preset fast -f avi {}",
      img_size_.width, img_size_.height, fps_, video_path_
    );
    video_writer_.open(ffmpeg_cmd, cv::CAP_FFMPEG, 0, fps_, img_size_, true);
  } else {
    // MJPG直接打开
    video_writer_.open(video_path_, fourcc_, fps_, img_size_, true);
  }
  
  // 如果打开失败，尝试重新打开（可能是资源问题）
  if (!video_writer_.isOpened()) {
    tools::logger()->warn("Failed to open video writer for new block, retrying...");
    video_writer_.release();
    video_writer_.open(video_path_, fourcc_, fps_, img_size_, true);
  }
  
  text_writer_.open(text_path_);
  
  tools::logger()->info("Created new recording block: {}, codec: {}, opened: {}", 
                        video_path_, fourcc_ == cv::VideoWriter::fourcc('H', '2', '6', '4') ? "H264" : "MJPG",
                        video_writer_.isOpened() ? "success" : "failed");
}

void Recorder::record(
  const cv::Mat & img, const Eigen::Quaterniond & q,
  const std::chrono::steady_clock::time_point & timestamp)
{
  // 如果录制被禁用或图像为空，直接返回
  if (!enabled_ || img.empty()) return;
  
  // 初始化检查
  if (!init_) {
    init(img);
    // 如果初始化失败，禁用录制
    if (!init_) {
      tools::logger()->error("Recorder init failed, disabling recording");
      enabled_ = false;
      return;
    }
  }

  bool should_write_img = false;
  bool should_write_text = false;
  
  // 图像帧率控制（15fps）
  auto since_last_img = tools::delta_time(timestamp, last_time_);
  if (since_last_img >= 1.0 / fps_) {
    should_write_img = true;
    last_time_ = timestamp;
  }
  
  // 云台数据帧率控制（30fps，是图像的2倍）
  auto since_last_text = tools::delta_time(timestamp, last_text_time_);
  if (since_last_text >= 1.0 / text_fps_) {
    should_write_text = true;
    last_text_time_ = timestamp;
  }
  
  // 如果需要写入图像或文本，推入队列
  if (should_write_img || should_write_text) {
    queue_.push({should_write_img ? img.clone() : cv::Mat(), should_write_img, q, timestamp});
  }
}

void Recorder::init(const cv::Mat & img)
{
  img_size_ = img.size();
  
  // 先尝试H.264编码（文件更小）
  fourcc_ = cv::VideoWriter::fourcc('H', '2', '6', '4');
  video_writer_ = cv::VideoWriter();
  
  // 使用FFmpeg管道方式确保码率控制
  std::string ffmpeg_cmd = fmt::format(
    "ffmpeg -y -f rawvideo -vcodec rawvideo -pix_fmt bgr24 -s {}x{} -r {} -i - -c:v libx264 -b:v 5M -preset fast -f avi {}",
    img_size_.width, img_size_.height, fps_, video_path_
  );
  
  video_writer_.open(ffmpeg_cmd, cv::CAP_FFMPEG, 0, fps_, img_size_, true);
  
  // 如果FFmpeg管道失败，尝试常规方式
  if (!video_writer_.isOpened()) {
    tools::logger()->warn("FFmpeg pipe failed, trying direct open");
    video_writer_.open(video_path_, fourcc_, fps_, img_size_, true);
    
    // 设置码率为5Mbps
    video_writer_.set(cv::CAP_PROP_BITRATE, 5000000);
  }
  
  // 如果H.264失败，回退到MJPG
  if (!video_writer_.isOpened()) {
    tools::logger()->info("H.264 codec not available, falling back to MJPG");
    fourcc_ = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    video_writer_ = cv::VideoWriter(video_path_, fourcc_, fps_, img_size_, true);
  }
  
  // 如果都失败，返回错误
  if (!video_writer_.isOpened()) {
    tools::logger()->error("Failed to open video writer with both H.264 and MJPG");
    return;
  }
  
  text_writer_.open(text_path_);
  if (!text_writer_.is_open()) {
    tools::logger()->error("Failed to open text file: {}", text_path_);
    video_writer_.release();
    return;
  }
  
  saving_thread_ = std::thread(&Recorder::save_to_file, this);
  
  tools::logger()->info("Recorder initialized with {} fps, codec: {}, bitrate: 5Mbps", 
                        fps_, fourcc_ == cv::VideoWriter::fourcc('H', '2', '6', '4') ? "H264" : "MJPG");
  
  init_ = true;
}

void Recorder::clean_old_files()
{
  namespace fs = std::filesystem;
  
  // 优化：使用计数器追踪，只在文件数量接近限制时才遍历清理
  // 每创建新块时计数+1，当计数达到阈值时才真正遍历文件系统
  if (tracked_file_count_ < max_file_count_) {
    tracked_file_count_++;
    return;
  }
  
  // 达到阈值，执行真正的清理
  try {
    std::vector<fs::path> avi_files;
    
    // 只收集 avi 文件（txt 会根据 avi 的 stem 名称删除）
    for (const auto& entry : fs::directory_iterator(folder_path_)) {
      if (entry.is_regular_file() && entry.path().extension() == ".avi") {
        avi_files.push_back(entry.path());
      }
    }
    
    int current_count = static_cast<int>(avi_files.size());
    
    if (current_count > max_file_count_) {
      std::sort(avi_files.begin(), avi_files.end(), 
                [](const fs::path& a, const fs::path& b) {
                  return fs::last_write_time(a) < fs::last_write_time(b);
                });
      
      int to_delete = current_count - max_file_count_;
      
      for (int i = 0; i < to_delete; ++i) {
        auto avi_path = avi_files[i];
        auto txt_path = avi_path.parent_path() / (avi_path.stem().string() + ".txt");
        
        if (fs::exists(avi_path)) {
          fs::remove(avi_path);
        }
        if (fs::exists(txt_path)) {
          fs::remove(txt_path);
        }
      }
      
      tools::logger()->info("Cleaned {} old recording files, remaining: {}", 
                           to_delete, max_file_count_);
    }
    
    // 重置计数器：当前文件数量就是 max_file_count_
    tracked_file_count_ = max_file_count_;
    
  } catch (const fs::filesystem_error& e) {
    tools::logger()->warn("Filesystem error during clean_old_files: {}", e.what());
    tracked_file_count_ = 0;  // 出错时重置，下次再检查
  }
}

}  // namespace tools

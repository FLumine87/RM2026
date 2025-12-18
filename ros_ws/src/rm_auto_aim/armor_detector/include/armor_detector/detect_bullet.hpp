#ifndef ARMOR_DETECTOR__DETECT_BULLET_HPP_
#define ARMOR_DETECTOR__DETECT_BULLET_HPP_

#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include <vector>
#include "armor_detector/do_reproj.hpp"

namespace rm_auto_aim
{

struct Bullet
{
  cv::Point2f center;
  float radius;
  Bullet(const cv::Point2f& c, float r) : center(c), radius(r) {}
};

class DetectBullet
{
public:
  DetectBullet();

	// 主入口：处理新帧
  std::vector<Bullet> process_new_frame(const cv::Mat& new_frame, const Eigen::Quaterniond& q);

  // 直接在内部构造和初始化do_reproj_
  void initReprojFromMat(const cv::Mat& cam, const cv::Mat& imu);
  // 兼容：外部传入DoReproj对象
  // void setReproj(const DoReproj& do_reproj);

	// 获取**候选区域**
  void get_possible();
  // 颜色掩码生成（inRange）
  cv::Mat getColorMask(const cv::Mat& hsv);
  // 亮度掩码生成
  cv::Mat getBrightnessMask(const cv::Mat& hsv);

  // 获取子弹
  void get_bullets();
  // 子弹验证
  bool test_is_bullet(const std::vector<cv::Point>& contour);
  // 轮廓点排序
  void sort_points(std::vector<cv::Point>& vec);
	// 结果绘制
	void drawResults(cv::Mat& img);

  // 可调参数
  int binary_thres;
  float min_area, max_area;
  float min_ratio, max_ratio;
  bool bullet_visual_debug;

  // 算法参数（构造函数中初始化）
  cv::Size Kernel1_Size;
  cv::Size Kernel2_Size;
  cv::Scalar Color_LowB;
  cv::Scalar Color_UpB;
  cv::Scalar Min_Value;
  // 性能统计
  double tme_total = 0, tme_diff = 0, tme_contour = 0, tme_bright = 0;
  // 输出结果
  std::vector<Bullet> bullets;

private:
  // 上一帧弹丸掩码
  cv::Mat last_bullet_mask_;
  // 上一帧弹丸结果
  std::vector<Bullet> last_bullets_;
  // 上一帧原图缓存
  cv::Mat last_cur_frame_;
  // 上一帧HSV缓存
  cv::Mat last_cur_hsv_;

	// 帧差器
  DoFrameDifference frame_diff_;
	// 重投影器
  DoReproj do_reproj_;
  bool has_reproj_ = false;
};

class DoFrameDifference
{
public:
  DoFrameDifference();
  // 帧差掩码生成
  cv::Mat getDiff(const cv::Mat& cur_hsv,
                  const cv::Mat& last_hsv_reproj,
                  const cv::Mat& ref_mask,
                  const cv::Mat& last_bullet_mask);
                  
private:
  // 帧差参数
  float Weights[3];
  uint8_t Diff_Step;
  uint8_t Diff_Threshold;
  cv::Mat kernel1_;
  double tme_ = 0;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__DETECT_BULLET_HPP_
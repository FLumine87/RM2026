#include "armor_detector/detect_bullet.hpp"

//ROS2
#include <rclcpp/duration.hpp>
#include <rclcpp/qos.hpp>

namespace rm_auto_aim
{

// 构造函数需要传入rclcpp::Node* node，参考detector_node.cpp参数读取方式
DetectBullet::DetectBullet(rclcpp::Node* node)
{
	// 可调参数（declare_parameter，支持运行时动态修改）
	binary_thres = node->declare_parameter<int>("bullet_binary_thres", 180);
	min_area = node->declare_parameter<double>("bullet_min_area", 10.0);
	max_area = node->declare_parameter<double>("bullet_max_area", 500.0);
	min_ratio = node->declare_parameter<double>("bullet_min_ratio", 0.7);
	max_ratio = node->declare_parameter<double>("bullet_max_ratio", 1.3);
	bullet_visual_debug = node->declare_parameter<bool>("bullet_visual_debug", false);

	// 算法参数（只初始化一次即可）
	auto kernel1_size = node->declare_parameter<std::vector<int>>("bullet_kernel1_size", {10, 10});
	Kernel1_Size = cv::Size(kernel1_size[0], kernel1_size[1]);
	auto kernel2_size = node->declare_parameter<std::vector<int>>("bullet_kernel2_size", {4, 4});
	Kernel2_Size = cv::Size(kernel2_size[0], kernel2_size[1]);
	auto color_lowb = node->declare_parameter<std::vector<int>>("bullet_color_lowb", {25, 40, 40});
	Color_LowB = cv::Scalar(color_lowb[0], color_lowb[1], color_lowb[2]);
	auto color_upb = node->declare_parameter<std::vector<int>>("bullet_color_upb", {90, 255, 255});
	Color_UpB = cv::Scalar(color_upb[0], color_upb[1], color_upb[2]);
	auto min_value = node->declare_parameter<std::vector<double>>("bullet_min_value", {0, 25.5, 51.0});
	Min_Value = cv::Scalar(min_value[0], min_value[1], min_value[2]);

	// 性能统计初始化
	tme_total = 0;
	tme_diff = 0;
	tme_contour = 0;
	tme_bright = 0;

	// 输出结果清空
	bullets.clear();

	// 其余成员初始化
	last_bullet_mask_ = cv::Mat();
	last_bullets_.clear();
	last_cur_frame_ = cv::Mat();
	last_cur_hsv_ = cv::Mat();
	has_reproj_ = false;

	// 帧差器参数初始化
	auto diff_weights = node->declare_parameter<std::vector<double>>("bullet_diff_weights", {4.0, 4.0, 2.0});
	frame_diff_.Weights[0] = diff_weights[0];
	frame_diff_.Weights[1] = diff_weights[1];
	frame_diff_.Weights[2] = diff_weights[2];
	frame_diff_.Diff_Step = node->declare_parameter<int>("bullet_diff_step", 5);
	frame_diff_.Diff_Threshold = node->declare_parameter<int>("bullet_diff_threshold", 30);
	frame_diff_.kernel1_ = cv::getStructuringElement(cv::MORPH_RECT, Kernel1_Size);
	frame_diff_.tme_ = 0;
}

std::vector<Bullet> DetectBullet::process_new_frame(const cv::Mat& new_frame, const Eigen::Quaterniond& q)
{
	// TODO: 处理新帧
	return {};
}

void DetectBullet::get_possible()
{
	// TODO: 获取候选区域
}

void DetectBullet::initReprojFromMat(const cv::Mat& cam, const cv::Mat& imu)
{
	// TODO: 初始化重投影器
}

// DoFrameDifference在DetectBullet构造时初始化参数
DoFrameDifference::DoFrameDifference() {}

cv::Mat DoFrameDifference::getDiff(const cv::Mat& cur_hsv, const cv::Mat& last_hsv_reproj, const cv::Mat& ref_mask, const cv::Mat& last_bullet_mask)
{
	// TODO: 帧差掩码生成
	return cv::Mat();
}

cv::Mat DetectBullet::getColorMask(const cv::Mat& hsv)
{
	// 颜色掩码生成，参考aimer::aim实现
	cv::Mat mask;
	cv::inRange(hsv, Color_LowB, Color_UpB, mask);
	return mask;
}

cv::Mat DetectBullet::getBrightnessMask(const cv::Mat& hsv)
{
	// 亮度掩码生成，参考aimer::aim实现
	cv::Mat mask;
	cv::inRange(hsv, Min_Value, cv::Scalar(255, 255, 255), mask);
	return mask;
}

void DetectBullet::get_bullets()
{
	// TODO: 获取子弹
}

bool DetectBullet::test_is_bullet(const std::vector<cv::Point>& contour)
{
	// TODO: 子弹验证
	return false;
}

void DetectBullet::sort_points(std::vector<cv::Point>& vec)
{
	// TODO: 轮廓点排序
}

void DetectBullet::drawResults(cv::Mat& img)
{
	// TODO: 结果绘制
}

}  // namespace rm_auto_aim

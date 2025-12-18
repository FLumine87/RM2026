#include "armor_detector/detect_bullet.hpp"

namespace rm_auto_aim
{

DetectBullet::DetectBullet()
{
	// TODO: 初始化参数
}

std::vector<Bullet> DetectBullet::process_new_frame(const cv::Mat& new_frame, const Eigen::Quaterniond& q)
{
	// TODO: 处理新帧
	return {};
}

void DetectBullet::initReprojFromMat(const cv::Mat& cam, const cv::Mat& imu)
{
	// TODO: 初始化重投影器
}

DoFrameDifference::DoFrameDifference()
{
	// TODO: 初始化参数
}

cv::Mat DoFrameDifference::getDiff(const cv::Mat& cur_hsv, const cv::Mat& last_hsv_reproj, const cv::Mat& ref_mask, const cv::Mat& last_bullet_mask)
{
	// TODO: 帧差掩码生成
	return cv::Mat();
}

void DetectBullet::get_possible()
{
	// TODO: 获取候选区域
}

cv::Mat DetectBullet::getColorMask(const cv::Mat& hsv)
{
	// TODO: 颜色掩码生成
	return cv::Mat();
}

cv::Mat DetectBullet::getBrightnessMask(const cv::Mat& hsv)
{
	// TODO: 亮度掩码生成
	return cv::Mat();
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

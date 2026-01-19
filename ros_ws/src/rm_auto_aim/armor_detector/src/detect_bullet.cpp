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
	tme_sort_points_ = 0;

	// 输出结果清空
	bullets.clear();

	// 其余成员初始化
	last_bullet_mask_ = cv::Mat();
	last_bullets_.clear();
	last_cur_frame_ = cv::Mat();
	last_cur_hsv_ = cv::Mat();
	cur_frame_ = cv::Mat();
	cur_hsv_ = cv::Mat();
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

std::vector<Bullet> DetectBullet::processNewFrame(const cv::Mat& new_frame, const Eigen::Quaterniond& q)
{
	tme_total -= (double)clock() / CLOCKS_PER_SEC;

	// 保存上一帧数据
	last_cur_hsv_ = cur_hsv_.clone();
	last_cur_frame_ = cur_frame_.clone();
	cur_frame_ = new_frame.clone();
	last_q_ = cur_q_;
	cur_q_ = q;

	// 转换当前帧为HSV
	cv::cvtColor(cur_frame_, cur_hsv_, cv::COLOR_BGR2HSV);

	if (!last_cur_frame_.empty()) {
		getPossible();
		getBullets();
	}

	tme_total += (double)clock() / CLOCKS_PER_SEC;
	return bullets;
}

void DetectBullet::initReprojFromMat(const cv::Mat& cam, const cv::Mat& odom)
{
	// 初始化重投影器
	do_reproj_ = DoReproj(cam, odom);
	has_reproj_ = true;
}

void DetectBullet::getPossible()
{
	tme_contour -= (double)clock() / CLOCKS_PER_SEC;

	// 检查上一帧hsv是否为空
	if (last_cur_hsv_.empty() || !has_reproj_) return;

	// 对上一帧的hsv进行重投影
	cv::Mat last_reproj = do_reproj_.reproject(last_cur_hsv_, last_q_, cur_q_);

	// 颜色掩码
	cv::Mat color_mask, not_dark_mask;
	cv::inRange(cur_hsv_, Color_LowB, Color_UpB, color_mask);
	// 亮度掩码
	cv::inRange(cur_hsv_, Min_Value, cv::Scalar(255, 255, 255), not_dark_mask);
	color_mask &= not_dark_mask;

	// 帧差掩码
	cv::Mat diff_mask = frame_diff_.getDiff(cur_hsv_, last_reproj, color_mask, last_bullet_mask_);
	// 结合帧差和颜色掩码
	color_mask &= diff_mask;

	// 形态学滤波
	cv::morphologyEx(color_mask, color_mask, cv::MORPH_OPEN, Kernel2_Size);

	// 轮廓提取
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(color_mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
	candidate_contours_ = contours;

	tme_contour += (double)clock() / CLOCKS_PER_SEC;
	//处理嵌套轮廓
	// for (const cv::Vec4i& vc: hierarchy) {
	// 	if (~vc[3]) {
	// 		// std::cerr << "Nested contours!" << std::endl;
	// 	}
	// }
}

void DetectBullet::getBullets()
{
	bullets.clear();
	// 新一帧的弹丸掩码，全部置零
	last_bullet_mask_ = cv::Mat::zeros(cur_frame_.rows, cur_frame_.cols, CV_8U);

	// 遍历所有候选轮廓
	for (uint32_t i = 0; i < candidate_contours_.size(); ++i) {
		const std::vector<cv::Point>& contour = candidate_contours_[i];
		cv::RotatedRect rect = cv::minAreaRect(contour);
		cv::Size rect_size = rect.size;
		// 面积过滤
		if (rect_size.area() < min_area)
			continue;
		double ratio = cv::contourArea(contour) / rect_size.area();
		// 形状过滤
		if (ratio < min_ratio)
			continue;
		if (ratio > max_ratio)
			continue;
		// 颜色验证
		if (testIsBullet(contour)) {
			bullets.emplace_back(rect.center, std::min(rect_size.height, rect_size.width) * .5f);
			// 在弹丸掩码上标记该轮廓
			// cv::drawContours(last_bullet_mask_, candidate_contours_, i, 255, cv::FILLED);
		}
	}
}

bool DetectBullet::testIsBullet(const std::vector<cv::Point>& contour)
{
	std::vector<cv::Point> sorted_contour = contour;
	sortPoints(sorted_contour);
	tme_bright -= (double)clock() / CLOCKS_PER_SEC;

	bool flag = false;
	// 按x分组，遍历每一列的y区间
	for (uint32_t i = 0, j = 0; i < sorted_contour.size() && !flag; i = j) {
		int x = sorted_contour[i].x;
		while (j < sorted_contour.size() && x == sorted_contour[j].x)
			++j;
		// 遍历该x下所有y
		for (int y = sorted_contour[i].y; y <= sorted_contour[j - 1].y && !flag; ++y) {
			// 判断该像素是否为弹丸颜色
			const cv::Vec3b& hsv_col = cur_hsv_.at<cv::Vec3b>(cv::Point(x, y));
			if (hsv_col[2] > 50 && std::fabs((int)hsv_col[0] - 50) < 10 + .5 * std::exp((hsv_col[1] + hsv_col[2]) / 100.0)) {
				flag = true;
			}
		}
	}
	tme_bright += (double)clock() / CLOCKS_PER_SEC;
	return flag;
}

void DetectBullet::sortPoints(std::vector<cv::Point>& vec)
{
	// 计时开始
	tme_sort_points_ -= (double)clock() / CLOCKS_PER_SEC;

  // 如果 sort_pts 没有初始化，那么先初始化
	if (sort_pts_.empty() && !cur_frame_.empty()) {
		sort_pts_ = std::vector<std::vector<uint32_t>>(cur_frame_.cols);
	}

	uint32_t mn_x = cur_frame_.cols, mx_x = 0;
	// 遍历所有点，将y值按x分组存入sort_pts_
	for (const cv::Point& pt : vec) {
		uint32_t x = pt.x, y = pt.y;
		sort_pts_[x].emplace_back(y);
		if (x < mn_x) mn_x = x; 
		if (x > mx_x) mx_x = x; 
	}
	// 清空原vec，准备重组
	std::vector<cv::Point>().swap(vec);

	// 按x从小到大遍历
	for (uint32_t x = mn_x; x <= mx_x; ++x) {
		std::vector<uint32_t>& vc_x = sort_pts_[x];
		// 如果y数量较多，直接用std::sort排序
		if (vc_x.size() > 10) {
			std::sort(vc_x.begin(), vc_x.end());
		} else {
			// 否则用冒泡排序（小数据更快）
			for (uint32_t i = 0; i < vc_x.size(); ++i) {
				for (uint32_t j = 0; j < i; ++j) {
					if (vc_x[j] > vc_x[i])
						std::swap(vc_x[i], vc_x[j]);
				}
			}
		}
		// 按排序后的y值重组点，加入vec
		for (uint32_t y : vc_x) {
			vec.emplace_back(x, y);
		}
		// 清空当前x的辅助容器，节省内存
		std::vector<uint32_t>().swap(vc_x);
	}

	// 计时结束
	tme_sort_points_ += (double)clock() / CLOCKS_PER_SEC;
}

void DetectBullet::drawResults(cv::Mat& img)
{
	// TODO: 结果绘制
	// 有可能在detect_node.cpp中绘制
}

// DoFrameDifference在DetectBullet构造时初始化参数
DoFrameDifference::DoFrameDifference() {}

cv::Mat DoFrameDifference::getDiff(
	const cv::Mat& cur_hsv, const cv::Mat& last_hsv_reproj, const cv::Mat& ref_mask, const cv::Mat& last_bullet_mask)
{
	tme_ -= (double)clock() / CLOCKS_PER_SEC;
	cv::Mat res = cv::Mat::zeros(cur_hsv.rows, cur_hsv.cols, CV_8U);
	for (int y = 0; y < cur_hsv.rows; y += Diff_Step) {
		for (int x = 0; x < cur_hsv.cols; x += Diff_Step) {
			cv::Point p(x, y);
			
			//如果参考掩码或上一帧子弹掩码无效则跳过
			if (!ref_mask.at<uint8_t>(p) || (!last_bullet_mask.empty() && last_bullet_mask.at<uint8_t>(p)))
				continue;
			
			const cv::Vec3b& c1 = cur_hsv.at<cv::Vec3b>(p);
			bool flag = true;
			
			// 这里dy/dx循环其实只跑一次（等价于只比较当前点），可扩展为邻域比较
			for (int dy = 0; dy < 1 && flag; ++dy) {
				int ty = y + dy;
				if (ty < 0 || ty >= cur_hsv.rows)
					continue;
				for (int dx = 0; dx < 1 && flag; ++dx) {
					int tx = x + dx;
					if (tx < 0 || tx >= cur_hsv.cols)
						continue;

					// 上一帧重投影后的该点HSV值
					const cv::Vec3b& c2 = last_hsv_reproj.at<cv::Vec3b>(cv::Point(tx, ty));
					// 计算加权HSV通道差异
					uint8_t tmp = (Weights[0] * abs(c1[0] - c2[0]) + Weights[1] * abs(c1[1] - c2[1])
								   + Weights[2] * abs(c1[2] - c2[2]))
						/ (Weights[0] + Weights[1] + Weights[2]);
					// 如果差异小于阈值，说明该点变化不显著，标记flag为false
					if (tmp < Diff_Threshold)
						flag = false;
				}
			}

			// 如果flag为true，说明该点帧差显著，标记为255（白），否则为0
			res.at<uint8_t>(p) = flag ? 255 : 0;
		}
	}

	// 对结果掩码做膨胀操作，消除小空洞、连接邻近区域
	cv::dilate(res, res, kernel1_);
	// 将上一帧子弹掩码也加入结果中，确保已识别子弹区域不被遗漏
	if (!last_bullet_mask.empty()) {
		res |= last_bullet_mask;
	}
	tme_ += (double)clock() / CLOCKS_PER_SEC;
	return res;
}

}  // namespace rm_auto_aim

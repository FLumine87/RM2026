#include "armor_detector/detect_bullet.hpp"
#include <opencv2/imgproc.hpp>
#include <cmath>

namespace rm_auto_aim{

namespace {
constexpr int DIFF_THRESHOLD = 30;
constexpr int KERNEL_SIZE = 4;
constexpr int MIN_BRIGHT_V = 50;
constexpr int BULLET_H = 50;
constexpr int BULLET_H_RANGE = 10;
constexpr int DIFF_STEP = 5; // 帧差步长
const float DIFF_WEIGHTS[3] = {4, 4, 2}; // 帧差权重
}


// 支持参数灵活配置的构造
DetectBullet::DetectBullet(int binary_thres, float min_area, float max_area, float min_ratio, float max_ratio, bool bullet_visual_debug)
    : binary_thres(binary_thres), min_area(min_area), max_area(max_area), min_ratio(min_ratio), max_ratio(max_ratio), bullet_visual_debug(bullet_visual_debug)
{
    diff_step_ = DIFF_STEP;
    diff_weights_[0] = DIFF_WEIGHTS[0];
    diff_weights_[1] = DIFF_WEIGHTS[1];
    diff_weights_[2] = DIFF_WEIGHTS[2];
    kernel_size_ = KERNEL_SIZE;
}

void DetectBullet::setReproj(const DoReproj& do_reproj)
{
    do_reproj_ = do_reproj;
    has_reproj_ = true;
}


// 寻找弹丸轮廓
std::vector<std::vector<cv::Point>> DetectBullet::findBulletContours(const cv::Mat& mask)
{
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    return contours;
}

// 判断轮廓是否为弹丸
bool DetectBullet::isBulletContour(const std::vector<cv::Point>& contour, const cv::Mat& hsv)
{
    if (contour.size() < 5) return false;
    float area = cv::contourArea(contour);
    if (area < min_area || area > max_area) return false;

    cv::RotatedRect ellipse = cv::fitEllipse(contour);
    float ratio = ellipse.size.width / ellipse.size.height;
    if (ratio < min_ratio || ratio > max_ratio) return false;

    // 逐行亮度判定
    return contourBrightestCheck(contour, hsv);
}

// 帧差法+颜色判定，生成弹丸掩码，支持参考掩码
cv::Mat DetectBullet::getBulletMask(const cv::Mat& cur_hsv, const cv::Mat& last_hsv_reproj, const cv::Mat& ref_mask, const cv::Mat& last_bullet_mask) {
    cv::Mat diff_mask = cv::Mat::zeros(cur_hsv.size(), CV_8U);
    for (int y = 0; y < cur_hsv.rows; y += diff_step_) {
        for (int x = 0; x < cur_hsv.cols; x += diff_step_) {
            if (!ref_mask.empty() && !ref_mask.at<uchar>(y, x)) continue;
            if (!last_bullet_mask.empty() && last_bullet_mask.at<uchar>(y, x)) continue;
            const cv::Vec3b& c1 = cur_hsv.at<cv::Vec3b>(y, x);
            const cv::Vec3b& c2 = last_hsv_reproj.at<cv::Vec3b>(y, x);
            float diff = (diff_weights_[0] * std::abs(c1[0] - c2[0]) + diff_weights_[1] * std::abs(c1[1] - c2[1]) + diff_weights_[2] * std::abs(c1[2] - c2[2]))
                / (diff_weights_[0] + diff_weights_[1] + diff_weights_[2]);
            if (diff > DIFF_THRESHOLD && c1[2] > MIN_BRIGHT_V && std::abs((int)c1[0] - BULLET_H) < BULLET_H_RANGE) {
                diff_mask.at<uchar>(y, x) = 255;
            }
        }
    }
    cv::dilate(diff_mask, diff_mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernel_size_, kernel_size_)));
    return diff_mask;
}

// 统计弹丸结果
std::vector<Bullet> DetectBullet::extractBullets(const std::vector<std::vector<cv::Point>>& contours, const cv::Mat& cur_hsv)
{
    std::vector<Bullet> bullets;
    for (const auto& contour : contours) {
        if (isBulletContour(contour, cur_hsv)) {
            cv::Point2f center;
            float radius;
            cv::minEnclosingCircle(contour, center, radius);
            bullets.emplace_back(center, radius);
        }
    }
    return bullets;
}

// 颜色掩码生成
cv::Mat getColorMask(const cv::Mat& hsv) {
    cv::Mat mask;
    cv::inRange(hsv, cv::Scalar(40, 40, 40), cv::Scalar(90, 255, 255), mask); // 可调参数
    return mask;
}

// 亮度掩码生成
cv::Mat getBrightnessMask(const cv::Mat& hsv) {
    cv::Mat mask;
    cv::inRange(hsv, cv::Scalar(0, 25, 50), cv::Scalar(255, 255, 255), mask); // 可调参数
    return mask;
}

// 轮廓查找方式可选
std::vector<std::vector<cv::Point>> findBulletContoursWithHierarchy(const cv::Mat& mask, int mode = cv::CHAIN_APPROX_NONE) {
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, mode);
    int nested_count = 0;
    for (const auto& h : hierarchy) {
        if (h[3] != -1) nested_count++;
    }
    if (nested_count > 0) {
        std::cerr << "[detect_bullet] Nested contours detected: " << nested_count << std::endl;
    }
    return contours;
}

// 亮度判定更灵活
bool testIsBulletColor(const cv::Vec3b& hsv_col) {
    return hsv_col[2] > MIN_BRIGHT_V && std::abs((int)hsv_col[0] - BULLET_H) < BULLET_H_RANGE + 0.5 * std::exp((hsv_col[1] + hsv_col[2]) / 100.0);
}

// aimer::aim风格的轮廓点排序与亮度判定
bool DetectBullet::contourBrightestCheck(const std::vector<cv::Point>& contour, const cv::Mat& hsv) {
    if (contour.empty()) return false;
    std::vector<cv::Point> sorted = contour;
    // 按x排序
    std::sort(sorted.begin(), sorted.end(), [](const cv::Point& a, const cv::Point& b) {
        return a.x < b.x || (a.x == b.x && a.y < b.y);
    });
    size_t i = 0;
    while (i < sorted.size()) {
        int x = sorted[i].x;
        size_t j = i;
        while (j < sorted.size() && sorted[j].x == x) ++j;
        for (int y = sorted[i].y; y <= sorted[j-1].y; ++y) {
            cv::Vec3b hsv_col = hsv.at<cv::Vec3b>(y, x);
            if (testIsBulletColor(hsv_col)) return true;
        }
        i = j;
    }
    return false;
}

std::vector<Bullet> DetectBullet::detectWithHistory(const cv::Mat& cur_frame, const cv::Mat& last_frame, const Eigen::Quaterniond& cur_q, const Eigen::Quaterniond& last_q)
{
    std::vector<Bullet> bullets;
    if (!has_reproj_ || cur_frame.empty() || last_frame.empty()) return bullets;

    cv::Mat cur_hsv, last_hsv;
    cv::cvtColor(cur_frame, cur_hsv, cv::COLOR_BGR2HSV);
    cv::cvtColor(last_frame, last_hsv, cv::COLOR_BGR2HSV);

    // 颜色掩码与亮度掩码分离
    cv::Mat color_mask = getColorMask(cur_hsv);
    cv::Mat bright_mask = getBrightnessMask(cur_hsv);
    color_mask &= bright_mask;

    // 重投影
    cv::Mat last_hsv_reproj = do_reproj_.reproject(last_hsv, last_q, cur_q);

    // 帧差法+颜色判定，生成弹丸掩码（参考掩码、上一帧弹丸掩码）
    cv::Mat diff_mask = getBulletMask(cur_hsv, last_hsv_reproj, color_mask, last_bullet_mask_);

    // 颜色掩码与帧差掩码结合
    color_mask &= diff_mask;

    // 轮廓查找（带层级统计与嵌套提示，方式可选）
    auto contours = findBulletContoursWithHierarchy(color_mask, cv::CHAIN_APPROX_NONE);

    // 统计弹丸结果
    bullets = extractBullets(contours, cur_hsv);

    // 结果结构多帧维护
    last_bullet_mask_ = cv::Mat::zeros(cur_hsv.size(), CV_8U);
    last_bullets_ = bullets;
    last_cur_frame_ = cur_frame.clone();
    last_cur_hsv_ = cur_hsv.clone();
    for (const auto& contour : contours) {
        if (isBulletContour(contour, cur_hsv)) {
            cv::drawContours(last_bullet_mask_, std::vector<std::vector<cv::Point>>{contour}, -1, 255, cv::FILLED);
        }
    }

    return bullets;
}

void DetectBullet::drawResults(cv::Mat& img)
{
    if (!bullet_visual_debug) return;
    for (const auto& bullet : bullets) {
        cv::circle(img, bullet.center, bullet.radius, cv::Scalar(0, 0, 255), 2);
        cv::circle(img, bullet.center, 2, cv::Scalar(255, 255, 255), -1);
    }
}

} // namespace rm_auto_aim
#ifndef BULLET_DETECTOR__DETECTOR_HPP_
#define BULLET_DETECTOR__DETECTOR_HPP_

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
    DetectBullet(int binary_thres, float min_area, float max_area, float min_ratio, float max_ratio, bool bullet_visual_debug);

    void setReproj(const DoReproj& do_reproj);

    // 主流程：帧差法+重投影+颜色亮度判定+上一帧弹丸掩码
    std::vector<Bullet> detectWithHistory(const cv::Mat& cur_frame, const cv::Mat& last_frame, const Eigen::Quaterniond& cur_q, const Eigen::Quaterniond& last_q);

    void drawResults(cv::Mat& img);

    int binary_thres;
    float min_area, max_area;
    float min_ratio, max_ratio;
    bool bullet_visual_debug;

    // 性能统计
    double tme_total = 0, tme_diff = 0, tme_contour = 0, tme_bright = 0;

private:
    // 颜色掩码生成
    cv::Mat getColorMask(const cv::Mat& hsv);
    // 亮度掩码生成
    cv::Mat getBrightnessMask(const cv::Mat& hsv);
    // 轮廓查找（带层级统计与嵌套提示，方式可选）
    std::vector<std::vector<cv::Point>> findBulletContoursWithHierarchy(const cv::Mat& mask, int mode = cv::CHAIN_APPROX_NONE);
    // 亮度判定更灵活
    bool testIsBulletColor(const cv::Vec3b& hsv_col);
    // 轮廓点排序与逐行亮度判定
    bool contourBrightestCheck(const std::vector<cv::Point>& contour, const cv::Mat& hsv);
    // 寻找弹丸轮廓
    std::vector<std::vector<cv::Point>> findBulletContours(const cv::Mat& mask);
    // 判断轮廓是否为弹丸
    bool isBulletContour(const std::vector<cv::Point>& contour, const cv::Mat& hsv);
    // 帧差法+颜色判定，生成弹丸掩码
    cv::Mat getBulletMask(const cv::Mat& cur_hsv, const cv::Mat& last_hsv_reproj);
    // 统计弹丸结果
    std::vector<Bullet> extractBullets(const std::vector<std::vector<cv::Point>>& contours, const cv::Mat& cur_hsv);

    // 上一帧弹丸掩码
    cv::Mat last_bullet_mask_;
    // 上一帧弹丸结果
    std::vector<Bullet> last_bullets_;
    // 上一帧原图缓存
    cv::Mat last_cur_frame_;
    // 上一帧HSV缓存
    cv::Mat last_cur_hsv_;

    DoReproj do_reproj_;
    bool has_reproj_ = false;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__DETECT_BULLET_HPP_
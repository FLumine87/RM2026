#ifndef BULLET_DETECTOR__DETECTOR_HPP_
#define BULLET_DETECTOR__DETECTOR_HPP_

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>

// STD
#include <cmath>
#include <string>
#include <vector>

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
    DetectBullet(int binary_thres = 180);

    std::vector<Bullet> detect(const cv::Mat& input);

    cv::Mat binary_img; // 二值化结果
    std::vector<Bullet> bullets; // 检测到的弹丸

    void drawResults(cv::Mat& img);

    int binary_thres;

private:
    std::vector<std::vector<cv::Point>> findBulletContours(const cv::Mat& binary_img);
    bool isBulletContour(const std::vector<cv::Point>& contour);
};

}  // namespace rm_auto_aim

#endif  // BULLET_DETECTOR__DETECTOR_HPP_
// OpenCV2
#include <opencv2/core.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

#include "armor_detector/detect_bullet.hpp"

namespace rm_auto_aim
{

DetectBullet::DetectBullet(int binary_thres)
    : binary_thres(binary_thres)
{
}

std::vector<std::vector<cv::Point>> DetectBullet::findBulletContours(const cv::Mat& binary_img)
{
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(binary_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    return contours;
}

bool DetectBullet::isBulletContour(const std::vector<cv::Point>& contour)
{
    // 基本判断：面积、圆度
    if (contour.size() < 5) return false;
    float area = cv::contourArea(contour);
    if (area < 10 || area > 500) return false; // 过滤太小/太大的区域

    cv::RotatedRect ellipse = cv::fitEllipse(contour);
    float ratio = ellipse.size.width / ellipse.size.height;
    if (ratio < 0.7 || ratio > 1.3) return false; // 近似圆形

    return true;
}

std::vector<Bullet> DetectBullet::detect(const cv::Mat& input)
{
    // 灰度化
    cv::Mat gray_img;
    if (input.channels() == 3)
        cv::cvtColor(input, gray_img, cv::COLOR_BGR2GRAY);
    else
        gray_img = input.clone();

    // 二值化
    cv::threshold(gray_img, binary_img, binary_thres, 255, cv::THRESH_BINARY);

    // 轮廓查找
    auto contours = findBulletContours(binary_img);

    bullets.clear();
    for (const auto& contour : contours)
    {
        if (isBulletContour(contour))
        {
            cv::Point2f center;
            float radius;
            cv::minEnclosingCircle(contour, center, radius);
            bullets.emplace_back(center, radius);
        }
    }
    return bullets;
}

void DetectBullet::drawResults(cv::Mat& img)
{
    for (const auto& bullet : bullets)
    {
        cv::circle(img, bullet.center, bullet.radius, cv::Scalar(0, 0, 255), 2);
        cv::circle(img, bullet.center, 2, cv::Scalar(255, 255, 255), -1);
    }
}

} // namespace rm_auto_aim
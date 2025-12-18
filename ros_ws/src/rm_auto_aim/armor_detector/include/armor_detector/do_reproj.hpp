#ifndef ARMOR_DETECTOR__DO_REPROJ_HPP_
#define ARMOR_DETECTOR__DO_REPROJ_HPP_

#include <opencv2/core.hpp>
#include <Eigen/Dense>

namespace rm_auto_aim
{

class DoReproj
{
public:
    using Quat = Eigen::Quaterniond;

    DoReproj(const cv::Mat& cam, const cv::Mat& imu);

    cv::Mat getTransformMat(const Quat& q1, const Quat& q2) const;
    cv::Mat reproject(const cv::Mat& src, const Quat& q1, const Quat& q2) const;

private:
    Eigen::Matrix4d cam_mat_;
    Eigen::Matrix3d imu_mat_;//名称需要修改

    Eigen::Matrix4d quatToTransMat(const Quat& q) const;
};

}  // namespace rm_auto_aim

#endif  // BULLET_DETECTOR__DO_REPROJ_HPP_
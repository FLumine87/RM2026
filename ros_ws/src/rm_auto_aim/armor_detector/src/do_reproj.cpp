#include "armor_detector/do_reproj.hpp"
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>

namespace rm_auto_aim
{

DoReproj::DoReproj(const cv::Mat& cam, const cv::Mat& imu)
{
    cam_mat_ = Eigen::Matrix4d::Zero();
    Eigen::Matrix<double, 3, 4> mat;
    cv::cv2eigen(cam, mat);
    cam_mat_.block<3, 4>(0, 0) = mat;
    cam_mat_(3, 3) = 1;
    cv::cv2eigen(imu, imu_mat_);
}

Eigen::Matrix4d DoReproj::quatToTransMat(const Quat& q) const
{
    Eigen::Matrix4d res = Eigen::Matrix4d::Zero();
    res.block<3, 3>(0, 0) = imu_mat_ * q.matrix().inverse();
    res(3, 3) = 1;
    return res;
}

cv::Mat DoReproj::getTransformMat(const Quat& q1, const Quat& q2) const
{
    Eigen::Matrix4d mat = cam_mat_ * quatToTransMat(q2) * (cam_mat_ * quatToTransMat(q1)).inverse();
    cv::Mat cv_mat;
    cv::eigen2cv(mat.block<3, 3>(0, 0), cv_mat);
    return cv_mat;
}

cv::Mat DoReproj::reproject(const cv::Mat& src, const Quat& q1, const Quat& q2) const
{
    cv::Mat trans_mat = getTransformMat(q1, q2);
    cv::Mat res;
    cv::warpPerspective(src, res, trans_mat, src.size());
    return res;
}

} // namespace rm_auto_aim
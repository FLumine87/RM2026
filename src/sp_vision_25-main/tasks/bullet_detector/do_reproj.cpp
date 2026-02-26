#ifndef __DO_REPROJ_CPP__
#define __DO_REPROJ_CPP__

#include "do_reproj.hpp"

#include <opencv2/core/eigen.hpp>

namespace aimer::aim {

DoReproj::DoReproj() {}

void DoReproj::init(const cv::Mat& cam, const cv::Mat& imu) {
    this->cam = Eigen::Matrix4d();
    
    // 处理相机矩阵
    if (cam.rows == 3 && cam.cols == 3) {
        // 如果是 3x3 相机内参矩阵，扩展为 4x4 矩阵
        Eigen::Matrix3d mat_3x3;
        cv::cv2eigen(cam, mat_3x3);
        this->cam.block<3, 3>(0, 0) = mat_3x3;
        this->cam.block<3, 1>(0, 3) = Eigen::Vector3d::Zero();
        this->cam(3, 3) = 1;
    } else if (cam.rows == 3 && cam.cols == 4) {
        // 如果是 3x4 投影矩阵，直接使用
        Eigen::Matrix<double, 3, 4> mat_3x4;
        cv::cv2eigen(cam, mat_3x4);
        this->cam.block<3, 4>(0, 0) = mat_3x4;
        this->cam(3, 3) = 1;
    }

    // 处理 IMU 旋转矩阵
    if (imu.rows == 3 && imu.cols == 3) {
        cv::cv2eigen(imu, this->imu);
    }
}

DoReproj::DoReproj(const cv::Mat& cam, const cv::Mat& imu) {
    this->init(cam, imu);
}

Eigen::Matrix4d DoReproj::from_q_get_trans_mat(const DoReproj::Quat& q) {
    Eigen::Matrix4d res = Eigen::Matrix4d::Zero(4, 4);
    res.block<3, 3>(0, 0) = this->imu * q.matrix().inverse();
    res(3, 3) = 1;
    return res;
}

Eigen::Matrix3d DoReproj::get_fr_trans_mat(const DoReproj::Quat& q1, const DoReproj::Quat& q2) {
    Eigen::Matrix4d mat = this->cam * this->from_q_get_trans_mat(q2)
        * (this->cam * this->from_q_get_trans_mat(q1)).inverse();
    return mat.block<3, 3>(0, 0);
}

cv::Mat DoReproj::reproj(const cv::Mat& src, const DoReproj::Quat& q1, const DoReproj::Quat& q2) {
    // get transform matrix
    Eigen::Matrix3d mat = this->get_fr_trans_mat(q1, q2);
    cv::Mat cv_mat;
    cv::eigen2cv(mat, cv_mat);

    cv::Mat res;
    cv::warpPerspective(src, res, cv_mat, src.size());
    return res.clone();
}
}  // namespace aimer::aim
#endif

#include "coord_converter.hpp"
#include "aimer_math.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

namespace aimer {

CoordConverter::CoordConverter(const std::string& config_path): config_path(config_path) {
    this->load_params(config_path);
}

void CoordConverter::load_params(const std::string& config_path) {
    if (config_path.empty()) {
        return;
    }

    try {
        YAML::Node config = YAML::LoadFile(config_path);

        if (config["camera_matrix"]) {
            auto camera_matrix = config["camera_matrix"].as<std::vector<double>>();
            // 确保相机内参矩阵是 3x3 的 CV_64F 类型
            this->f_cv_mat = cv::Mat::zeros(3, 3, CV_64F);
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    if (i * 3 + j < camera_matrix.size()) {
                        this->f_cv_mat.at<double>(i, j) = camera_matrix[i * 3 + j];
                    }
                }
            }
            // 同时更新 Eigen 矩阵
            cv::cv2eigen(this->f_cv_mat, this->f_mat);
        }

        if (config["distortion_coefficients"]) {
            auto dist_coeffs = config["distortion_coefficients"].as<std::vector<double>>();
            // 只使用前 4 个畸变系数，因为 cv::fisheye::undistortPoints 只接受 4 个
            this->c_cv_mat = cv::Mat::zeros(1, 4, CV_64F);
            for (int i = 0; i < std::min(4, static_cast<int>(dist_coeffs.size())); ++i) {
                this->c_cv_mat.at<double>(0, i) = dist_coeffs[i];
            }
        }

        if (config["rotation_ic"]) {
            auto rot_ic = config["rotation_ic"].as<std::vector<double>>();
            this->rot_ic_sup << rot_ic[0], rot_ic[1], rot_ic[2],
                              rot_ic[3], rot_ic[4], rot_ic[5],
                              rot_ic[6], rot_ic[7], rot_ic[8];
            cv::eigen2cv(this->rot_ic_sup, this->rot_ic_sup_cv_mat);
        }
    } catch (const std::exception& e) {
    }
}

void CoordConverter::update(const cv::Mat& img, const Eigen::Quaterniond& q, const double& timestamp) {
    this->img = img;
    this->q = q;
    this->img_t = timestamp;
    this->frame++;
}

Eigen::Vector3d CoordConverter::pc_to_pi(const Eigen::Vector3d& pc) const {
    return this->rot_ic_sup * pc;
}

Eigen::Vector3d CoordConverter::pi_to_pc(const Eigen::Vector3d& pi) const {
    return this->rot_ic_sup.transpose() * pi;
}

cv::Point2f CoordConverter::pc_to_pu(const Eigen::Vector3d& pc) const {
    Eigen::Vector3d uv = this->f_mat * pc;
    return cv::Point2f(uv(0, 0) / uv(2, 0), uv(1, 0) / uv(2, 0));
}

Eigen::Vector3d CoordConverter::pu_to_pc_norm(const cv::Point2f& pu) const {
    Eigen::Vector3d uv(pu.x, pu.y, 1.0);
    return this->f_mat.inverse() * uv;
}

Eigen::Vector3d CoordConverter::pu_to_pc(const cv::Point2f& pu, double distance) const {
    Eigen::Vector3d pc_norm = this->pu_to_pc_norm(pu);
    return pc_norm * distance;
}

aimer::math::YpdCoord CoordConverter::pu_to_yp_c(const cv::Point2f& pu) const {
    Eigen::Vector3d pc = this->pu_to_pc_norm(pu);
    return aimer::math::camera_xyz_to_ypd(pc);
}

cv::Point2f CoordConverter::pu_to_pd(const cv::Point2f& pu) const {
    // 直接返回输入点，避免使用 cv::fisheye::distortPoints
    // 因为该函数对输入参数有严格的要求
    return pu;
}

cv::Point2f CoordConverter::pd_to_pu(const cv::Point2f& pd) const {
    // 直接返回输入点，避免使用 cv::fisheye::undistortPoints
    // 因为该函数对输入参数有严格的要求
    return pd;
}

cv::Point2f CoordConverter::pi_to_pu(const Eigen::Vector3d& pi) const {
    Eigen::Vector3d pc = this->pi_to_pc(pi);
    return this->pc_to_pu(pc);
}

cv::Point2f CoordConverter::pi_to_pd(const Eigen::Vector3d& pi) const {
    cv::Point2f pu = this->pi_to_pu(pi);
    return this->pu_to_pd(pu);
}

aimer::math::YpdCoord CoordConverter::pd_to_yp_c(const cv::Point2f& pd) const {
    cv::Point2f pu = this->pd_to_pu(pd);
    return this->pu_to_yp_c(pu);
}

Eigen::Vector3d CoordConverter::xyz_i_camera_to_xyz_i_barrel(const Eigen::Vector3d& xyz_i_camera) const {
    return xyz_i_camera;
}

Eigen::Vector3d CoordConverter::xyz_i_barrel_to_xyz_i_camera(const Eigen::Vector3d& xyz_i_barrel) const {
    return xyz_i_barrel;
}

} // namespace aimer
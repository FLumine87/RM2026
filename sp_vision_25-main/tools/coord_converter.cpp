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
            this->f_mat << camera_matrix[0], camera_matrix[1], camera_matrix[2],
                          camera_matrix[3], camera_matrix[4], camera_matrix[5],
                          camera_matrix[6], camera_matrix[7], camera_matrix[8];
            cv::eigen2cv(this->f_mat, this->f_cv_mat);
        }

        if (config["distortion_coefficients"]) {
            auto dist_coeffs = config["distortion_coefficients"].as<std::vector<double>>();
            for (int i = 0; i < 5; ++i) {
                this->c_mat(0, i) = dist_coeffs[i];
            }
            cv::eigen2cv(this->c_mat, this->c_cv_mat);
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
    std::vector<cv::Point2f> src = {pu};
    std::vector<cv::Point2f> dst;
    cv::fisheye::distortPoints(src, dst, this->f_cv_mat, this->c_cv_mat);
    return dst[0];
}

cv::Point2f CoordConverter::pd_to_pu(const cv::Point2f& pd) const {
    std::vector<cv::Point2f> src = {pd};
    std::vector<cv::Point2f> dst;
    cv::fisheye::undistortPoints(src, dst, this->f_cv_mat, this->c_cv_mat);
    return dst[0];
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
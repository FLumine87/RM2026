#ifndef AIMER_COORD_CONVERTER_HPP
#define AIMER_COORD_CONVERTER_HPP

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <string>
#include "yaml-cpp/yaml.h"
#include "aimer_math.hpp"

namespace aimer {

struct ShootParam {
    double v0 = 0.;
    double aim_angle = 0.;
    Eigen::Vector3d aim_xyz_i_barrel = Eigen::Vector3d::Zero();
    Eigen::Vector3d target_xyz_i_camera = Eigen::Vector3d::Zero();
};

struct AimInfo {
    aimer::math::YpdCoord ypd;
    aimer::math::YpdCoord ypd_v;
    aimer::ShootParam shoot_param;
    int shoot = 0;
    int info = 0;

    AimInfo() = default;

    static const AimInfo idle() {
        return AimInfo();
    }
};

class CoordConverter {
private:
    Eigen::Matrix3d rot_ic_sup;
    Eigen::Matrix3d f_mat;
    Eigen::Matrix<double, 1, 5> c_mat;
    cv::Mat rot_ic_sup_cv_mat;
    cv::Mat f_cv_mat;
    cv::Mat c_cv_mat;
    Eigen::Quaterniond q;
    cv::Mat img;
    int frame = 0;
    double img_t = 0.;
    std::string config_path;

public:
    CoordConverter(const std::string& config_path = "");
    ~CoordConverter() = default;

    const cv::Mat& get_img_ref() const { return this->img; }
    int get_frame() const { return this->frame; }
    double get_img_t() const { return this->img_t; }
    void set_img_t(double t) { this->img_t = t; }

    Eigen::Quaterniond get_q() const { return this->q; }

    const cv::Mat& get_f_cv_mat_ref() const { return this->f_cv_mat; }
    const cv::Mat& get_c_cv_mat_ref() const { return this->c_cv_mat; }
    const cv::Mat& get_rot_ic_sup_cv_mat_ref() const { return this->rot_ic_sup_cv_mat; }

    void update(const cv::Mat& img, const Eigen::Quaterniond& q, const double& timestamp);

    Eigen::Vector3d pc_to_pi(const Eigen::Vector3d& pc) const;
    Eigen::Vector3d pi_to_pc(const Eigen::Vector3d& pi) const;

    cv::Point2f pc_to_pu(const Eigen::Vector3d& pc) const;
    Eigen::Vector3d pu_to_pc_norm(const cv::Point2f& pu) const;
    Eigen::Vector3d pu_to_pc(const cv::Point2f& pu, double distance = 1.0) const;
    aimer::math::YpdCoord pu_to_yp_c(const cv::Point2f& pu) const;

    cv::Point2f pu_to_pd(const cv::Point2f& pu) const;
    cv::Point2f pd_to_pu(const cv::Point2f& pd) const;

    cv::Point2f pi_to_pu(const Eigen::Vector3d& pi) const;
    cv::Point2f pi_to_pd(const Eigen::Vector3d& pi) const;
    aimer::math::YpdCoord pd_to_yp_c(const cv::Point2f& pd) const;

    Eigen::Vector3d xyz_i_camera_to_xyz_i_barrel(const Eigen::Vector3d& xyz_i_camera) const;
    Eigen::Vector3d xyz_i_barrel_to_xyz_i_camera(const Eigen::Vector3d& xyz_i_barrel) const;

    double get_img_to_predict_latency() const { return 0.03; }
    double get_predict_to_send_latency() const { return 0.01; }
    double get_send_to_control_latency() const { return 0.01; }
    double get_control_to_fire_latency() const { return 0.1; }

private:
    void load_params(const std::string& config_path);
};

} // namespace aimer

#endif /* AIMER_COORD_CONVERTER_HPP */
#ifndef AIMER_MATH_HPP
#define AIMER_MATH_HPP

#include <Eigen/Dense>
#include <cmath>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <utility>
#include <vector>

namespace aimer::math {

struct YpdCoord {
    YpdCoord() = default;
    YpdCoord(const double& y, const double& p, const double& d): yaw(y), pitch(p), dis(d) {}
    double yaw = 0.;
    double pitch = 0.;
    double dis = 0.;

    math::YpdCoord operator+(const math::YpdCoord& ypd) {
        return math::YpdCoord(this->yaw + ypd.yaw, this->pitch + ypd.pitch, this->dis + ypd.dis);
    }

    math::YpdCoord& operator+=(const math::YpdCoord& ypd) {
        *this = *this + ypd;
        return *this;
    }
};

template<typename T>
class Circle {
public:
    Circle(const cv::Point_<T>& center, const T& r): center(center), r(r) {}
    cv::Point_<T> center;
    T r;
};

using CircleF = Circle<float>;

inline bool is_nan_or_inf(const double& x) {
    return std::isnan(x) || std::isinf(x);
}

template<typename T>
T sq(const T& x) {
    return x * x;
}

inline double sigmoid(const double& x) {
    return 1. / (1. + std::exp(-x));
}

template<typename T>
T clamp_default(const T& x, const T& lower, const T& upper, const T& default_value) {
    if (x < lower || x > upper) {
        return default_value;
    }
    return x;
}

inline double get_ratio(const double& x, const double& y) {
    if (x == 0. || y == 0.) {
        return 0.;
    }
    return (x < y) ? x / y : y / x;
}

inline double get_norm(const double& x, const double& y) {
    return std::sqrt(x * x + y * y);
}

inline float get_dis(const cv::Point2f& pt1, const cv::Point2f& pt2) {
    return std::sqrt((pt1.x - pt2.x) * (pt1.x - pt2.x) + (pt1.y - pt2.y) * (pt1.y - pt2.y));
}

inline float get_cross(const cv::Point2f& pt1, const cv::Point2f& pt2, const cv::Point2f& pt3) {
    return (pt2.x - pt1.x) * (pt3.y - pt1.y) - (pt3.x - pt1.x) * (pt2.y - pt1.y);
}

inline float get_area(const cv::Point2f* pts) {
    return std::fabs(math::get_cross(pts[0], pts[1], pts[2]))
        + std::fabs(math::get_cross(pts[0], pts[2], pts[3]));
}

template<class F, class T, class... Ts>
T reduce(F&& func, T x, Ts... xs) {
    if constexpr (sizeof...(Ts) > 0) {
        return func(x, math::reduce(std::forward<F>(func), xs...));
    } else {
        return x;
    }
}

template<class T, class... Ts>
T reduce_min(T x, Ts... xs) {
    return math::reduce([](auto a, auto b) { return std::min(a, b); }, x, xs...);
}

template<class T, class... Ts>
T reduce_max(T x, Ts... xs) {
    return math::reduce([](auto a, auto b) { return std::max(a, b); }, x, xs...);
}

inline cv::Rect2f get_box(const cv::Point2f pts[4]) {
    cv::Rect2f box;
    box.x = math::reduce_min(pts[0].x, pts[1].x, pts[2].x, pts[3].x);
    box.y = math::reduce_min(pts[0].y, pts[1].y, pts[2].y, pts[3].y);
    box.width = math::reduce_max(pts[0].x, pts[1].x, pts[2].x, pts[3].x) - box.x;
    box.height = math::reduce_max(pts[0].y, pts[1].y, pts[2].y, pts[3].y) - box.y;
    return box;
}

inline float get_box_iou(const cv::Point2f pts1[4], const cv::Point2f pts2[4]) {
    cv::Rect2f box1 = math::get_box(pts1);
    cv::Rect2f box2 = math::get_box(pts2);
    if ((box1 & box2).area() == 0.) {
        return 0.f;
    }
    return (box1 & box2).area() / (box1 | box2).area();
}

template<typename T>
double rad_to_deg(const T& x) {
    return T(x) / T(M_PI) * T(180.);
}

template<typename T>
double deg_to_rad(const T& x) {
    return T(x) / T(180.) * T(M_PI);
}

inline double reduced_angle(const double& x) {
    return std::atan2(std::sin(x), std::cos(x));
}

inline double reduced(const double& x, const double& range) {
    double times = range / (2. * M_PI);
    return times * (math::reduced_angle(x / times - M_PI) + M_PI);
}

inline double get_abs_angle(const Eigen::Vector2d& vec1, const Eigen::Vector2d& vec2) {
    if (vec1.norm() == 0. || vec2.norm() == 0.) {
        return 0.;
    }
    return std::acos(vec1.dot(vec2) / (vec1.norm() * vec2.norm()));
}

inline double get_theta(const Eigen::Vector2d& vec) {
    return std::atan2(vec(1, 0), vec(0, 0));
}

inline double get_rotate_angle(const double& a1, const double& a2) {
    return math::reduced_angle(a2 - a1);
}

inline double get_rotate_angle(const Eigen::Vector2d& vec1, const Eigen::Vector2d& vec2) {
    double a1 = math::get_theta(vec1);
    double a2 = math::get_theta(vec2);
    return math::reduced_angle(a2 - a1);
}

inline double min_angle(const double& a1, const double& a2) {
    if (math::get_rotate_angle(a1, a2) > 0.) {
        return a1;
    }
    return a2;
}

inline double max_angle(const double& a1, const double& a2) {
    if (math::get_rotate_angle(a1, a2) > 0.) {
        return a2;
    }
    return a1;
}

inline double
get_weighted_angle(const double& theta1, const double& w1, const double& theta2, const double& w2) {
    double rotate_angle = reduced_angle(theta2 - theta1);
    return math::reduced_angle(theta1 + rotate_angle * (w2 / (w1 + w2)));
}

inline double get_closest(const double& cur, const double& tar, const double& period) {
    double reduced = math::reduced(cur, period);
    double possibles[3] = { reduced - period, reduced, reduced + period };
    double closest = possibles[0];
    for (double possible: possibles) {
        if (std::fabs(tar - possible) < std::fabs(tar - closest)) {
            closest = possible;
        }
    }
    return closest;
}

inline double get_closest_angle(const double& cur, const double& tar) {
    const double delta = reduced_angle(cur - tar);
    return tar + delta;
}

inline double get_termination_dis(const double& dis, const double& angle) {
    return std::fabs(2. * dis * std::sin(math::reduced_angle(angle) / 2.));
}

inline double get_termination_dis(const double& dis, const double& angle1, const double& angle2) {
    return std::fabs(2. * dis * std::sin(math::get_rotate_angle(angle1, angle2) / 2.));
}

inline Eigen::Vector2d rotate(const Eigen::Vector2d& vec, const double& angle) {
    Eigen::Matrix2d mat;
    double sin_angle = std::sin(angle);
    double cos_angle = std::cos(angle);
    mat << cos_angle, -sin_angle, sin_angle, cos_angle;
    return mat * vec;
}

inline math::YpdCoord xyz_to_ypd(const Eigen::Vector3d& xyz) {
    math::YpdCoord ypd;
    ypd.yaw = std::atan2(xyz(1, 0), xyz(0, 0));
    ypd.pitch = std::atan2(xyz(2, 0), std::sqrt(xyz(0, 0) * xyz(0, 0) + xyz(1, 0) * xyz(1, 0)));
    ypd.dis = xyz.norm();
    return ypd;
}

inline Eigen::Vector3d ypd_to_xyz(const math::YpdCoord& ypd) {
    Eigen::Vector3d xyz;
    double length_xy = ypd.dis * std::cos(ypd.pitch);
    xyz(0, 0) = length_xy * std::cos(ypd.yaw);
    xyz(1, 0) = length_xy * std::sin(ypd.yaw);
    xyz(2, 0) = ypd.dis * std::sin(ypd.pitch);
    return xyz;
}

inline math::YpdCoord camera_xyz_to_ypd(const Eigen::Vector3d& xyz) {
    math::YpdCoord res;
    res.yaw = std::atan2(xyz(0, 0), xyz(2, 0));
    res.pitch = std::atan2(xyz(1, 0), xyz(2, 0));
    res.dis = xyz.norm();
    return res;
}

inline Eigen::Vector3d camera_ypd_to_xyz(const math::YpdCoord& ypd) {
    Eigen::Vector3d xyz;
    double t1 = std::tan(ypd.yaw);
    double t2 = std::tan(ypd.pitch);
    xyz(2, 0) = std::sqrt(ypd.dis * ypd.dis / (t1 * t1 + t2 * t2 + 1.));
    xyz(0, 0) = xyz(2, 0) * t1;
    xyz(1, 0) = xyz(2, 0) * t2;
    return xyz;
}

template<typename T, size_t NUM>
Eigen::Matrix<T, NUM, NUM> array_to_diag_mat(const std::array<T, NUM>& arr) {
    Eigen::Matrix<T, NUM, NUM> mat;
    for (size_t i = 0; i < NUM; i++) {
        mat(i, i) = arr[i];
    }
    return mat;
}

template<int N_X>
Eigen::Matrix<double, N_X, N_X> vec_x_to_mat_xx(const std::vector<double>& x_vec) {
    Eigen::Matrix<double, N_X, N_X> xx_mat = Eigen::Matrix<double, N_X, N_X>::Zero();
    for (int i = 0; i < N_X; ++i) {
        xx_mat(i, i) = x_vec[i];
    }
    return xx_mat;
}

template<int N_X>
Eigen::Matrix<double, N_X, 1> vec_x_to_mat_x1(const std::vector<double>& x_vec) {
    Eigen::Matrix<double, N_X, 1> x1_mat = Eigen::Matrix<double, N_X, 1>::Zero();
    for (int i = 0; i < N_X; ++i) {
        x1_mat(i, 0) = x_vec[i];
    }
    return x1_mat;
}

template<typename T>
T get_vec_mean(const std::vector<T>& vec) {
    if (vec.empty()) {
        return T(0);
    }
    T sum = vec[0];
    for (std::size_t i = 1; i < vec.size(); ++i) {
        sum = sum + vec[i];
    }
    return sum * (1. / double(vec.size()));
}

template<typename T>
T get_vec_variance(const std::vector<T>& vec) {
    if (vec.empty()) {
        return T(0);
    }
    T mean = math::get_vec_mean(vec);
    T res = (vec[0] - mean) * (vec[0] - mean);
    for (std::size_t i = 1; i < vec.size(); ++i) {
        res = res + (vec[1] - mean) * (vec[1] - mean);
    }
    return res * (1. / double(vec.size()));
}

class Bisection {
public:
    template<typename ValueT, class Func>
    std::pair<ValueT, ValueT>
    find(ValueT left, ValueT right, Func&& cost_function, const int& iterations_num) {
        for (int i = 0; i < iterations_num; ++i) {
            ValueT mid = (left + right) / ValueT(2);
            if (cost_function(mid) < ValueT(0)) {
                left = mid;
            } else {
                right = mid;
            }
        }
        return std::make_pair((left + right) / ValueT(2), right - left);
    }
};

class Trisection {
public:
    template<typename ValueT, class Func>
    std::pair<ValueT, ValueT>
    find(ValueT left, ValueT right, Func&& cost_function, const int& iterations_num) {
        ValueT phi = (std::sqrt(5.) - 1.) / 2.;
        ValueT ml_cost = 0., mr_cost = 0.;
        int reserved = -1;
        for (int i = 0; i < iterations_num; ++i) {
            ValueT ml = left + (right - left) * (1. - phi);
            ValueT mr = left + (right - left) * phi;
            if (reserved != 0) {
                ml_cost = cost_function(ml);
            }
            if (reserved != 1) {
                mr_cost = cost_function(mr);
            }
            if (ml_cost < mr_cost) {
                right = mr;
                mr_cost = ml_cost;
                reserved = 1;
            } else {
                left = ml;
                ml_cost = mr_cost;
                reserved = 0;
            }
        }
        return std::make_pair((left + right) / ValueT(2), right - left);
    }
};

} // namespace aimer::math
#endif
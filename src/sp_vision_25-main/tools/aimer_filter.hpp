#ifndef AIMER_FILTER_HPP
#define AIMER_FILTER_HPP

#include <Eigen/Dense>
#include <vector>

namespace aimer {

template<int N_X>
class SingleFilter {
public:
    SingleFilter() = default;

    void init_x(const Eigen::Matrix<double, N_X, 1>& x0) {
        this->x = x0;
        this->initialized = true;
    }

    void update(
        const Eigen::Matrix<double, N_X, 1>& z,
        const double& dt,
        const Eigen::Matrix<double, N_X, 1>& u,
        const std::vector<double>& r_vec
    ) {
        if (!this->initialized) {
            this->x = z;
            this->initialized = true;
            return;
        }

        double alpha = 0.1; // 默认滤波系数
        if (!r_vec.empty()) {
            alpha = r_vec[0];
        }

        this->x = alpha * z + (1.0 - alpha) * this->x;
    }

    Eigen::Matrix<double, N_X, 1> predict(const double& dt) const {
        return this->x;
    }

private:
    Eigen::Matrix<double, N_X, 1> x = Eigen::Matrix<double, N_X, 1>::Zero();
    bool initialized = false;
};

} // namespace aimer

#endif /* AIMER_FILTER_HPP */
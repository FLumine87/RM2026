#include "trajectory.hpp"

#include <cmath>

namespace tools
{
constexpr double g = 9.7833;

Trajectory::Trajectory(const double v0, const double d, const double h)
{
  auto a = g * d * d / (2 * v0 * v0);
  auto b = -d;
  auto c = a + h;
  auto delta = b * b - 4 * a * c;

  if (delta < 0) {
    unsolvable = true;
    return;
  }

  unsolvable = false;
  auto tan_pitch_1 = (-b + std::sqrt(delta)) / (2 * a);
  auto tan_pitch_2 = (-b - std::sqrt(delta)) / (2 * a);
  auto pitch_1 = std::atan(tan_pitch_1);
  auto pitch_2 = std::atan(tan_pitch_2);
  auto t_1 = d / (v0 * std::cos(pitch_1));
  auto t_2 = d / (v0 * std::cos(pitch_2));

  pitch = (t_1 < t_2) ? pitch_1 : pitch_2;
  fly_time = (t_1 < t_2) ? t_1 : t_2;
}

Trajectory::Trajectory(const double v0, const double d, const double h, const double k)
{
  auto a = g * d * d / (2 * v0 * v0);
  auto b = -d;
  auto c = a + h;
  auto delta = b * b - 4 * a * c;

  if (delta < 0) {
    unsolvable = true;
    return;
  }

  unsolvable = false;
  auto tan_pitch_1 = (-b + std::sqrt(delta)) / (2 * a);
  auto tan_pitch_2 = (-b - std::sqrt(delta)) / (2 * a);
  auto pitch_1 = std::atan(tan_pitch_1);
  auto pitch_2 = std::atan(tan_pitch_2);
  auto t_1 = d / (v0 * std::cos(pitch_1));
  auto t_2 = d / (v0 * std::cos(pitch_2));

  pitch = (t_1 < t_2) ? pitch_1 : pitch_2;
  fly_time = (t_1 < t_2) ? t_1 : t_2;

  double vx = v0 * std::cos(pitch);
  double vy = v0 * std::sin(pitch);

  auto CalcZWithResistance = [&](double t, double pitch_cur) {
    double vx_cur = v0 * std::cos(pitch_cur);
    double vy_cur = v0 * std::sin(pitch_cur);
    double t_eff = (std::exp(k * d) - 1.0) / (k * vx_cur);
    double z = vy_cur * t_eff - 0.5 * g * t_eff * t_eff;
    return z;
  };

  double z_actual = CalcZWithResistance(fly_time, pitch);
  double dz = h - z_actual;

  for (int i = 0; i < 5; ++i) {
    if (std::abs(dz) < 0.001) {
      break;
    }
    double compensation = std::atan2(dz, d);
    double factor = (i == 0) ? 0.8 : 0.5;
    pitch += compensation * factor;
    z_actual = CalcZWithResistance(fly_time, pitch);
    dz = h - z_actual;
  }
}

}  // namespace tools
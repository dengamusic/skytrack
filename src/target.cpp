#include "skytrack/target.hpp"

#include <cmath>

namespace skytrack {

std::string_view to_string(TargetClass cls) {
  switch (cls) {
    case TargetClass::Drone: return "drone";
    case TargetClass::Aircraft: return "aircraft";
    case TargetClass::Bird: return "bird";
  }
  return "unknown";
}

std::optional<TargetClass> target_class_from_string(std::string_view name) {
  if (name == "drone") return TargetClass::Drone;
  if (name == "aircraft") return TargetClass::Aircraft;
  if (name == "bird") return TargetClass::Bird;
  return std::nullopt;
}

Vec3 Target::navigation_error(double t) const {
  if (!cooperative) return Vec3::Zero();
  // Sum of three sinusoids per axis. Each has variance A^2/2, so three of
  // them with amplitude A = std * sqrt(2/3) have the requested variance.
  static constexpr std::array<double, 3> kPeriods{41.0, 97.0, 233.0};
  Vec3 error = Vec3::Zero();
  for (int axis = 0; axis < 3; ++axis) {
    for (int i = 0; i < 3; ++i) {
      const auto k = static_cast<std::size_t>(axis * 3 + i);
      error[axis] += std::sin(2.0 * geo::kPi * t / kPeriods[static_cast<std::size_t>(i)] +
                              cooperative->error_phase[k]);
    }
  }
  const double amplitude = std::sqrt(2.0 / 3.0);
  error.x() *= amplitude * cooperative->horizontal_std_m;
  error.y() *= amplitude * cooperative->horizontal_std_m;
  error.z() *= amplitude * cooperative->vertical_std_m;
  return error;
}

}  // namespace skytrack

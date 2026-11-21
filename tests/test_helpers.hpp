#pragma once

#include <cmath>
#include <string>

#include "skytrack/sensor.hpp"
#include "skytrack/target.hpp"

namespace skytrack::test {

/// A target flying in a straight line (or standing still) for a long time.
inline Target straight_target(int id, const Vec3& position, const Vec3& velocity = Vec3::Zero(),
                              TargetClass cls = TargetClass::Drone, double rcs_dbsm = 0.0, double size_m = 1.0) {
  return Target{
      .id = id,
      .name = "target-" + std::to_string(id),
      .cls = cls,
      .trajectory = Trajectory(0.0, 1.0e6, position, velocity, {}),
      .rcs_dbsm = rcs_dbsm,
      .size_m = size_m,
      .rf_signature = "",
      .cooperative = std::nullopt,
  };
}

inline SensorCommon common(const std::string& id, double rate_hz = 1.0) {
  SensorCommon c;
  c.id = id;
  c.rate_hz = rate_hz;
  return c;
}

/// Unit vector toward a compass bearing and elevation, in ENU.
inline Vec3 direction(double bearing_rad, double elevation_rad) {
  return {std::sin(bearing_rad) * std::cos(elevation_rad), std::cos(bearing_rad) * std::cos(elevation_rad),
          std::sin(elevation_rad)};
}

}  // namespace skytrack::test

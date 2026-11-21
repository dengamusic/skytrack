#pragma once

#include <array>
#include <optional>
#include <string>
#include <string_view>

#include "skytrack/trajectory.hpp"

namespace skytrack {

enum class TargetClass { Drone, Aircraft, Bird };

std::string_view to_string(TargetClass cls);
std::optional<TargetClass> target_class_from_string(std::string_view name);

/// A target that broadcasts its own position (ADS-B for aircraft, Remote ID
/// for drones). The broadcast position comes from the target's own GNSS and
/// carries that GNSS error, so every receiver sees the same error.
struct CooperativeBroadcast {
  std::string protocol;            // "adsb" or "remote_id"
  std::string identity;            // ICAO address, Remote ID serial, ...
  double horizontal_std_m = 5.0;   // 1-sigma GNSS error per horizontal axis
  double vertical_std_m = 10.0;
  std::array<double, 9> error_phase{};  // fixed per target, drawn from the scenario seed
};

struct Target {
  int id = 0;
  std::string name;
  TargetClass cls = TargetClass::Drone;
  Trajectory trajectory;
  double rcs_dbsm = -20.0;   // radar cross section
  double size_m = 0.5;       // largest visible dimension, for cameras
  std::string rf_signature;  // control-link type it emits; empty = silent
  std::optional<CooperativeBroadcast> cooperative;

  /// Error of the target's own navigation solution at time t: a slowly
  /// varying offset (periods of minutes), zero for non-cooperative targets.
  Vec3 navigation_error(double t) const;
};

}  // namespace skytrack

#pragma once

#include <vector>

#include "skytrack/geo.hpp"

namespace skytrack {

using geo::Mat3;
using geo::Vec3;

struct KinematicState {
  Vec3 position = Vec3::Zero();      // ENU, m
  Vec3 velocity = Vec3::Zero();      // ENU, m/s
  Vec3 acceleration = Vec3::Zero();  // ENU, m/s^2
};

enum class MotionModel { ConstantVelocity, ConstantAcceleration, CoordinatedTurn };

struct MotionSegment {
  MotionModel model = MotionModel::ConstantVelocity;
  double duration_s = 0.0;
  Vec3 acceleration = Vec3::Zero();  // ConstantAcceleration only
  double turn_rate_rad_s = 0.0;      // CoordinatedTurn only; positive = left turn seen from above
};

/// A target's path: a chain of motion segments, each solved in closed form.
///
/// The state is exact at any time, so sensors that sample at different
/// instants (a rotating radar, a 15 Hz camera, a delayed ADS-B feed) all see
/// one consistent truth. After the last segment the target keeps its
/// velocity until t_end.
class Trajectory {
 public:
  Trajectory(double t_start, double t_end, const Vec3& position, const Vec3& velocity,
             const std::vector<MotionSegment>& segments);

  double t_start() const { return t_start_; }
  double t_end() const { return t_end_; }
  bool active(double t) const { return t >= t_start_ && t <= t_end_; }

  /// State at time t. Times outside [t_start, t_end] are clamped to it.
  KinematicState state(double t) const;

  /// Propagates a state through one segment for dt seconds.
  static KinematicState propagate(const MotionSegment& segment, const KinematicState& start, double dt);

 private:
  struct Piece {
    double t0;
    MotionSegment segment;
    KinematicState start;
  };

  double t_start_;
  double t_end_;
  std::vector<Piece> pieces_;
};

}  // namespace skytrack

#include "skytrack/trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <stdexcept>

namespace skytrack {

Trajectory::Trajectory(double t_start, double t_end, const Vec3& position, const Vec3& velocity,
                       const std::vector<MotionSegment>& segments)
    : t_start_(t_start), t_end_(t_end) {
  if (!(t_end > t_start)) throw std::invalid_argument("trajectory needs t_end > t_start");
  KinematicState state{position, velocity, Vec3::Zero()};
  double t = t_start;
  for (const MotionSegment& segment : segments) {
    if (!(segment.duration_s > 0.0)) throw std::invalid_argument("segment duration must be positive");
    if (t >= t_end) break;
    pieces_.push_back({t, segment, state});
    state = propagate(segment, state, segment.duration_s);
    t += segment.duration_s;
  }
  if (t < t_end) pieces_.push_back({t, MotionSegment{}, state});  // coast to the end
}

KinematicState Trajectory::state(double t) const {
  t = std::clamp(t, t_start_, t_end_);
  // Last piece starting at or before t. pieces_[0] starts at t_start_, so it exists.
  const auto it = std::upper_bound(pieces_.begin(), pieces_.end(), t,
                                   [](double value, const Piece& p) { return value < p.t0; });
  const Piece& piece = *std::prev(it);
  return propagate(piece.segment, piece.start, t - piece.t0);
}

KinematicState Trajectory::propagate(const MotionSegment& segment, const KinematicState& start, double dt) {
  KinematicState s;
  switch (segment.model) {
    case MotionModel::ConstantVelocity:
      s.position = start.position + start.velocity * dt;
      s.velocity = start.velocity;
      s.acceleration.setZero();
      break;
    case MotionModel::ConstantAcceleration:
      s.position = start.position + start.velocity * dt + 0.5 * segment.acceleration * dt * dt;
      s.velocity = start.velocity + segment.acceleration * dt;
      s.acceleration = segment.acceleration;
      break;
    case MotionModel::CoordinatedTurn: {
      // Horizontal velocity rotates at a constant rate; vertical velocity is kept.
      const double w = segment.turn_rate_rad_s;
      const double vx = start.velocity.x();
      const double vy = start.velocity.y();
      double sin_term = dt;              // sin(w dt) / w
      double cos_term = 0.5 * w * dt * dt;  // (1 - cos(w dt)) / w
      if (std::abs(w) > 1e-9) {
        sin_term = std::sin(w * dt) / w;
        cos_term = (1.0 - std::cos(w * dt)) / w;
      }
      s.position = start.position + Vec3(sin_term * vx - cos_term * vy,
                                         cos_term * vx + sin_term * vy,
                                         start.velocity.z() * dt);
      const double c = std::cos(w * dt);
      const double sn = std::sin(w * dt);
      s.velocity = Vec3(c * vx - sn * vy, sn * vx + c * vy, start.velocity.z());
      s.acceleration = Vec3(-w * s.velocity.y(), w * s.velocity.x(), 0.0);
      break;
    }
  }
  return s;
}

}  // namespace skytrack

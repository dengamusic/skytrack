#include "skytrack/sensors/camera.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace skytrack {
namespace {

constexpr std::array<TargetClass, 3> kClasses{TargetClass::Drone, TargetClass::Aircraft, TargetClass::Bird};

std::size_t class_index(TargetClass cls) {
  return static_cast<std::size_t>(std::find(kClasses.begin(), kClasses.end(), cls) - kClasses.begin());
}

}  // namespace

Camera::Camera(SensorCommon common, CameraParams params) : Sensor(std::move(common)), params_(std::move(params)) {
  const auto fail = [this](const std::string& what) {
    throw std::invalid_argument("sensor '" + id() + "': " + what);
  };
  if (params_.width_px <= 0 || params_.height_px <= 0) fail("image size must be positive");
  if (!(params_.fx_px > 0.0 && params_.fy_px > 0.0)) fail("focal lengths must be positive");
  if (!(params_.size_pd_slope_px > 0.0)) fail("size_pd_slope_px must be positive");
  for (std::size_t i = 1; i < params_.gimbal.size(); ++i) {
    if (!(params_.gimbal[i].t_s > params_.gimbal[i - 1].t_s)) fail("gimbal keys must have increasing t_s");
  }
}

GimbalReport Camera::gimbal_at(double t) const {
  const auto& keys = params_.gimbal;
  if (keys.empty()) return {};
  if (t <= keys.front().t_s) return {keys.front().pan_rad, keys.front().tilt_rad};
  if (t >= keys.back().t_s) return {keys.back().pan_rad, keys.back().tilt_rad};
  const auto next = std::upper_bound(keys.begin(), keys.end(), t,
                                     [](double value, const GimbalKey& k) { return value < k.t_s; });
  const GimbalKey& b = *next;
  const GimbalKey& a = *std::prev(next);
  const double f = (t - a.t_s) / (b.t_s - a.t_s);
  return {a.pan_rad + f * (b.pan_rad - a.pan_rad), a.tilt_rad + f * (b.tilt_rad - a.tilt_rad)};
}

Mat3 Camera::R_true_world_from_view(const std::optional<GimbalReport>& pointing) const {
  if (!pointing) return R_true();
  const GimbalReport& g = *pointing;
  // Pan turns clockwise about the mount's up axis, then tilt raises the boresight.
  return R_true() * (Eigen::AngleAxisd(-g.pan_rad, Vec3::UnitZ()) *
                     Eigen::AngleAxisd(-g.tilt_rad, Vec3::UnitY()))
                        .toRotationMatrix();
}

std::optional<Eigen::Vector2d> Camera::project(const Vec3& p_world, double t) const {
  return project(p_world, has_gimbal() ? std::optional<GimbalReport>(gimbal_at(t)) : std::nullopt);
}

std::optional<Eigen::Vector2d> Camera::project(const Vec3& p_world, const std::optional<GimbalReport>& pointing) const {
  const Vec3 b = R_true_world_from_view(pointing).transpose() * (p_world - true_position());
  if (b.x() <= 0.0) return std::nullopt;  // behind the camera
  // Body (forward, left, up) -> image (right, down) on the normalised plane.
  const double xn = -b.y() / b.x();
  const double yn = -b.z() / b.x();
  // Far outside the image the distortion polynomial folds back; reject early.
  if (std::abs(xn) > 2.0 * params_.width_px / params_.fx_px ||
      std::abs(yn) > 2.0 * params_.height_px / params_.fy_px) {
    return std::nullopt;
  }
  const double r2 = xn * xn + yn * yn;
  const double distortion = 1.0 + params_.k1 * r2 + params_.k2 * r2 * r2;
  const double u = params_.fx_px * xn * distortion + params_.cx_px;
  const double v = params_.fy_px * yn * distortion + params_.cy_px;
  if (u < 0.0 || v < 0.0 || u >= params_.width_px || v >= params_.height_px) return std::nullopt;
  return Eigen::Vector2d(u, v);
}

double Camera::detection_probability(double size_px) const {
  if (size_px < params_.min_size_px) return 0.0;
  return params_.pd_max / (1.0 + std::exp(-(size_px - params_.size_at_half_pd_px) / params_.size_pd_slope_px));
}

Scan Camera::observe(double t, const World& world, Rng& rng) const {
  return observe_with(t, world, rng, has_gimbal() ? std::optional<GimbalReport>(gimbal_at(t)) : std::nullopt);
}

Scan Camera::observe_pointing(double t, const World& world, Rng& rng, const GimbalReport& pointing) const {
  if (!has_gimbal()) throw std::logic_error("camera '" + id() + "' has no gimbal to point");
  return observe_with(t, world, rng, pointing);
}

Scan Camera::observe_with(double t, const World& world, Rng& rng, const std::optional<GimbalReport>& pointing) const {
  Scan scan;
  scan.t_true_s = t;
  scan.t_ready_s = t;
  scan.gimbal = pointing;  // the gimbal encoders report where it points

  const double scale = common_.errors.noise_scale;
  for (const Target& target : world.targets) {
    if (!target.trajectory.active(t)) continue;
    const KinematicState state = target.trajectory.state(t);
    const auto pixel = project(state.position, pointing);
    if (!pixel) continue;
    const double range = (state.position - true_position()).norm();
    if (range > params_.max_range_m) continue;
    const double size_px = params_.fx_px * target.size_m / range;
    if (!rng.bernoulli(detection_probability(size_px))) continue;

    CameraDetection d;
    d.u_px = pixel->x() + scale * params_.sigma_px * rng.normal();
    d.v_px = pixel->y() + scale * params_.sigma_px * rng.normal();
    d.width_px = std::max(1.0, size_px * (1.0 + params_.size_noise_frac * rng.normal()));
    d.height_px = std::max(1.0, 0.5 * size_px * (1.0 + params_.size_noise_frac * rng.normal()));
    const std::size_t truth = class_index(target.cls);
    if (rng.bernoulli(params_.class_accuracy)) {
      d.label = std::string(to_string(target.cls));
      d.confidence = rng.uniform(0.6, 0.95);
    } else {
      const std::size_t wrong = (truth + (rng.bernoulli(0.5) ? 1 : 2)) % kClasses.size();
      d.label = std::string(to_string(kClasses[wrong]));
      d.confidence = rng.uniform(0.3, 0.7);
    }
    scan.detections.push_back(make_detection(t, target.id, d));
  }

  // False alarms: small boxes anywhere in the frame (clouds, glints, leaves).
  const int false_alarms = rng.poisson(params_.false_alarms_per_frame);
  for (int i = 0; i < false_alarms; ++i) {
    CameraDetection d;
    d.u_px = rng.uniform(0.0, params_.width_px);
    d.v_px = rng.uniform(0.0, params_.height_px);
    d.width_px = rng.uniform(3.0, 15.0);
    d.height_px = d.width_px * rng.uniform(0.4, 1.0);
    d.label = std::string(to_string(kClasses[static_cast<std::size_t>(rng.uniform(0.0, 3.0))]));
    d.confidence = rng.uniform(0.25, 0.6);
    scan.detections.push_back(make_detection(t, -1, d));
  }
  return scan;
}

}  // namespace skytrack

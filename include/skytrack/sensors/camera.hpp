#pragma once

#include <optional>
#include <vector>

#include "skytrack/sensor.hpp"

namespace skytrack {

/// A pan/tilt command at a point in time; the gimbal moves linearly between keys.
struct GimbalKey {
  double t_s = 0.0;
  double pan_rad = 0.0;   // clockwise from the mount's boresight
  double tilt_rad = 0.0;  // up
};

struct CameraParams {
  int width_px = 1920;
  int height_px = 1080;
  double fx_px = 1400.0;
  double fy_px = 1400.0;
  double cx_px = 960.0;
  double cy_px = 540.0;
  double k1 = 0.0;  // radial distortion
  double k2 = 0.0;

  double sigma_px = 1.5;  // stated 1-sigma noise of the box centre

  // Detection probability rises with apparent size:
  // pd_max / (1 + exp(-(size_px - size_at_half_pd_px) / size_pd_slope_px)),
  // and is zero below min_size_px (a box detector cannot find sub-pixel objects).
  double min_size_px = 1.0;
  double size_at_half_pd_px = 4.0;
  double size_pd_slope_px = 1.0;
  double pd_max = 0.95;
  double max_range_m = 5000.0;

  double false_alarms_per_frame = 0.2;
  double class_accuracy = 0.85;  // chance the detector's label is right
  double size_noise_frac = 0.15;

  std::vector<GimbalKey> gimbal;  // empty = fixed camera
};

/// Pinhole camera with an object detector on its output. It reports box
/// centres in pixels, so the tracker has to know the intrinsics and the
/// mount to turn them into bearings. A camera gives no range.
class Camera final : public Sensor {
 public:
  Camera(SensorCommon common, CameraParams params);

  SensorType type() const override { return SensorType::Camera; }
  Scan observe(double t, const World& world, Rng& rng) const override;
  /// A frame with the gimbal where a controller put it, not where the
  /// scenario's keys say (closed-loop cueing). Needs a gimbal.
  Scan observe_pointing(double t, const World& world, Rng& rng, const GimbalReport& pointing) const;

  const CameraParams& params() const { return params_; }
  bool has_gimbal() const { return !params_.gimbal.empty(); }
  GimbalReport gimbal_at(double t) const;
  /// Noise-free pixel of a world point with the camera's true pose at time t,
  /// or nothing if the point is behind the camera or outside the image.
  std::optional<Eigen::Vector2d> project(const Vec3& p_world, double t) const;
  std::optional<Eigen::Vector2d> project(const Vec3& p_world, const std::optional<GimbalReport>& pointing) const;
  double detection_probability(double size_px) const;

 private:
  Mat3 R_true_world_from_view(const std::optional<GimbalReport>& pointing) const;
  Scan observe_with(double t, const World& world, Rng& rng, const std::optional<GimbalReport>& pointing) const;

  CameraParams params_;
};

}  // namespace skytrack

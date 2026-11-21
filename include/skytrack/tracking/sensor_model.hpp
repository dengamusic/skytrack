#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <Eigen/Dense>

#include "skytrack/geo.hpp"
#include "skytrack/measurements.hpp"
#include "skytrack/sensors/camera.hpp"
#include "skytrack/sensors/cooperative.hpp"
#include "skytrack/sensors/radar.hpp"
#include "skytrack/sensors/rf_bearing.hpp"
#include "skytrack/tracking/config.hpp"
#include "skytrack/tracking/motion.hpp"

namespace skytrack {

class SensorModel;

/// One detection, ready for the filter.
struct Measurement {
  std::int64_t meas_id = -1;
  const SensorModel* sensor = nullptr;
  double t_s = 0.0;  // when it was measured, on the tracker's clock
  std::optional<double> t_stamp_s;  // t_s before the estimated clock error was taken out
  Eigen::VectorXd z;
  std::optional<GimbalReport> gimbal;  // camera pointing when the frame was taken
  /// Estimated mount error (heading, pitch) in radians, from online
  /// registration. h(x) is evaluated with the mount turned by this much.
  /// (The estimated clock error is already taken out of t_s.)
  Eigen::Vector2d mount_correction = Eigen::Vector2d::Zero();
  /// What is still unknown about (heading, pitch, clock) of the sensor.
  Eigen::Matrix3d registration_covariance = Eigen::Matrix3d::Zero();
  // Attributes used for classification and identity.
  std::string label;       // camera class guess
  double confidence = 0.0;
  std::string signature;   // RF emitter type
  double snr_db = 0.0;     // radar: lets the tracker estimate the target's radar cross section
  std::string identity;    // cooperative "protocol:id"
  std::optional<Eigen::Vector2d> reported_std_m;  // cooperative: accuracy it reports (horizontal, vertical)
};

/// The tracker's model of one sensor, built from its nominal (as surveyed)
/// setup. It knows nothing about the errors the simulator injected.
///
/// predict() is the measurement function h(x): what the sensor would report
/// for a target in state x. The filter linearises it numerically, so a new
/// sensor type needs only h(x), its noise and how to read its detections.
class SensorModel {
 public:
  SensorModel(const Sensor& nominal, SensorOverrides overrides);
  virtual ~SensorModel() = default;
  SensorModel(const SensorModel&) = delete;
  SensorModel& operator=(const SensorModel&) = delete;

  const std::string& id() const { return id_; }
  const Vec3& position() const { return position_; }
  virtual SensorType type() const = 0;

  /// A detection from a message, converted to a measurement on the tracker clock.
  Measurement convert(const Detection& detection, const SensorMessage& msg) const;
  /// When the scan a message reports had finished, on the tracker clock.
  double scan_end_time(const SensorMessage& msg) const;

  /// h(x); empty if the sensor cannot observe x at all (a camera looking away).
  virtual std::optional<Eigen::VectorXd> predict(const Vec6& x, const Measurement& m) const = 0;
  virtual Eigen::MatrixXd noise(const Measurement& m) const = 0;
  /// z - zhat, with angles wrapped.
  virtual Eigen::VectorXd residual(const Eigen::VectorXd& z, const Eigen::VectorXd& zhat) const { return z - zhat; }

  /// A new track from a single measurement, if this sensor measures enough to start one.
  virtual std::optional<Gaussian> initiate(const Measurement& /*m*/, const TrackerConfig& /*cfg*/) const {
    return std::nullopt;
  }
  /// Whether the sensor can see a target in state x at all (range limits).
  /// A track it cannot see never competes for its measurements.
  virtual bool can_observe(const Vec6& /*x*/) const { return true; }
  /// Whether its measurements depend on how the sensor is mounted, so that
  /// online registration can estimate its heading and pitch errors.
  virtual bool measures_angles() const { return true; }

 protected:
  virtual void read_payload(const MeasurementPayload& payload, Measurement& m) const = 0;
  virtual double scan_duration_s() const { return 0.0; }
  /// Mount orientation, corrected by the measurement's registration estimate.
  Mat3 rotation(const Measurement& m) const;
  /// A world point in the sensor's nominal body frame.
  Vec3 to_body(const Vec3& p_world) const { return R_nominal_.transpose() * (p_world - position_); }

  std::string id_;
  Vec3 position_;
  Mount mount_;
  Mat3 R_nominal_;
  SensorOverrides overrides_;
};

class RadarModel final : public SensorModel {
 public:
  explicit RadarModel(const Radar& radar, SensorOverrides overrides);
  SensorType type() const override { return SensorType::Radar; }
  std::optional<Eigen::VectorXd> predict(const Vec6& x, const Measurement& m) const override;
  Eigen::MatrixXd noise(const Measurement& m) const override;
  Eigen::VectorXd residual(const Eigen::VectorXd& z, const Eigen::VectorXd& zhat) const override;
  std::optional<Gaussian> initiate(const Measurement& m, const TrackerConfig& cfg) const override;
  bool rotating() const { return params_.mode == RadarParams::Mode::Rotating; }

  /// Detection probability for a target in state x with the given radar
  /// cross section, ignoring the Doppler notch; 0 outside the coverage.
  double expected_pd(const Vec6& x, double rcs_dbsm) const;
  /// Detection probability for an uncertain state, including the chance
  /// that its range rate falls inside the Doppler notch.
  double detection_probability(const Gaussian& g, double rcs_dbsm) const;
  /// One sample of the radar cross section (linear, m^2) from a detection's
  /// SNR. Unbiased for a Swerling I target even though only detections above
  /// the threshold are seen: the excess power over the threshold is
  /// exponential with mean 1 + SNR.
  double rcs_sample_m2(double snr_db, double range_m) const;
  /// Range rate of state x seen from this radar (positive = moving away).
  double range_rate(const Vec6& x) const;
  double doppler_notch_mps() const { return params_.doppler_notch_mps; }

 protected:
  void read_payload(const MeasurementPayload& payload, Measurement& m) const override;
  double scan_duration_s() const override { return rotating() ? period_s_ : 0.0; }

 private:
  RadarParams params_;
  double period_s_;
};

class CameraModel final : public SensorModel {
 public:
  explicit CameraModel(const Camera& camera, SensorOverrides overrides);
  SensorType type() const override { return SensorType::Camera; }
  std::optional<Eigen::VectorXd> predict(const Vec6& x, const Measurement& m) const override;
  Eigen::MatrixXd noise(const Measurement& m) const override;
  bool can_observe(const Vec6& x) const override;

 protected:
  void read_payload(const MeasurementPayload& payload, Measurement& m) const override;

 private:
  CameraParams params_;
};

class BearingModel final : public SensorModel {
 public:
  explicit BearingModel(const RfBearingSensor& sensor, SensorOverrides overrides);
  SensorType type() const override { return SensorType::RfBearing; }
  std::optional<Eigen::VectorXd> predict(const Vec6& x, const Measurement& m) const override;
  Eigen::MatrixXd noise(const Measurement& m) const override;
  Eigen::VectorXd residual(const Eigen::VectorXd& z, const Eigen::VectorXd& zhat) const override;
  bool can_observe(const Vec6& x) const override;

 protected:
  void read_payload(const MeasurementPayload& payload, Measurement& m) const override;

 private:
  RfBearingParams params_;
};

class CooperativeModel final : public SensorModel {
 public:
  CooperativeModel(const CooperativeReceiver& receiver, SensorOverrides overrides, const geo::LocalFrame& frame,
                   double std_h_m, double std_v_m, double white_std_m);
  SensorType type() const override { return SensorType::Cooperative; }
  std::optional<Eigen::VectorXd> predict(const Vec6& x, const Measurement& m) const override;
  /// All of the report's error: the slowly drifting GNSS error plus jitter.
  Eigen::MatrixXd noise(const Measurement& m) const override;
  /// The GNSS position error (east, north, up), which consecutive reports of
  /// one target share: the accuracy the report states, else the configured one.
  Eigen::Matrix3d bias_covariance(const Measurement& m) const;
  /// What each report adds on its own: position jitter and velocity noise.
  Eigen::MatrixXd white_noise(const Measurement& m) const;
  std::optional<Gaussian> initiate(const Measurement& m, const TrackerConfig& cfg) const override;
  bool measures_angles() const override { return false; }

 protected:
  void read_payload(const MeasurementPayload& payload, Measurement& m) const override;

 private:
  CooperativeParams params_;
  geo::LocalFrame frame_;
  double std_h_m_;
  double std_v_m_;
  double white_std_m_;
};

/// The nominal site: origin and sensor models, read from sensors.json.
class SensorSetup {
 public:
  SensorSetup(const Json& sensors_json, const TrackerConfig& config);
  static SensorSetup load(const std::filesystem::path& sensors_json, const TrackerConfig& config);

  const geo::LocalFrame& frame() const { return frame_; }
  const std::vector<std::unique_ptr<SensorModel>>& sensors() const { return sensors_; }
  /// nullptr if there is no such sensor.
  const SensorModel* find(std::string_view id) const;

 private:
  geo::LocalFrame frame_;
  std::vector<std::unique_ptr<SensorModel>> sensors_;
};

/// Numerical Jacobian of the measurement function at x (central differences).
/// Empty if the sensor cannot observe x.
Eigen::MatrixXd measurement_jacobian(const SensorModel& model, const Vec6& x, const Measurement& m);
/// Jacobian of the measurement function with respect to the sensor's
/// registration errors (heading, pitch, clock) for a target in state x.
/// Empty if the sensor cannot observe x.
Eigen::MatrixXd registration_jacobian(const SensorModel& model, const Vec6& x, const Measurement& m);

/// The chi-square gate for a measurement of this dimension (probability 0.999).
double gate_threshold(Eigen::Index dim);

}  // namespace skytrack

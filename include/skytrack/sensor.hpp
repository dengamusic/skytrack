#pragma once

#include <optional>
#include <string>
#include <vector>

#include "skytrack/geo.hpp"
#include "skytrack/measurements.hpp"
#include "skytrack/rng.hpp"
#include "skytrack/target.hpp"

namespace skytrack {

/// Where a sensor sits and which way it points.
struct Mount {
  Vec3 position_enu = Vec3::Zero();
  double heading_rad = 0.0;  // compass bearing of the boresight
  double pitch_rad = 0.0;    // up
  double roll_rad = 0.0;

  Mat3 R_world_from_body() const { return geo::rotation_world_from_body(heading_rad, pitch_rad, roll_rad); }
};

/// The difference between a sensor as configured and as it really is.
/// The tracker only ever sees the configured (nominal) values.
struct SensorErrors {
  Vec3 position_offset_enu = Vec3::Zero();  // survey error
  double heading_bias_rad = 0.0;            // mounted turned clockwise by this much
  double pitch_bias_rad = 0.0;
  double roll_bias_rad = 0.0;
  double clock_offset_s = 0.0;   // sensor clock minus true time
  double clock_drift_ppm = 0.0;
  double latency_mean_s = 0.0;   // from "scan finished" to "message received"
  double latency_jitter_s = 0.0;
  double dropout_prob = 0.0;     // chance a whole message is lost
  double noise_scale = 1.0;      // real noise / noise stated in the config
};

/// Whether a sensor stamps its own data or the fusion host has to use arrival time.
enum class TimestampSource { Sensor, Arrival };

struct SensorCommon {
  std::string id;
  double rate_hz = 1.0;  // scans or frames per second
  double phase_s = 0.0;  // time of the first scan
  TimestampSource timestamps = TimestampSource::Sensor;
  Mount mount;           // nominal (as surveyed)
  SensorErrors errors;   // hidden truth
};

/// One scan or frame as the sensor produced it, before clock and network effects.
struct Scan {
  double t_true_s = 0.0;   // scan start or frame time
  double t_ready_s = 0.0;  // when the sensor could send it (end of scan for a rotating radar)
  std::optional<GimbalReport> gimbal;
  std::vector<Detection> detections;  // t_true_s and origin set; meas_id and t_sensor_s set later
};

struct World {
  const std::vector<Target>& targets;
  const geo::LocalFrame& frame;
};

class Sensor {
 public:
  explicit Sensor(SensorCommon common);
  virtual ~Sensor() = default;
  Sensor(const Sensor&) = delete;
  Sensor& operator=(const Sensor&) = delete;

  virtual SensorType type() const = 0;
  /// Observes the world for the scan or frame that starts at true time t.
  virtual Scan observe(double t, const World& world, Rng& rng) const = 0;

  const SensorCommon& common() const { return common_; }
  const std::string& id() const { return common_.id; }
  double period_s() const { return 1.0 / common_.rate_hz; }

  /// Nominal mount plus the hidden errors: where the sensor really is and points.
  Mount true_mount() const;
  /// What the sensor's clock reads at true time t.
  double sensor_clock(double t_true) const;

 protected:
  const Vec3& true_position() const { return true_position_; }
  const Mat3& R_true() const { return R_true_; }
  /// A world point relative to the sensor, in its true body frame.
  Vec3 relative_body(const Vec3& p_world) const { return R_true_.transpose() * (p_world - true_position_); }

  SensorCommon common_;

 private:
  Vec3 true_position_;
  Mat3 R_true_;
};

inline Detection make_detection(double t_true, int origin_target_id, MeasurementPayload payload) {
  Detection d;
  d.payload = std::move(payload);
  d.t_true_s = t_true;
  d.origin_target_id = origin_target_id;
  return d;
}

}  // namespace skytrack

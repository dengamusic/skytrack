#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include "skytrack/json.hpp"

/// What sensors send to the fusion host. These types are shared with the
/// tracker, which reads them back from measurements.jsonl.
namespace skytrack {

enum class SensorType { Radar, Camera, RfBearing, Cooperative };

std::string_view to_string(SensorType type);
std::optional<SensorType> sensor_type_from_string(std::string_view name);

/// Radar plot in the radar's own body frame.
struct RadarDetection {
  double range_m = 0.0;
  double az_rad = 0.0;          // counter-clockwise from boresight
  double el_rad = 0.0;
  double range_rate_mps = 0.0;  // positive = moving away
  double snr_db = 0.0;
};

/// Bounding-box centre and size in pixels, plus the detector's class guess.
struct CameraDetection {
  double u_px = 0.0;
  double v_px = 0.0;
  double width_px = 0.0;
  double height_px = 0.0;
  std::string label;
  double confidence = 0.0;
};

/// Direction-finding bearing in the antenna's body frame.
struct BearingDetection {
  double az_rad = 0.0;  // counter-clockwise from boresight
  double snr_db = 0.0;
  std::string signature;  // emitter type if recognised, else empty
};

/// Position a cooperative target broadcasts about itself (ADS-B, Remote ID).
struct CooperativeReport {
  std::string protocol;
  std::string identity;
  double lat_deg = 0.0;
  double lon_deg = 0.0;
  double alt_m = 0.0;
  double vel_east_mps = 0.0;
  double vel_north_mps = 0.0;
  double vel_up_mps = 0.0;
  /// The position accuracy the target reports for itself (1 sigma), as
  /// ADS-B does with NACp and Remote ID with its accuracy fields.
  std::optional<double> accuracy_h_m;
  std::optional<double> accuracy_v_m;
};

using MeasurementPayload = std::variant<RadarDetection, CameraDetection, BearingDetection, CooperativeReport>;

struct Detection {
  std::int64_t meas_id = -1;
  std::optional<double> t_sensor_s;  // sensor clock; empty if the sensor does not timestamp
  MeasurementPayload payload;
  // Ground truth. Never written to measurements.jsonl.
  double t_true_s = 0.0;
  int origin_target_id = -1;  // -1 = false alarm
};

struct GimbalReport {
  double pan_rad = 0.0;   // clockwise from the mount's boresight
  double tilt_rad = 0.0;  // up
};

/// One message as the fusion host receives it: a radar scan, a camera frame, ...
/// A message with no detections still matters: it says the sensor looked and saw nothing.
struct SensorMessage {
  std::string sensor_id;
  SensorType sensor_type = SensorType::Radar;
  std::uint64_t seq = 0;             // scan counter; a gap means messages were lost
  double t_arrival_s = 0.0;          // fusion host clock
  std::optional<double> t_sensor_s;  // scan start on the sensor clock
  std::optional<GimbalReport> gimbal;
  std::vector<Detection> detections;
  // Ground truth. Never written to measurements.jsonl.
  double t_true_s = 0.0;
};

/// The fields a tracker may see, as one JSON object (one line of measurements.jsonl).
Json to_observed_json(const SensorMessage& msg);
/// Inverse of to_observed_json. Ground-truth fields stay at their defaults.
SensorMessage from_observed_json(const Json& j);

}  // namespace skytrack

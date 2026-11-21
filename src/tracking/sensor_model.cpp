#include "skytrack/tracking/sensor_model.hpp"

#include <cmath>
#include <stdexcept>

#include "skytrack/scenario.hpp"

namespace skytrack {
namespace {

template <typename Payload>
const Payload& payload_as(const MeasurementPayload& payload, const std::string& sensor_id) {
  const Payload* p = std::get_if<Payload>(&payload);
  if (p == nullptr) throw std::runtime_error("sensor '" + sensor_id + "' sent a detection of the wrong type");
  return *p;
}

Eigen::VectorXd vec(std::initializer_list<double> values) {
  Eigen::VectorXd v(static_cast<Eigen::Index>(values.size()));
  Eigen::Index i = 0;
  for (double x : values) v[i++] = x;
  return v;
}

}  // namespace

// ---- base -------------------------------------------------------------------

SensorModel::SensorModel(const Sensor& nominal, SensorOverrides overrides)
    : id_(nominal.id()),
      position_(nominal.common().mount.position_enu),
      mount_(nominal.common().mount),
      R_nominal_(mount_.R_world_from_body()),
      overrides_(overrides) {}

Mat3 SensorModel::rotation(const Measurement& m) const {
  if (m.mount_correction.isZero()) return R_nominal_;
  return geo::rotation_world_from_body(mount_.heading_rad + m.mount_correction[0],
                                       mount_.pitch_rad + m.mount_correction[1], mount_.roll_rad);
}

Measurement SensorModel::convert(const Detection& detection, const SensorMessage& msg) const {
  Measurement m;
  m.meas_id = detection.meas_id;
  m.sensor = this;
  if (detection.t_sensor_s) {
    m.t_s = *detection.t_sensor_s - overrides_.clock_offset_s;
  } else if (msg.t_sensor_s) {
    m.t_s = *msg.t_sensor_s - overrides_.clock_offset_s;
  } else {
    m.t_s = msg.t_arrival_s - overrides_.assumed_latency_s;
  }
  m.gimbal = msg.gimbal;
  read_payload(detection.payload, m);
  return m;
}

double SensorModel::scan_end_time(const SensorMessage& msg) const {
  if (msg.t_sensor_s) return *msg.t_sensor_s - overrides_.clock_offset_s + scan_duration_s();
  return msg.t_arrival_s - overrides_.assumed_latency_s;
}

Eigen::MatrixXd measurement_jacobian(const SensorModel& model, const Vec6& x, const Measurement& m) {
  // Steps small against the scales involved (metres, m/s) but far above rounding.
  constexpr double kStep = 1e-2;
  Eigen::MatrixXd H;
  for (int k = 0; k < 6; ++k) {
    Vec6 plus = x;
    Vec6 minus = x;
    plus[k] += kStep;
    minus[k] -= kStep;
    const auto zp = model.predict(plus, m);
    const auto zm = model.predict(minus, m);
    if (!zp || !zm) return {};
    if (H.size() == 0) H.resize(zp->size(), 6);
    H.col(k) = model.residual(*zp, *zm) / (2.0 * kStep);
  }
  return H;
}

Eigen::MatrixXd registration_jacobian(const SensorModel& model, const Vec6& x, const Measurement& m) {
  constexpr double kStep = 1e-6;  // radians
  Eigen::MatrixXd J;
  for (int k = 0; k < 2; ++k) {
    Measurement plus = m;
    Measurement minus = m;
    plus.mount_correction[k] += kStep;
    minus.mount_correction[k] -= kStep;
    const auto zp = model.predict(x, plus);
    const auto zm = model.predict(x, minus);
    if (!zp || !zm) return {};
    if (J.size() == 0) J.resize(zp->size(), 3);
    J.col(k) = model.residual(*zp, *zm) / (2.0 * kStep);
  }
  // A clock running dt ahead means the measurement was really taken dt
  // earlier, when the target was v * dt further back along its path.
  const Eigen::MatrixXd H = measurement_jacobian(model, x, m);
  if (H.size() == 0) return {};
  J.col(2) = -H.leftCols(3) * x.tail<3>();
  return J;
}

double gate_threshold(Eigen::Index dim) {
  static constexpr double kChi2_999[] = {0.0, 10.828, 13.816, 16.266, 18.467, 20.515, 22.458};
  if (dim < 1 || dim > 6) throw std::invalid_argument("no gate for a measurement of this size");
  return kChi2_999[dim];
}

// ---- radar ------------------------------------------------------------------

RadarModel::RadarModel(const Radar& radar, SensorOverrides overrides)
    : SensorModel(radar, overrides), params_(radar.params()), period_s_(radar.period_s()) {}

void RadarModel::read_payload(const MeasurementPayload& payload, Measurement& m) const {
  const auto& d = payload_as<RadarDetection>(payload, id_);
  m.z = vec({d.range_m, d.az_rad, d.el_rad, d.range_rate_mps});
  m.snr_db = d.snr_db;
}

std::optional<Eigen::VectorXd> RadarModel::predict(const Vec6& x, const Measurement& m) const {
  const Vec3 rel = rotation(m).transpose() * (x.head<3>() - position_);
  const geo::Spherical s = geo::cartesian_to_spherical(rel);
  if (s.range < 1.0) return std::nullopt;
  return vec({s.range, s.az, s.el, range_rate(x)});
}

double RadarModel::range_rate(const Vec6& x) const {
  const Vec3 rel = x.head<3>() - position_;  // range rate does not depend on the mount
  const double range = rel.norm();
  return range > 0.0 ? rel.dot(x.tail<3>()) / range : 0.0;
}

Eigen::MatrixXd RadarModel::noise(const Measurement& /*m*/) const {
  return vec({params_.sigma_range_m * params_.sigma_range_m, params_.sigma_az_rad * params_.sigma_az_rad,
              params_.sigma_el_rad * params_.sigma_el_rad,
              params_.sigma_range_rate_mps * params_.sigma_range_rate_mps})
      .asDiagonal();
}

Eigen::VectorXd RadarModel::residual(const Eigen::VectorXd& z, const Eigen::VectorXd& zhat) const {
  Eigen::VectorXd r = z - zhat;
  r[1] = geo::wrap_pi(r[1]);
  return r;
}

std::optional<Gaussian> RadarModel::initiate(const Measurement& m, const TrackerConfig& cfg) const {
  const double r = m.z[0];
  const double az = m.z[1];
  const double el = m.z[2];
  const Vec3 rel_body = geo::spherical_to_cartesian({r, az, el});
  // d(x, y, z) / d(range, az, el), to carry the measurement noise into position.
  Mat3 J;
  J << std::cos(el) * std::cos(az), -r * std::cos(el) * std::sin(az), -r * std::sin(el) * std::cos(az),
      std::cos(el) * std::sin(az), r * std::cos(el) * std::cos(az), -r * std::sin(el) * std::sin(az),
      std::sin(el), 0.0, r * std::cos(el);
  // Angle noise plus what is still unknown about the mount (heading ~ azimuth, pitch ~ elevation).
  const Vec3 sigma2(params_.sigma_range_m * params_.sigma_range_m,
                    params_.sigma_az_rad * params_.sigma_az_rad + m.registration_covariance(0, 0),
                    params_.sigma_el_rad * params_.sigma_el_rad + m.registration_covariance(1, 1));
  const Mat3 R = rotation(m);
  const Mat3 JR = R * J;

  Gaussian g;
  g.x.head<3>() = position_ + R * rel_body;
  // Doppler gives the speed along the line of sight; the rest is unknown.
  const Vec3 los = (R * rel_body).normalized();
  g.x.tail<3>() = m.z[3] * los;
  const Mat3 along = los * los.transpose();
  const double v2 = cfg.init_velocity_std_mps * cfg.init_velocity_std_mps;
  g.P.setZero();
  g.P.topLeftCorner<3, 3>() = JR * sigma2.asDiagonal() * JR.transpose();
  g.P.bottomRightCorner<3, 3>() =
      v2 * (Mat3::Identity() - along) + params_.sigma_range_rate_mps * params_.sigma_range_rate_mps * along;
  return g;
}

double RadarModel::expected_pd(const Vec6& x, double rcs_dbsm) const {
  const geo::Spherical s = geo::cartesian_to_spherical(to_body(x.head<3>()));
  if (s.range < params_.min_range_m || s.range > params_.max_range_m) return 0.0;
  if (s.el < params_.min_el_rad || s.el > params_.max_el_rad) return 0.0;
  if (params_.fov_az_rad < 2.0 * geo::kPi && std::abs(s.az) > 0.5 * params_.fov_az_rad) return 0.0;
  const double snr = params_.snr_ref_db + rcs_dbsm - 40.0 * std::log10(s.range / params_.ref_range_m);
  return std::pow(params_.pfa, 1.0 / (1.0 + std::pow(10.0, snr / 10.0)));
}

double RadarModel::detection_probability(const Gaussian& g, double rcs_dbsm) const {
  const double pd = expected_pd(g.x, rcs_dbsm);
  const double notch = params_.doppler_notch_mps;
  if (pd <= 0.0 || !(notch > 0.0)) return pd;
  // Range rate ~ N(mean, sigma^2) from the track; the radar drops |range rate| < notch.
  const Vec3 rel = g.x.head<3>() - position_;
  const Vec3 los = rel.normalized();
  const double mean = los.dot(g.x.tail<3>());
  const double sigma = std::sqrt(std::max(los.dot(g.P.bottomRightCorner<3, 3>() * los), 1e-6));
  const auto cdf = [](double z) { return 0.5 * std::erfc(-z / std::sqrt(2.0)); };
  const double in_notch = cdf((notch - mean) / sigma) - cdf((-notch - mean) / sigma);
  return pd * (1.0 - in_notch);
}

double RadarModel::rcs_sample_m2(double snr_db, double range_m) const {
  const double threshold = -std::log(params_.pfa);
  const double snr = std::pow(10.0, snr_db / 10.0) - threshold - 1.0;
  const double snr_ref = std::pow(10.0, params_.snr_ref_db / 10.0);
  return snr / snr_ref * std::pow(range_m / params_.ref_range_m, 4.0);
}


// ---- camera -----------------------------------------------------------------

CameraModel::CameraModel(const Camera& camera, SensorOverrides overrides)
    : SensorModel(camera, overrides), params_(camera.params()) {}

void CameraModel::read_payload(const MeasurementPayload& payload, Measurement& m) const {
  const auto& d = payload_as<CameraDetection>(payload, id_);
  m.z = vec({d.u_px, d.v_px});
  m.label = d.label;
  m.confidence = d.confidence;
}

std::optional<Eigen::VectorXd> CameraModel::predict(const Vec6& x, const Measurement& m) const {
  Mat3 R_view = rotation(m);
  if (m.gimbal) {
    // Same convention as the simulator: pan clockwise about up, then tilt up.
    R_view = R_view * (Eigen::AngleAxisd(-m.gimbal->pan_rad, Vec3::UnitZ()) *
                       Eigen::AngleAxisd(-m.gimbal->tilt_rad, Vec3::UnitY()))
                          .toRotationMatrix();
  }
  const Vec3 b = R_view.transpose() * (x.head<3>() - position_);
  if (b.x() < 1.0) return std::nullopt;  // behind (or right at) the camera
  const double xn = -b.y() / b.x();
  const double yn = -b.z() / b.x();
  if (std::abs(xn) > 2.0 * params_.width_px / params_.fx_px || std::abs(yn) > 2.0 * params_.height_px / params_.fy_px) {
    return std::nullopt;  // far outside the image, where the distortion model is meaningless
  }
  const double r2 = xn * xn + yn * yn;
  const double distortion = 1.0 + params_.k1 * r2 + params_.k2 * r2 * r2;
  return vec({params_.fx_px * xn * distortion + params_.cx_px, params_.fy_px * yn * distortion + params_.cy_px});
}

Eigen::MatrixXd CameraModel::noise(const Measurement& /*m*/) const {
  return Eigen::Matrix2d::Identity() * params_.sigma_px * params_.sigma_px;
}

bool CameraModel::can_observe(const Vec6& x) const { return (x.head<3>() - position_).norm() <= params_.max_range_m; }

// ---- RF direction finder ------------------------------------------------------

BearingModel::BearingModel(const RfBearingSensor& sensor, SensorOverrides overrides)
    : SensorModel(sensor, overrides), params_(sensor.params()) {}

void BearingModel::read_payload(const MeasurementPayload& payload, Measurement& m) const {
  const auto& d = payload_as<BearingDetection>(payload, id_);
  m.z = vec({d.az_rad});
  m.signature = d.signature;
}

std::optional<Eigen::VectorXd> BearingModel::predict(const Vec6& x, const Measurement& m) const {
  const Vec3 rel = rotation(m).transpose() * (x.head<3>() - position_);
  if (rel.head<2>().norm() < 1.0) return std::nullopt;  // straight overhead: bearing undefined
  return vec({std::atan2(rel.y(), rel.x())});
}

Eigen::MatrixXd BearingModel::noise(const Measurement& /*m*/) const {
  return Eigen::MatrixXd::Constant(1, 1, params_.sigma_az_rad * params_.sigma_az_rad);
}

Eigen::VectorXd BearingModel::residual(const Eigen::VectorXd& z, const Eigen::VectorXd& zhat) const {
  return vec({geo::wrap_pi(z[0] - zhat[0])});
}

bool BearingModel::can_observe(const Vec6& x) const {
  return (x.head<3>() - position_).norm() <= params_.max_range_m;
}

// ---- cooperative ----------------------------------------------------------------

CooperativeModel::CooperativeModel(const CooperativeReceiver& receiver, SensorOverrides overrides,
                                   const geo::LocalFrame& frame, double std_h_m, double std_v_m,
                                   double white_std_m)
    : SensorModel(receiver, overrides),
      params_(receiver.params()),
      frame_(frame),
      std_h_m_(overrides.position_std_h_m.value_or(std_h_m)),
      std_v_m_(overrides.position_std_v_m.value_or(std_v_m)),
      white_std_m_(white_std_m) {}

void CooperativeModel::read_payload(const MeasurementPayload& payload, Measurement& m) const {
  const auto& d = payload_as<CooperativeReport>(payload, id_);
  const Vec3 p = frame_.geodetic_to_enu({geo::deg2rad(d.lat_deg), geo::deg2rad(d.lon_deg), d.alt_m});
  m.z = vec({p.x(), p.y(), p.z(), d.vel_east_mps, d.vel_north_mps, d.vel_up_mps});
  m.identity = d.protocol + ":" + d.identity;
  if (d.accuracy_h_m && d.accuracy_v_m && *d.accuracy_h_m > 0.0 && *d.accuracy_v_m > 0.0) {
    m.reported_std_m = Eigen::Vector2d(*d.accuracy_h_m, *d.accuracy_v_m);
  }
}

std::optional<Eigen::VectorXd> CooperativeModel::predict(const Vec6& x, const Measurement& /*m*/) const {
  return Eigen::VectorXd(x);
}

Eigen::Matrix3d CooperativeModel::bias_covariance(const Measurement& m) const {
  const Eigen::Vector2d s = m.reported_std_m.value_or(Eigen::Vector2d(std_h_m_, std_v_m_));
  return Eigen::Vector3d(s[0] * s[0], s[0] * s[0], s[1] * s[1]).asDiagonal();
}

Eigen::MatrixXd CooperativeModel::white_noise(const Measurement& /*m*/) const {
  const double p2 = white_std_m_ * white_std_m_;
  const double v2 = params_.velocity_std_mps * params_.velocity_std_mps;
  return vec({p2, p2, p2, v2, v2, v2}).asDiagonal();
}

Eigen::MatrixXd CooperativeModel::noise(const Measurement& m) const {
  Eigen::MatrixXd R = white_noise(m);
  R.topLeftCorner<3, 3>() += bias_covariance(m);
  return R;
}

std::optional<Gaussian> CooperativeModel::initiate(const Measurement& m, const TrackerConfig& /*cfg*/) const {
  Gaussian g;
  g.x = m.z;
  g.P = noise(m);
  return g;
}

// ---- setup ------------------------------------------------------------------------

SensorSetup::SensorSetup(const Json& j, const TrackerConfig& config) {
  if (!j.contains("origin") || !j.contains("sensors")) throw std::runtime_error("sensors.json needs 'origin' and 'sensors'");
  frame_ = parse_origin(j.at("origin"));
  for (const Json& block : j.at("sensors")) {
    const std::unique_ptr<Sensor> sensor = parse_sensor(block);
    const auto it = config.sensors.find(sensor->id());
    const SensorOverrides overrides = it == config.sensors.end() ? SensorOverrides{} : it->second;
    if (const auto* radar = dynamic_cast<const Radar*>(sensor.get())) {
      sensors_.push_back(std::make_unique<RadarModel>(*radar, overrides));
    } else if (const auto* camera = dynamic_cast<const Camera*>(sensor.get())) {
      sensors_.push_back(std::make_unique<CameraModel>(*camera, overrides));
    } else if (const auto* rf = dynamic_cast<const RfBearingSensor*>(sensor.get())) {
      sensors_.push_back(std::make_unique<BearingModel>(*rf, overrides));
    } else if (const auto* coop = dynamic_cast<const CooperativeReceiver*>(sensor.get())) {
      sensors_.push_back(std::make_unique<CooperativeModel>(*coop, overrides, frame_, config.cooperative_std_h_m,
                                                            config.cooperative_std_v_m,
                                                            config.cooperative_white_std_m));
    } else {
      throw std::runtime_error("sensor '" + sensor->id() + "' has a type the tracker does not support");
    }
  }
  for (const auto& [id, overrides] : config.sensors) {
    if (find(id) == nullptr) throw std::runtime_error("tracker config: no sensor '" + id + "' in sensors.json");
  }
}

SensorSetup SensorSetup::load(const std::filesystem::path& path, const TrackerConfig& config) {
  try {
    return SensorSetup(read_json_file(path), config);
  } catch (const Json::exception& e) {
    throw std::runtime_error(path.string() + ": " + e.what());
  }
}

const SensorModel* SensorSetup::find(std::string_view id) const {
  for (const auto& s : sensors_) {
    if (s->id() == id) return s.get();
  }
  return nullptr;
}

}  // namespace skytrack

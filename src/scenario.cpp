#include "skytrack/scenario.hpp"

#include <algorithm>
#include <fstream>
#include <initializer_list>
#include <set>
#include <stdexcept>
#include <string_view>

#include "skytrack/json_fields.hpp"
#include "skytrack/sensors/camera.hpp"
#include "skytrack/sensors/cooperative.hpp"
#include "skytrack/sensors/radar.hpp"
#include "skytrack/sensors/rf_bearing.hpp"

namespace skytrack {
namespace {

using geo::deg2rad;

using namespace json;  // strict field readers

// ---- targets ----------------------------------------------------------------

MotionSegment parse_segment(const Json& j, const std::string& ctx) {
  check_object(j, ctx, {"model", "duration_s", "accel_enu_mps2", "turn_rate_deg_s"});
  MotionSegment s;
  s.duration_s = number(j, "duration_s", ctx);
  require(s.duration_s > 0.0, ctx, "duration_s must be positive");
  const std::string model = text(j, "model", ctx);
  if (model == "cv") {
    s.model = MotionModel::ConstantVelocity;
    require(!j.contains("accel_enu_mps2") && !j.contains("turn_rate_deg_s"), ctx, "a 'cv' segment takes only duration_s");
  } else if (model == "ca") {
    s.model = MotionModel::ConstantAcceleration;
    s.acceleration = vec3(j, "accel_enu_mps2", ctx);
    require(!j.contains("turn_rate_deg_s"), ctx, "a 'ca' segment has no turn rate");
  } else if (model == "ct") {
    s.model = MotionModel::CoordinatedTurn;
    s.turn_rate_rad_s = deg2rad(number(j, "turn_rate_deg_s", ctx));
    require(!j.contains("accel_enu_mps2"), ctx, "a 'ct' segment has no acceleration");
  } else {
    fail(ctx, "model must be 'cv', 'ca' or 'ct', got '" + model + "'");
  }
  return s;
}

Target parse_target(const Json& j, double duration_s, std::uint64_t seed) {
  std::string ctx = "target";
  if (j.is_object() && j.contains("id")) ctx += " " + j.at("id").dump();
  check_object(j, ctx, {"id", "name", "class", "t_start_s", "t_end_s", "position_enu_m", "velocity_enu_mps",
                        "segments", "rcs_dbsm", "size_m", "rf_signature", "cooperative"});

  const std::int64_t id = integer(j, "id", ctx);
  require(id >= 0 && id < 1000000, ctx, "id must be between 0 and 999999");

  const std::string class_name = text(j, "class", ctx);
  const auto cls = target_class_from_string(class_name);
  if (!cls) fail(ctx, "class must be 'drone', 'aircraft' or 'bird', got '" + class_name + "'");

  std::vector<MotionSegment> segments;
  if (j.contains("segments")) {
    const Json& list = array_field(j, "segments", ctx);
    for (std::size_t i = 0; i < list.size(); ++i) {
      segments.push_back(parse_segment(list.at(i), ctx + " segment " + std::to_string(i)));
    }
  }
  const double t_start = number_or(j, "t_start_s", 0.0, ctx);
  const double t_end = number_or(j, "t_end_s", duration_s, ctx);
  require(t_end > t_start, ctx, "t_end_s must be after t_start_s");

  const double size_m = number_or(j, "size_m", 0.5, ctx);
  require(size_m > 0.0, ctx, "size_m must be positive");

  Target target{
      .id = static_cast<int>(id),
      .name = text_or(j, "name", "target-" + std::to_string(id), ctx),
      .cls = *cls,
      .trajectory = Trajectory(t_start, t_end, vec3(j, "position_enu_m", ctx),
                               vec3_or(j, "velocity_enu_mps", Vec3::Zero(), ctx), segments),
      .rcs_dbsm = number_or(j, "rcs_dbsm", -20.0, ctx),
      .size_m = size_m,
      .rf_signature = text_or(j, "rf_signature", "", ctx),
      .cooperative = std::nullopt,
  };

  if (j.contains("cooperative")) {
    const std::string cctx = ctx + " cooperative";
    const Json& c = j.at("cooperative");
    check_object(c, cctx, {"protocol", "id", "horizontal_std_m", "vertical_std_m"});
    CooperativeBroadcast b;
    b.protocol = text(c, "protocol", cctx);
    b.identity = text(c, "id", cctx);
    b.horizontal_std_m = non_negative_or(c, "horizontal_std_m", 5.0, cctx);
    b.vertical_std_m = non_negative_or(c, "vertical_std_m", 10.0, cctx);
    Rng rng(mix_seed(seed, fnv1a64("navigation-error/" + std::to_string(id))));
    for (double& phase : b.error_phase) phase = rng.uniform(0.0, 2.0 * geo::kPi);
    target.cooperative = b;
  }
  return target;
}

// ---- sensors ----------------------------------------------------------------

Mount parse_mount(const Json& j, const std::string& ctx) {
  check_object(j, ctx, {"position_enu_m", "heading_deg", "pitch_deg", "roll_deg"});
  Mount m;
  m.position_enu = vec3_or(j, "position_enu_m", Vec3::Zero(), ctx);
  m.heading_rad = angle_or(j, "heading_deg", 0.0, ctx);
  m.pitch_rad = angle_or(j, "pitch_deg", 0.0, ctx);
  m.roll_rad = angle_or(j, "roll_deg", 0.0, ctx);
  return m;
}

SensorErrors parse_errors(const Json& j, const std::string& ctx) {
  check_object(j, ctx, {"position_offset_enu_m", "heading_bias_deg", "pitch_bias_deg", "roll_bias_deg",
                        "clock_offset_s", "clock_drift_ppm", "latency_mean_s", "latency_jitter_s",
                        "dropout_prob", "noise_scale"});
  SensorErrors e;
  e.position_offset_enu = vec3_or(j, "position_offset_enu_m", Vec3::Zero(), ctx);
  e.heading_bias_rad = angle_or(j, "heading_bias_deg", 0.0, ctx);
  e.pitch_bias_rad = angle_or(j, "pitch_bias_deg", 0.0, ctx);
  e.roll_bias_rad = angle_or(j, "roll_bias_deg", 0.0, ctx);
  e.clock_offset_s = number_or(j, "clock_offset_s", 0.0, ctx);
  e.clock_drift_ppm = number_or(j, "clock_drift_ppm", 0.0, ctx);
  e.latency_mean_s = non_negative_or(j, "latency_mean_s", 0.0, ctx);
  e.latency_jitter_s = non_negative_or(j, "latency_jitter_s", 0.0, ctx);
  e.dropout_prob = probability_or(j, "dropout_prob", 0.0, ctx);
  e.noise_scale = non_negative_or(j, "noise_scale", 1.0, ctx);
  return e;
}

RadarParams parse_radar(const Json& j, const std::string& ctx) {
  check_object(j, ctx, {"mode", "min_range_m", "max_range_m", "fov_az_deg", "min_el_deg", "max_el_deg",
                        "sigma_range_m", "sigma_az_deg", "sigma_el_deg", "sigma_range_rate_mps", "snr_ref_db",
                        "ref_range_m", "pfa", "doppler_notch_mps", "false_alarms_per_scan", "max_range_rate_mps"});
  RadarParams p;
  const std::string mode = text_or(j, "mode", "staring", ctx);
  if (mode == "staring") {
    p.mode = RadarParams::Mode::Staring;
  } else if (mode == "rotating") {
    p.mode = RadarParams::Mode::Rotating;
  } else {
    fail(ctx, "mode must be 'staring' or 'rotating', got '" + mode + "'");
  }
  p.min_range_m = number_or(j, "min_range_m", p.min_range_m, ctx);
  p.max_range_m = number_or(j, "max_range_m", p.max_range_m, ctx);
  p.fov_az_rad = angle_or(j, "fov_az_deg", p.fov_az_rad, ctx);
  p.min_el_rad = angle_or(j, "min_el_deg", p.min_el_rad, ctx);
  p.max_el_rad = angle_or(j, "max_el_deg", p.max_el_rad, ctx);
  p.sigma_range_m = non_negative_or(j, "sigma_range_m", p.sigma_range_m, ctx);
  p.sigma_az_rad = angle_or(j, "sigma_az_deg", p.sigma_az_rad, ctx);
  p.sigma_el_rad = angle_or(j, "sigma_el_deg", p.sigma_el_rad, ctx);
  p.sigma_range_rate_mps = non_negative_or(j, "sigma_range_rate_mps", p.sigma_range_rate_mps, ctx);
  p.snr_ref_db = number_or(j, "snr_ref_db", p.snr_ref_db, ctx);
  p.ref_range_m = number_or(j, "ref_range_m", p.ref_range_m, ctx);
  p.pfa = number_or(j, "pfa", p.pfa, ctx);
  p.doppler_notch_mps = non_negative_or(j, "doppler_notch_mps", p.doppler_notch_mps, ctx);
  p.false_alarms_per_scan = non_negative_or(j, "false_alarms_per_scan", p.false_alarms_per_scan, ctx);
  p.max_range_rate_mps = non_negative_or(j, "max_range_rate_mps", p.max_range_rate_mps, ctx);
  return p;
}

CameraParams parse_camera(const Json& j, const std::string& ctx) {
  check_object(j, ctx, {"width_px", "height_px", "fx_px", "fy_px", "cx_px", "cy_px", "k1", "k2", "sigma_px",
                        "min_size_px", "size_at_half_pd_px", "size_pd_slope_px", "pd_max", "max_range_m", "false_alarms_per_frame",
                        "class_accuracy", "size_noise_frac", "gimbal"});
  CameraParams p;
  p.width_px = static_cast<int>(j.contains("width_px") ? integer(j, "width_px", ctx) : p.width_px);
  p.height_px = static_cast<int>(j.contains("height_px") ? integer(j, "height_px", ctx) : p.height_px);
  p.fx_px = number_or(j, "fx_px", p.fx_px, ctx);
  p.fy_px = number_or(j, "fy_px", p.fx_px, ctx);  // square pixels unless told otherwise
  p.cx_px = number_or(j, "cx_px", 0.5 * p.width_px, ctx);
  p.cy_px = number_or(j, "cy_px", 0.5 * p.height_px, ctx);
  p.k1 = number_or(j, "k1", p.k1, ctx);
  p.k2 = number_or(j, "k2", p.k2, ctx);
  p.sigma_px = non_negative_or(j, "sigma_px", p.sigma_px, ctx);
  p.min_size_px = non_negative_or(j, "min_size_px", p.min_size_px, ctx);
  p.size_at_half_pd_px = number_or(j, "size_at_half_pd_px", p.size_at_half_pd_px, ctx);
  p.size_pd_slope_px = number_or(j, "size_pd_slope_px", p.size_pd_slope_px, ctx);
  p.pd_max = probability_or(j, "pd_max", p.pd_max, ctx);
  p.max_range_m = number_or(j, "max_range_m", p.max_range_m, ctx);
  p.false_alarms_per_frame = non_negative_or(j, "false_alarms_per_frame", p.false_alarms_per_frame, ctx);
  p.class_accuracy = probability_or(j, "class_accuracy", p.class_accuracy, ctx);
  p.size_noise_frac = non_negative_or(j, "size_noise_frac", p.size_noise_frac, ctx);
  if (j.contains("gimbal")) {
    const Json& keys = array_field(j, "gimbal", ctx);
    for (std::size_t i = 0; i < keys.size(); ++i) {
      const std::string kctx = ctx + " gimbal key " + std::to_string(i);
      const Json& k = keys.at(i);
      check_object(k, kctx, {"t_s", "pan_deg", "tilt_deg"});
      p.gimbal.push_back({number(k, "t_s", kctx), deg2rad(number_or(k, "pan_deg", 0.0, kctx)),
                          deg2rad(number_or(k, "tilt_deg", 0.0, kctx))});
    }
  }
  return p;
}

RfBearingParams parse_rf_bearing(const Json& j, const std::string& ctx) {
  check_object(j, ctx, {"max_range_m", "fov_az_deg", "sigma_az_deg", "pd", "snr_ref_db", "ref_range_m",
                        "false_alarms_per_report", "signature_id_prob"});
  RfBearingParams p;
  p.max_range_m = number_or(j, "max_range_m", p.max_range_m, ctx);
  p.fov_az_rad = angle_or(j, "fov_az_deg", p.fov_az_rad, ctx);
  p.sigma_az_rad = angle_or(j, "sigma_az_deg", p.sigma_az_rad, ctx);
  p.pd = probability_or(j, "pd", p.pd, ctx);
  p.snr_ref_db = number_or(j, "snr_ref_db", p.snr_ref_db, ctx);
  p.ref_range_m = number_or(j, "ref_range_m", p.ref_range_m, ctx);
  p.false_alarms_per_report = non_negative_or(j, "false_alarms_per_report", p.false_alarms_per_report, ctx);
  p.signature_id_prob = probability_or(j, "signature_id_prob", p.signature_id_prob, ctx);
  return p;
}

CooperativeParams parse_cooperative(const Json& j, const std::string& ctx) {
  check_object(j, ctx, {"protocol", "max_range_m", "reception_prob", "velocity_std_mps"});
  CooperativeParams p;
  p.protocol = text_or(j, "protocol", p.protocol, ctx);
  p.max_range_m = number_or(j, "max_range_m", p.max_range_m, ctx);
  p.reception_prob = probability_or(j, "reception_prob", p.reception_prob, ctx);
  p.velocity_std_mps = non_negative_or(j, "velocity_std_mps", p.velocity_std_mps, ctx);
  return p;
}

}  // namespace

std::unique_ptr<Sensor> parse_sensor(const Json& j) {
  std::string ctx = "sensor";
  if (j.is_object() && j.contains("id") && j.at("id").is_string()) ctx += " '" + j.at("id").get<std::string>() + "'";
  check_object(j, ctx, {"id", "type", "rate_hz", "phase_s", "timestamps", "mount", "errors", "radar", "camera",
                        "rf_bearing", "cooperative"});

  SensorCommon c;
  c.id = text(j, "id", ctx);
  const std::string type_name = text(j, "type", ctx);
  const auto type = sensor_type_from_string(type_name);
  if (!type) fail(ctx, "type must be 'radar', 'camera', 'rf_bearing' or 'cooperative', got '" + type_name + "'");
  for (const char* block : {"radar", "camera", "rf_bearing", "cooperative"}) {
    if (j.contains(block) && type_name != block) {
      fail(ctx, std::string("has a '") + block + "' block but its type is '" + type_name + "'");
    }
  }

  c.rate_hz = number(j, "rate_hz", ctx);
  require(c.rate_hz > 0.0, ctx, "rate_hz must be positive");
  c.phase_s = non_negative_or(j, "phase_s", 0.0, ctx);
  const std::string timestamps = text_or(j, "timestamps", "sensor", ctx);
  if (timestamps == "sensor") {
    c.timestamps = TimestampSource::Sensor;
  } else if (timestamps == "arrival") {
    c.timestamps = TimestampSource::Arrival;
  } else {
    fail(ctx, "timestamps must be 'sensor' or 'arrival', got '" + timestamps + "'");
  }
  c.mount = parse_mount(object_or_empty(j, "mount"), ctx + " mount");
  c.errors = parse_errors(object_or_empty(j, "errors"), ctx + " errors");

  const Json& params = object_or_empty(j, type_name.c_str());
  const std::string pctx = ctx + " " + type_name;
  switch (*type) {
    case SensorType::Radar:
      return std::make_unique<Radar>(std::move(c), parse_radar(params, pctx));
    case SensorType::Camera:
      return std::make_unique<Camera>(std::move(c), parse_camera(params, pctx));
    case SensorType::RfBearing:
      return std::make_unique<RfBearingSensor>(std::move(c), parse_rf_bearing(params, pctx));
    case SensorType::Cooperative:
      return std::make_unique<CooperativeReceiver>(std::move(c), parse_cooperative(params, pctx));
  }
  fail(ctx, "unhandled sensor type");
}

geo::LocalFrame parse_origin(const Json& j) {
  const std::string ctx = "origin";
  check_object(j, ctx, {"lat_deg", "lon_deg", "alt_m"});
  return geo::LocalFrame(geo::Geodetic{deg2rad(number(j, "lat_deg", ctx)), deg2rad(number(j, "lon_deg", ctx)),
                                       number_or(j, "alt_m", 0.0, ctx)});
}

Scenario parse_scenario(const Json& j) {
  const std::string ctx = "scenario";
  check_object(j, ctx, {"name", "description", "seed", "duration_s", "truth_rate_hz", "origin", "targets", "sensors"});

  Scenario sc;
  sc.source = j;
  sc.name = text_or(j, "name", "unnamed", ctx);
  if (j.contains("seed")) {
    const Json& seed = j.at("seed");
    const bool ok = seed.is_number_unsigned() || (seed.is_number_integer() && seed.get<std::int64_t>() >= 0);
    require(ok, ctx, "seed must be a non-negative integer");
    sc.seed = seed.get<std::uint64_t>();
  }
  sc.duration_s = number(j, "duration_s", ctx);
  require(sc.duration_s > 0.0, ctx, "duration_s must be positive");
  sc.truth_rate_hz = number_or(j, "truth_rate_hz", 10.0, ctx);
  require(sc.truth_rate_hz > 0.0, ctx, "truth_rate_hz must be positive");

  if (!j.contains("origin")) fail(ctx, "missing field 'origin'");
  sc.frame = parse_origin(j.at("origin"));

  std::set<int> target_ids;
  for (const Json& t : array_field(j, "targets", ctx)) {
    sc.targets.push_back(parse_target(t, sc.duration_s, sc.seed));
    if (!target_ids.insert(sc.targets.back().id).second) {
      fail(ctx, "two targets share id " + std::to_string(sc.targets.back().id));
    }
  }
  std::set<std::string> sensor_ids;
  for (const Json& s : array_field(j, "sensors", ctx)) {
    sc.sensors.push_back(parse_sensor(s));
    if (!sensor_ids.insert(sc.sensors.back()->id()).second) {
      fail(ctx, "two sensors share id '" + sc.sensors.back()->id() + "'");
    }
  }
  return sc;
}

Json read_json_file(const std::filesystem::path& path) {
  std::ifstream file(path);
  if (!file) throw std::runtime_error("cannot open " + path.string());
  try {
    return Json::parse(file);
  } catch (const Json::parse_error& e) {
    throw std::runtime_error(path.string() + ": " + e.what());
  }
}

Scenario load_scenario(const std::filesystem::path& path) {
  try {
    return parse_scenario(read_json_file(path));
  } catch (const Json::exception& e) {
    throw std::runtime_error(path.string() + ": " + e.what());
  }
}

}  // namespace skytrack

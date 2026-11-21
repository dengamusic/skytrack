#include "skytrack/measurements.hpp"

#include <stdexcept>

namespace skytrack {

std::string_view to_string(SensorType type) {
  switch (type) {
    case SensorType::Radar: return "radar";
    case SensorType::Camera: return "camera";
    case SensorType::RfBearing: return "rf_bearing";
    case SensorType::Cooperative: return "cooperative";
  }
  return "unknown";
}

std::optional<SensorType> sensor_type_from_string(std::string_view name) {
  if (name == "radar") return SensorType::Radar;
  if (name == "camera") return SensorType::Camera;
  if (name == "rf_bearing") return SensorType::RfBearing;
  if (name == "cooperative") return SensorType::Cooperative;
  return std::nullopt;
}

namespace {

Json optional_to_json(const std::optional<double>& value) {
  return value ? Json(*value) : Json(nullptr);
}

std::optional<double> optional_from_json(const Json& j, const char* key) {
  const auto it = j.find(key);
  if (it == j.end() || it->is_null()) return std::nullopt;
  return it->get<double>();
}

double get_double(const Json& j, const char* key) { return j.at(key).get<double>(); }

struct PayloadWriter {
  Json& out;

  void operator()(const RadarDetection& d) const {
    out["range_m"] = d.range_m;
    out["az_rad"] = d.az_rad;
    out["el_rad"] = d.el_rad;
    out["range_rate_mps"] = d.range_rate_mps;
    out["snr_db"] = d.snr_db;
  }
  void operator()(const CameraDetection& d) const {
    out["u_px"] = d.u_px;
    out["v_px"] = d.v_px;
    out["width_px"] = d.width_px;
    out["height_px"] = d.height_px;
    out["label"] = d.label;
    out["confidence"] = d.confidence;
  }
  void operator()(const BearingDetection& d) const {
    out["az_rad"] = d.az_rad;
    out["snr_db"] = d.snr_db;
    out["signature"] = d.signature;
  }
  void operator()(const CooperativeReport& d) const {
    out["protocol"] = d.protocol;
    out["identity"] = d.identity;
    out["lat_deg"] = d.lat_deg;
    out["lon_deg"] = d.lon_deg;
    out["alt_m"] = d.alt_m;
    out["vel_east_mps"] = d.vel_east_mps;
    out["vel_north_mps"] = d.vel_north_mps;
    out["vel_up_mps"] = d.vel_up_mps;
    if (d.accuracy_h_m) out["accuracy_h_m"] = *d.accuracy_h_m;
    if (d.accuracy_v_m) out["accuracy_v_m"] = *d.accuracy_v_m;
  }
};

MeasurementPayload payload_from_json(SensorType type, const Json& j) {
  switch (type) {
    case SensorType::Radar:
      return RadarDetection{get_double(j, "range_m"), get_double(j, "az_rad"), get_double(j, "el_rad"),
                            get_double(j, "range_rate_mps"), get_double(j, "snr_db")};
    case SensorType::Camera:
      return CameraDetection{get_double(j, "u_px"), get_double(j, "v_px"), get_double(j, "width_px"),
                             get_double(j, "height_px"), j.at("label").get<std::string>(),
                             get_double(j, "confidence")};
    case SensorType::RfBearing:
      return BearingDetection{get_double(j, "az_rad"), get_double(j, "snr_db"),
                              j.at("signature").get<std::string>()};
    case SensorType::Cooperative: {
      CooperativeReport r;
      r.protocol = j.at("protocol").get<std::string>();
      r.identity = j.at("identity").get<std::string>();
      r.lat_deg = get_double(j, "lat_deg");
      r.lon_deg = get_double(j, "lon_deg");
      r.alt_m = get_double(j, "alt_m");
      r.vel_east_mps = get_double(j, "vel_east_mps");
      r.vel_north_mps = get_double(j, "vel_north_mps");
      r.vel_up_mps = get_double(j, "vel_up_mps");
      if (j.contains("accuracy_h_m")) r.accuracy_h_m = get_double(j, "accuracy_h_m");
      if (j.contains("accuracy_v_m")) r.accuracy_v_m = get_double(j, "accuracy_v_m");
      return r;
    }
  }
  throw std::logic_error("unhandled sensor type");
}

}  // namespace

Json to_observed_json(const SensorMessage& msg) {
  Json j;
  j["sensor_id"] = msg.sensor_id;
  j["sensor_type"] = std::string(to_string(msg.sensor_type));
  j["seq"] = msg.seq;
  j["t_arrival_s"] = msg.t_arrival_s;
  j["t_sensor_s"] = optional_to_json(msg.t_sensor_s);
  if (msg.gimbal) {
    j["gimbal"] = {{"pan_rad", msg.gimbal->pan_rad}, {"tilt_rad", msg.gimbal->tilt_rad}};
  } else {
    j["gimbal"] = nullptr;
  }
  Json detections = Json::array();
  for (const Detection& d : msg.detections) {
    Json jd;
    jd["meas_id"] = d.meas_id;
    jd["t_sensor_s"] = optional_to_json(d.t_sensor_s);
    std::visit(PayloadWriter{jd}, d.payload);
    detections.push_back(std::move(jd));
  }
  j["detections"] = std::move(detections);
  return j;
}

SensorMessage from_observed_json(const Json& j) {
  SensorMessage msg;
  msg.sensor_id = j.at("sensor_id").get<std::string>();
  const std::string type_name = j.at("sensor_type").get<std::string>();
  const auto type = sensor_type_from_string(type_name);
  if (!type) throw std::runtime_error("unknown sensor_type '" + type_name + "'");
  msg.sensor_type = *type;
  msg.seq = j.at("seq").get<std::uint64_t>();
  msg.t_arrival_s = j.at("t_arrival_s").get<double>();
  msg.t_sensor_s = optional_from_json(j, "t_sensor_s");
  if (const auto g = j.find("gimbal"); g != j.end() && !g->is_null()) {
    msg.gimbal = GimbalReport{g->at("pan_rad").get<double>(), g->at("tilt_rad").get<double>()};
  }
  for (const Json& jd : j.at("detections")) {
    Detection d;
    d.meas_id = jd.at("meas_id").get<std::int64_t>();
    d.t_sensor_s = optional_from_json(jd, "t_sensor_s");
    d.payload = payload_from_json(msg.sensor_type, jd);
    msg.detections.push_back(std::move(d));
  }
  return msg;
}

}  // namespace skytrack

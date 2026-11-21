#include "skytrack/tracking/config.hpp"

#include <string>
#include <string_view>
#include <vector>

#include "skytrack/json_fields.hpp"

namespace skytrack {

using namespace json;

namespace {

double positive_or(const Json& j, const char* key, double fallback, const std::string& ctx) {
  const double v = number_or(j, key, fallback, ctx);
  require(v > 0.0, ctx, std::string(key) + " must be positive");
  return v;
}

int positive_int_or(const Json& j, const char* key, int fallback, const std::string& ctx) {
  if (!j.contains(key)) return fallback;
  const std::int64_t v = integer(j, key, ctx);
  require(v > 0 && v < 1000, ctx, std::string(key) + " must be a positive integer");
  return static_cast<int>(v);
}

/// The scalar settings, listed once: parsing, the allowed-field check and
/// serialisation all come from these tables, so they cannot drift apart.
enum class Check { Any, NonNegative, Positive, Probability };

struct NumberField {
  const char* name;
  double TrackerConfig::*member;
  Check check;
};

constexpr NumberField kNumbers[] = {
    {"reorder_window_s", &TrackerConfig::reorder_window_s, Check::NonNegative},
    {"max_measurement_age_s", &TrackerConfig::max_measurement_age_s, Check::NonNegative},
    {"late_initiation_s", &TrackerConfig::late_initiation_s, Check::NonNegative},
    {"output_rate_hz", &TrackerConfig::output_rate_hz, Check::Positive},
    {"init_velocity_std_mps", &TrackerConfig::init_velocity_std_mps, Check::Positive},
    {"cooperative_std_h_m", &TrackerConfig::cooperative_std_h_m, Check::Positive},
    {"cooperative_std_v_m", &TrackerConfig::cooperative_std_v_m, Check::Positive},
    {"cooperative_white_std_m", &TrackerConfig::cooperative_white_std_m, Check::Positive},
    {"broadcast_error_correlation_s", &TrackerConfig::broadcast_error_correlation_s, Check::NonNegative},
    {"same_identity_gate_scale", &TrackerConfig::same_identity_gate_scale, Check::Positive},
    {"tentative_coast_s", &TrackerConfig::tentative_coast_s, Check::Positive},
    {"confirmed_coast_s", &TrackerConfig::confirmed_coast_s, Check::Positive},
    {"max_position_std_m", &TrackerConfig::max_position_std_m, Check::Positive},
    {"merge_gate_chi2", &TrackerConfig::merge_gate_chi2, Check::Positive},
    {"ghost_direction_deg", &TrackerConfig::ghost_direction_deg, Check::NonNegative},
    {"ghost_live_radar_s", &TrackerConfig::ghost_live_radar_s, Check::NonNegative},
    {"ghost_stale_radar_s", &TrackerConfig::ghost_stale_radar_s, Check::NonNegative},
    {"radar_sector_s", &TrackerConfig::radar_sector_s, Check::Positive},
    {"radar_assumed_rcs_dbsm", &TrackerConfig::radar_assumed_rcs_dbsm, Check::Any},
    {"radar_min_expected_pd", &TrackerConfig::radar_min_expected_pd, Check::Probability},
    {"radar_max_assumed_pd", &TrackerConfig::radar_max_assumed_pd, Check::Probability},
    {"notch_inference_probability", &TrackerConfig::notch_inference_probability, Check::Probability},
    {"notch_evidence_s", &TrackerConfig::notch_evidence_s, Check::NonNegative},
    {"notch_gate_chi2", &TrackerConfig::notch_gate_chi2, Check::Positive},
    {"radar_conflict_deg", &TrackerConfig::radar_conflict_deg, Check::NonNegative},
    {"refute_probability", &TrackerConfig::refute_probability, Check::Probability},
    {"class_vote_rf_signature", &TrackerConfig::class_vote_rf_signature, Check::NonNegative},
    {"class_vote_broadcast", &TrackerConfig::class_vote_broadcast, Check::NonNegative},
    {"mount_accuracy_deg", &TrackerConfig::mount_accuracy_deg, Check::NonNegative},
    {"clock_accuracy_s", &TrackerConfig::clock_accuracy_s, Check::NonNegative},
    {"mount_drift_deg_per_sqrt_h", &TrackerConfig::mount_drift_deg_per_sqrt_h, Check::NonNegative},
    {"clock_drift_s_per_sqrt_h", &TrackerConfig::clock_drift_s_per_sqrt_h, Check::NonNegative},
    {"registration_reference_max_age_s", &TrackerConfig::registration_reference_max_age_s, Check::NonNegative},
};

struct IntField {
  const char* name;
  int TrackerConfig::*member;
};

constexpr IntField kInts[] = {
    {"confirm_hits", &TrackerConfig::confirm_hits},
    {"confirm_window", &TrackerConfig::confirm_window},
    {"cooperative_confirm_hits", &TrackerConfig::cooperative_confirm_hits},
};

struct BoolField {
  const char* name;
  bool TrackerConfig::*member;
};

constexpr BoolField kBools[] = {
    {"merge_duplicates", &TrackerConfig::merge_duplicates},
    {"registration", &TrackerConfig::registration},
    {"consider_registration", &TrackerConfig::consider_registration},
};

double read_number(const Json& j, const NumberField& f, double fallback, const std::string& ctx) {
  switch (f.check) {
    case Check::Any: return number_or(j, f.name, fallback, ctx);
    case Check::NonNegative: return non_negative_or(j, f.name, fallback, ctx);
    case Check::Positive: return positive_or(j, f.name, fallback, ctx);
    case Check::Probability: return probability_or(j, f.name, fallback, ctx);
  }
  return fallback;
}

}  // namespace

TrackerConfig parse_tracker_config(const Json& j) {
  const std::string ctx = "tracker config";
  std::vector<std::string_view> allowed{"process_noise", "sensors"};
  for (const auto& f : kNumbers) allowed.emplace_back(f.name);
  for (const auto& f : kInts) allowed.emplace_back(f.name);
  for (const auto& f : kBools) allowed.emplace_back(f.name);
  check_object(j, ctx, allowed);

  TrackerConfig c;
  for (const auto& f : kNumbers) c.*f.member = read_number(j, f, c.*f.member, ctx);
  for (const auto& f : kInts) c.*f.member = positive_int_or(j, f.name, c.*f.member, ctx);
  for (const auto& f : kBools) {
    if (!j.contains(f.name)) continue;
    require(j.at(f.name).is_boolean(), ctx, std::string(f.name) + " must be true or false");
    c.*f.member = j.at(f.name).get<bool>();
  }
  require(c.confirm_hits <= c.confirm_window, ctx, "confirm_hits cannot exceed confirm_window");

  const Json& q = object_or_empty(j, "process_noise");
  const std::string qctx = "tracker config process_noise";
  check_object(q, qctx, {"horizontal_m2_s3", "vertical_m2_s3"});
  c.process_noise.horizontal_m2_s3 = non_negative_or(q, "horizontal_m2_s3", c.process_noise.horizontal_m2_s3, qctx);
  c.process_noise.vertical_m2_s3 = non_negative_or(q, "vertical_m2_s3", c.process_noise.vertical_m2_s3, qctx);

  const Json& sensors = object_or_empty(j, "sensors");
  require(sensors.is_object(), ctx, "sensors must be an object keyed by sensor id");
  for (auto it = sensors.begin(); it != sensors.end(); ++it) {
    const std::string sctx = "tracker config sensor '" + it.key() + "'";
    const Json& s = it.value();
    check_object(s, sctx, {"clock_offset_s", "assumed_latency_s", "position_std_h_m", "position_std_v_m"});
    SensorOverrides o;
    o.clock_offset_s = number_or(s, "clock_offset_s", 0.0, sctx);
    o.assumed_latency_s = non_negative_or(s, "assumed_latency_s", 0.0, sctx);
    if (s.contains("position_std_h_m")) o.position_std_h_m = positive_or(s, "position_std_h_m", 1.0, sctx);
    if (s.contains("position_std_v_m")) o.position_std_v_m = positive_or(s, "position_std_v_m", 1.0, sctx);
    c.sensors[it.key()] = o;
  }
  return c;
}

Json to_json(const TrackerConfig& c) {
  Json j;
  for (const auto& f : kNumbers) j[f.name] = c.*f.member;
  for (const auto& f : kInts) j[f.name] = c.*f.member;
  for (const auto& f : kBools) j[f.name] = c.*f.member;
  j["process_noise"] = {{"horizontal_m2_s3", c.process_noise.horizontal_m2_s3},
                        {"vertical_m2_s3", c.process_noise.vertical_m2_s3}};
  Json sensors = Json::object();
  for (const auto& [id, o] : c.sensors) {
    Json s;
    s["clock_offset_s"] = o.clock_offset_s;
    s["assumed_latency_s"] = o.assumed_latency_s;
    if (o.position_std_h_m) s["position_std_h_m"] = *o.position_std_h_m;
    if (o.position_std_v_m) s["position_std_v_m"] = *o.position_std_v_m;
    sensors[id] = std::move(s);
  }
  j["sensors"] = std::move(sensors);
  return j;
}

}  // namespace skytrack

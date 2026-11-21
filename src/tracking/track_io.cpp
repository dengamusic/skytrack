#include "skytrack/tracking/track_io.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

namespace skytrack {
namespace {

namespace fs = std::filesystem;

std::ofstream open_for_write(const fs::path& path) {
  std::ofstream file(path);
  if (!file) throw std::runtime_error("cannot write " + path.string());
  file << std::setprecision(10);
  return file;
}

std::vector<std::string> split_csv_line(const std::string& line) {
  std::vector<std::string> fields;
  std::string field;
  std::istringstream in(line);
  while (std::getline(in, field, ',')) fields.push_back(field);
  if (!line.empty() && line.back() == ',') fields.emplace_back();
  return fields;
}

/// The full state covariance goes out as its upper triangle (21 values),
/// p_<row>_<col> over (e, n, u, ve, vn, vu): a fusion node downstream needs
/// the position-velocity cross terms, not just the variances.
constexpr std::array<const char*, 6> kStateNames{"e", "n", "u", "ve", "vn", "vu"};
constexpr std::size_t kCovarianceFields = 21;
constexpr std::size_t kFirstCovarianceField = 12;
constexpr std::size_t kTracksFields = kFirstCovarianceField + kCovarianceFields + 3;

const std::string& tracks_header() {
  static const std::string header = [] {
    std::string h = "t_s,track_id,status,class,identity,emitter,east_m,north_m,up_m,vel_east_mps,vel_north_mps,vel_up_mps";
    for (std::size_t i = 0; i < 6; ++i) {
      for (std::size_t j = i; j < 6; ++j) h += std::string(",p_") + kStateNames[i] + "_" + kStateNames[j];
    }
    return h + ",last_update_s,sensors,published_s";
  }();
  return header;
}

/// Mean and percentiles of a sample, for latency and timing summaries.
Json distribution(std::vector<double> v) {
  if (v.empty()) return Json::object();
  std::sort(v.begin(), v.end());
  const auto at = [&](double q) { return v[static_cast<std::size_t>(q * static_cast<double>(v.size() - 1))]; };
  double sum = 0.0;
  for (const double x : v) sum += x;
  return {{"mean", sum / static_cast<double>(v.size())}, {"p50", at(0.5)}, {"p95", at(0.95)}, {"p99", at(0.99)},
          {"max", v.back()}};
}

}  // namespace

MeasurementLog read_measurements(const fs::path& path) {
  constexpr std::size_t kErrorsKept = 5;
  std::ifstream file(path);
  if (!file) throw std::runtime_error("cannot open " + path.string());
  MeasurementLog log;
  std::string line;
  std::size_t line_no = 0;
  while (std::getline(file, line)) {
    ++line_no;
    if (line.empty()) continue;
    try {
      log.messages.push_back(from_observed_json(Json::parse(line)));
    } catch (const std::exception& e) {
      ++log.bad_lines;
      if (log.first_errors.size() < kErrorsKept) {
        log.first_errors.push_back(path.string() + ":" + std::to_string(line_no) + ": " + e.what());
      }
    }
  }
  return log;
}

Tracker run_tracker(std::shared_ptr<const SensorSetup> setup, const TrackerConfig& config,
                    const std::vector<SensorMessage>& messages) {
  Tracker tracker(std::move(setup), config);
  for (const SensorMessage& msg : messages) tracker.ingest(msg);
  tracker.finish();
  return tracker;
}

Json report_latency_json(const Tracker& tracker) {
  // Confirmed reports only, and not the final flush at the end of the data,
  // which is published all at once whatever its age.
  std::vector<double> latency;
  const double end = tracker.reports().empty() ? 0.0 : tracker.reports().back().t_published_s;
  for (const TrackReport& r : tracker.reports()) {
    if (r.status == TrackStatus::Confirmed && r.t_published_s < end) latency.push_back(r.t_published_s - r.t_s);
  }
  return distribution(std::move(latency));
}

Json processing_time_json(const Tracker& tracker) {
  const auto& us = tracker.ingest_time_us();
  Json j = distribution(std::vector<double>(us.begin(), us.end()));
  double total_s = 0.0;
  for (const float x : us) total_s += 1e-6 * x;
  j["total_s"] = total_s;
  return j;
}

Json tracker_summary_json(const Tracker& tracker) {
  const TrackerStats& s = tracker.stats();
  Json j;
  j["config"] = to_json(tracker.config());
  j["messages"] = s.messages;
  j["measurements"] = s.measurements;
  j["updates"] = s.updates;
  j["late_updates"] = s.late_updates;
  j["initiations"] = s.initiations;
  j["unassigned"] = s.unassigned;
  j["too_old"] = s.too_old;
  j["tracks_confirmed"] = s.tracks_confirmed;
  j["tracks_merged"] = s.tracks_merged;
  j["tracks_deleted"] = s.tracks_deleted;
  j["notch_inferences"] = s.notch_inferences;
  j["notch_rejections"] = s.notch_rejections;
  j["inconsistent_innovations"] = s.inconsistent_innovations;
  j["covariance_repairs"] = s.covariance_repairs;
  j["tracks_refuted"] = s.tracks_refuted;
  j["ghosts_removed"] = s.ghosts_removed;
  j["registration_samples"] = s.registration_samples;
  j["rejected_messages"] = s.rejected_messages;
  j["rejected_measurements"] = s.rejected_measurements;

  Json health = Json::object();
  for (const auto& [sensor, h] : tracker.health()) {
    health[sensor] = {{"messages", h.messages},
                      {"rate_hz", h.rate_hz()},
                      {"detections", h.detections},
                      {"lost_messages", h.lost_messages},
                      {"out_of_order", h.out_of_order},
                      {"late_messages", h.late_messages},
                      {"rejected_detections", h.rejected},
                      {"mean_latency_s", h.mean_latency_s()},
                      {"max_latency_s", h.latency_max_s}};
  }
  j["sensor_health"] = std::move(health);
  j["report_latency_s"] = report_latency_json(tracker);
  Json registration = Json::object();
  for (const auto& [sensor, e] : tracker.registration().estimates()) {
    registration[sensor] = {{"heading_bias_deg", geo::rad2deg(e.bias[0])},
                            {"pitch_bias_deg", geo::rad2deg(e.bias[1])},
                            {"clock_offset_s", e.bias[2]},
                            {"heading_std_deg", geo::rad2deg(std::sqrt(e.P(0, 0)))},
                            {"pitch_std_deg", geo::rad2deg(std::sqrt(e.P(1, 1)))},
                            {"clock_std_s", std::sqrt(e.P(2, 2))},
                            {"samples", e.samples},
                            {"rejected", e.rejected}};
  }
  j["registration"] = std::move(registration);
  if (const auto span = tracker.output_span()) {
    j["first_output_s"] = span->first;
    j["last_output_s"] = span->second;
  }
  j["output_rate_hz"] = tracker.config().output_rate_hz;
  return j;
}

void write_track_reports(const std::vector<TrackReport>& reports, const fs::path& path) {
  auto file = open_for_write(path);
  file << tracks_header() << '\n';
  for (const TrackReport& r : reports) {
    const Vec6& x = r.state.x;
    const Mat6& P = r.state.P;
    file << r.t_s << ',' << r.track_id << ',' << to_string(r.status) << ',' << r.classification << ','
         << r.identity << ',' << r.emitter;
    for (int k = 0; k < 6; ++k) file << ',' << x[k];
    for (int i = 0; i < 6; ++i) {
      for (int j = i; j < 6; ++j) file << ',' << P(i, j);
    }
    file << ',' << r.last_update_s << ',' << r.sensors << ',' << r.t_published_s << '\n';
  }
}

void write_tracker_output(const Tracker& tracker, const fs::path& dir) {
  fs::create_directories(dir);
  write_track_reports(tracker.reports(), dir / "tracks.csv");
  {
    auto file = open_for_write(dir / "associations.csv");
    file << "meas_id,sensor_id,t_s,track_id,outcome,d2\n";
    for (const AssociationRecord& a : tracker.associations()) {
      file << a.meas_id << ',' << a.sensor_id << ',' << a.t_s << ',' << a.track_id << ',' << to_string(a.outcome)
           << ',' << a.d2 << '\n';
    }
  }
  {
    auto file = open_for_write(dir / "summary.json");
    file << tracker_summary_json(tracker).dump(2) << '\n';
  }
  {
    // Kept apart from summary.json, which is deterministic: timings are not.
    auto file = open_for_write(dir / "timing.json");
    file << Json{{"ingest_us", processing_time_json(tracker)}}.dump(2) << '\n';
  }
}

std::vector<TrackReport> read_track_reports(const fs::path& path) {
  std::ifstream file(path);
  if (!file) throw std::runtime_error("cannot open " + path.string());
  std::string line;
  if (!std::getline(file, line) || line != tracks_header()) {
    throw std::runtime_error(path.string() + ": not a tracks.csv written by this version");
  }
  std::vector<TrackReport> reports;
  std::size_t line_no = 1;
  while (std::getline(file, line)) {
    ++line_no;
    if (line.empty()) continue;
    const auto f = split_csv_line(line);
    if (f.size() != kTracksFields) {
      throw std::runtime_error(path.string() + ":" + std::to_string(line_no) + ": expected " +
                               std::to_string(kTracksFields) + " fields");
    }
    try {
      TrackReport r;
      r.t_s = std::stod(f[0]);
      r.track_id = std::stoi(f[1]);
      r.status = f[2] == "confirmed" ? TrackStatus::Confirmed : TrackStatus::Tentative;
      r.classification = f[3];
      r.identity = f[4];
      r.emitter = f[5];
      for (int k = 0; k < 6; ++k) r.state.x[k] = std::stod(f[static_cast<std::size_t>(6 + k)]);
      Mat6& P = r.state.P;
      std::size_t field = kFirstCovarianceField;
      for (int i = 0; i < 6; ++i) {
        for (int j = i; j < 6; ++j) P(i, j) = P(j, i) = std::stod(f[field++]);
      }
      r.last_update_s = std::stod(f[field++]);
      r.sensors = f[field++];
      r.t_published_s = std::stod(f[field]);
      reports.push_back(std::move(r));
    } catch (const std::logic_error&) {
      throw std::runtime_error(path.string() + ":" + std::to_string(line_no) + ": bad number");
    }
  }
  return reports;
}

}  // namespace skytrack

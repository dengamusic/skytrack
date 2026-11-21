// Runs the tracker over a simulator run (or any directory with sensors.json
// and measurements.jsonl) and writes the tracks next to it.

#include <algorithm>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>

#include "skytrack/scenario.hpp"
#include "skytrack/tracking/track_io.hpp"

namespace {

int usage() {
  std::fprintf(stderr,
               "usage: skytrack_track <run-dir> [--config tracker.json] [--sensors ID,ID,...] [--out DIR]\n"
               "  reads  <run-dir>/sensors.json and <run-dir>/measurements.jsonl\n"
               "  --sensors: use only these sensors' messages, as one node of a distributed system would\n"
               "  writes <run-dir>/tracker/ (or DIR): tracks.csv, associations.csv, summary.json, timing.json\n");
  return 2;
}

}  // namespace

int main(int argc, char** argv) {
  namespace fs = std::filesystem;
  using namespace skytrack;
  if (argc < 2) return usage();
  const fs::path run_dir = argv[1];
  fs::path out_dir = run_dir / "tracker";
  std::optional<fs::path> config_path;
  std::set<std::string> only_sensors;
  for (int i = 2; i < argc; ++i) {
    const std::string_view arg = argv[i];
    if (arg == "--config" && i + 1 < argc) {
      config_path = argv[++i];
    } else if (arg == "--sensors" && i + 1 < argc) {
      std::string_view rest = argv[++i];
      while (!rest.empty()) {
        const auto comma = rest.find(',');
        if (comma != 0) only_sensors.insert(std::string(rest.substr(0, comma)));
        rest = comma == std::string_view::npos ? std::string_view{} : rest.substr(comma + 1);
      }
    } else if (arg == "--out" && i + 1 < argc) {
      out_dir = argv[++i];
    } else {
      return usage();
    }
  }

  try {
    const TrackerConfig config = config_path ? parse_tracker_config(read_json_file(*config_path)) : TrackerConfig{};
    const auto setup = std::make_shared<const SensorSetup>(SensorSetup::load(run_dir / "sensors.json", config));
    MeasurementLog log = read_measurements(run_dir / "measurements.jsonl");
    if (!only_sensors.empty()) {
      for (const std::string& id : only_sensors) {
        if (setup->find(id) == nullptr) throw std::runtime_error("--sensors: no sensor '" + id + "' in sensors.json");
      }
      std::erase_if(log.messages, [&](const SensorMessage& m) { return !only_sensors.contains(m.sensor_id); });
    }
    if (log.bad_lines > 0) {
      std::fprintf(stderr, "warning: skipped %zu corrupt line(s) of measurements.jsonl\n", log.bad_lines);
      for (const std::string& e : log.first_errors) std::fprintf(stderr, "  %s\n", e.c_str());
    }
    const Tracker tracker = run_tracker(setup, config, log.messages);
    write_tracker_output(tracker, out_dir);

    const TrackerStats& s = tracker.stats();
    std::printf("%llu messages, %llu measurements: %llu track updates (%llu late), %llu new tracks, %llu unused\n",
                static_cast<unsigned long long>(s.messages), static_cast<unsigned long long>(s.measurements),
                static_cast<unsigned long long>(s.updates), static_cast<unsigned long long>(s.late_updates),
                static_cast<unsigned long long>(s.initiations),
                static_cast<unsigned long long>(s.unassigned + s.too_old));
    std::printf("%d tracks confirmed, %d merged, %d deleted\n", s.tracks_confirmed, s.tracks_merged,
                s.tracks_deleted);
    if (s.rejected_messages + s.rejected_measurements > 0) {
      std::printf("rejected: %llu messages from unknown sensors, %llu detections with non-finite values\n",
                  static_cast<unsigned long long>(s.rejected_messages),
                  static_cast<unsigned long long>(s.rejected_measurements));
    }

    // Sensor health: what the data stream itself says about each sensor.
    std::printf("\n%-12s %8s %8s %6s %6s %13s %13s\n", "sensor", "messages", "rate_hz", "lost", "late",
                "latency_mean", "latency_max");
    for (const auto& [id, h] : tracker.health()) {
      std::printf("%-12s %8llu %8.2f %6llu %6llu %12.2fs %12.2fs\n", id.c_str(),
                  static_cast<unsigned long long>(h.messages), h.rate_hz(),
                  static_cast<unsigned long long>(h.lost_messages), static_cast<unsigned long long>(h.late_messages),
                  h.mean_latency_s(), h.latency_max_s);
    }

    const Json latency = report_latency_json(tracker);
    const Json timing = processing_time_json(tracker);
    if (latency.contains("mean")) {
      std::printf("\nconfirmed reports are %.2f s old when published (p95 %.2f s)\n", latency["mean"].get<double>(),
                  latency["p95"].get<double>());
    }
    if (const auto span = tracker.output_span(); span && timing.contains("p99")) {
      const double total = timing["total_s"].get<double>();
      std::printf("processing: %.3f s for %.0f s of data (%.0fx real time); per message p50 %.0f us, p99 %.0f us\n\n",
                  total, span->second - span->first, (span->second - span->first) / std::max(total, 1e-9),
                  timing["p50"].get<double>(), timing["p99"].get<double>());
    }

    // One line per confirmed track: lifetime and what it became.
    struct Life {
      double first = 0.0, last = 0.0;
      std::string cls, identity, emitter;
      std::set<std::string> sensors;
    };
    std::map<int, Life> lives;
    for (const TrackReport& r : tracker.reports()) {
      if (r.status != TrackStatus::Confirmed) continue;
      auto [it, inserted] = lives.try_emplace(r.track_id);
      Life& life = it->second;
      if (inserted) life.first = r.t_s;
      life.last = r.t_s;
      life.cls = r.classification;
      life.identity = r.identity;
      life.emitter = r.emitter;
      std::string_view rest = r.sensors;
      while (!rest.empty()) {
        const auto plus = rest.find('+');
        life.sensors.insert(std::string(rest.substr(0, plus)));
        rest = plus == std::string_view::npos ? std::string_view{} : rest.substr(plus + 1);
      }
    }
    std::printf("%-6s %8s %8s  %-9s %-28s %-8s %s\n", "track", "from_s", "to_s", "class", "identity", "emitter",
                "sensors");
    for (const auto& [id, life] : lives) {
      std::string sensors;
      for (const auto& name : life.sensors) sensors += (sensors.empty() ? "" : " ") + name;
      std::printf("%-6d %8.1f %8.1f  %-9s %-28s %-8s %s\n", id, life.first, life.last, life.cls.c_str(),
                  life.identity.empty() ? "-" : life.identity.c_str(), life.emitter.empty() ? "-" : life.emitter.c_str(),
                  sensors.c_str());
    }
    std::printf("\nwrote %s\n", out_dir.string().c_str());
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
  return 0;
}

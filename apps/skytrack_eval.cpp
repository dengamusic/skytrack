// Scores a tracker run against the simulator's ground truth.

#include <algorithm>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <string>
#include <string_view>

#include "skytrack/eval/metrics.hpp"
#include "skytrack/scenario.hpp"
#include "skytrack/tracking/track_io.hpp"

namespace {

int usage() {
  std::fprintf(stderr,
               "usage: skytrack_eval <run-dir> [--tracks DIR] [--c METRES]\n"
               "  reads  <run-dir>/truth/ and the tracker output (default <run-dir>/tracker/)\n"
               "  writes <tracks>/eval/: metrics.json, gospa.csv, matches.csv\n");
  return 2;
}

}  // namespace

int main(int argc, char** argv) {
  namespace fs = std::filesystem;
  using namespace skytrack;
  if (argc < 2) return usage();
  const fs::path run_dir = argv[1];
  fs::path tracks_dir = run_dir / "tracker";
  eval::EvalConfig config;
  for (int i = 2; i < argc; ++i) {
    const std::string_view arg = argv[i];
    if (arg == "--tracks" && i + 1 < argc) {
      tracks_dir = argv[++i];
    } else if (arg == "--c" && i + 1 < argc) {
      config.gospa_c_m = std::stod(argv[++i]);
    } else {
      return usage();
    }
  }

  try {
    const auto truth = eval::read_truth_targets(run_dir / "truth" / "targets.csv");
    const auto origins = eval::read_measurement_origins(run_dir / "truth" / "measurement_origins.csv");
    // A fusion centre's output has no per-measurement associations to score.
    const bool has_associations = std::filesystem::exists(tracks_dir / "associations.csv");
    const auto associations = has_associations ? eval::read_associations(tracks_dir / "associations.csv")
                                               : decltype(eval::read_associations(tracks_dir)){};
    const auto reports = read_track_reports(tracks_dir / "tracks.csv");
    const Json summary = read_json_file(tracks_dir / "summary.json");
    if (!summary.contains("first_output_s")) throw std::runtime_error("the tracker published nothing");

    // Score only while the scene runs: afterwards there is no truth to compare with.
    const double scene_end_s = read_json_file(run_dir / "scenario.json").at("duration_s").get<double>();
    const double last_s = std::min(summary.at("last_output_s").get<double>(), scene_end_s);
    const eval::Evaluation ev =
        eval::evaluate(truth, reports, summary.at("first_output_s").get<double>(), last_s,
                       summary.at("output_rate_hz").get<double>(), config, &origins,
                       has_associations ? &associations : nullptr);
    eval::write_evaluation(ev, tracks_dir / "eval");

    std::printf("%-11s %-8s %7s %8s %7s %9s %6s %4s %6s %6s %6s %5s\n", "target", "class", "tracked", "to_track",
                "tracks", "switches", "gaps", "", "rmse_h", "rmse_v", "nees", "class");
    for (const auto& s : ev.targets) {
      char to_track[16] = "-";
      if (s.first_detection_s && s.first_track_s) {
        std::snprintf(to_track, sizeof to_track, "%.1fs", *s.first_track_s - *s.first_detection_s);
      }
      std::printf("%-11s %-8s %6.0f%% %8s %7zu %9d %6d %4s %5.1fm %5.1fm %6.1f %4.0f%%\n", s.name.c_str(), s.cls.c_str(),
                  100.0 * s.tracked_fraction, to_track, s.track_ids.size(), s.id_switches, s.gaps, "",
                  s.rmse_horizontal_m, s.rmse_vertical_m, s.nees_mean, 100.0 * s.class_correct);
    }
    const Json j = ev.to_json();
    std::printf("\nGOSPA (c = %.0f m): mean %.1f m, localisation %.1f m, %.2f missed and %.2f false tracks on average\n",
                config.gospa_c_m, ev.mean_gospa(), j["gospa"]["mean_localisation_error_m"].get<double>(),
                j["gospa"]["mean_missed_targets"].get<double>(), j["gospa"]["mean_false_tracks"].get<double>());
    std::printf("confirmed tracks: %zu, false: %d (%.1f per hour)\n", ev.tracks.size(), ev.false_track_count(),
                ev.false_tracks_per_hour());
    if (ev.association) {
      const auto& a = *ev.association;
      std::printf("detections used: %.1f%% of target detections, %.2f%% of false alarms; %lld went to the wrong target\n",
                  a.target_detections > 0 ? 100.0 * a.target_detections_used / a.target_detections : 0.0,
                  a.false_alarms > 0 ? 100.0 * a.false_alarms_used / a.false_alarms : 0.0,
                  static_cast<long long>(a.wrong_target_updates));
    }
    std::printf("\nwrote %s\n", (tracks_dir / "eval").string().c_str());
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
  return 0;
}

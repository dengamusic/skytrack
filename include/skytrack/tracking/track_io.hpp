#pragma once

#include <filesystem>
#include <memory>
#include <vector>

#include "skytrack/json.hpp"
#include "skytrack/measurements.hpp"
#include "skytrack/tracking/tracker.hpp"

namespace skytrack {

/// Reads measurements.jsonl: one message per line, in arrival order.
struct MeasurementLog {
  std::vector<SensorMessage> messages;
  std::size_t bad_lines = 0;              // lines that were not a valid message: skipped
  std::vector<std::string> first_errors;  // "file:line: what", for the first few
};
/// Reads measurements.jsonl. A corrupt line (a truncated write, a sensor
/// bug) is skipped and counted, as a live system would, not fatal.
MeasurementLog read_measurements(const std::filesystem::path& path);

/// Runs the tracker over a whole run, as if the messages arrived live.
Tracker run_tracker(std::shared_ptr<const SensorSetup> setup, const TrackerConfig& config,
                    const std::vector<SensorMessage>& messages);

/// Writes reports as tracks.csv: one line per track per output time, with
/// the full state covariance. It is also what a node sends a fusion centre.
void write_track_reports(const std::vector<TrackReport>& reports, const std::filesystem::path& path);
/// Writes tracks.csv, associations.csv and summary.json into dir.
void write_tracker_output(const Tracker& tracker, const std::filesystem::path& dir);
/// Counters, configuration, registration estimates, sensor health and report
/// latency. Deterministic for a given input.
Json tracker_summary_json(const Tracker& tracker);
/// Age of confirmed reports when published (host clock minus report time).
Json report_latency_json(const Tracker& tracker);
/// Wall-clock time per ingested message; varies from run to run.
Json processing_time_json(const Tracker& tracker);

/// Reads tracks.csv back, full covariance included (to 10 significant digits).
std::vector<TrackReport> read_track_reports(const std::filesystem::path& path);

}  // namespace skytrack

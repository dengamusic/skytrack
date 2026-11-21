#pragma once

#include <cstdint>
#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "skytrack/json.hpp"
#include "skytrack/scenario.hpp"
#include "skytrack/tracking/tracker.hpp"

/// Scoring a tracker run against the simulator's ground truth.
namespace skytrack::eval {

/// One target's true trajectory, as sampled in truth/targets.csv.
struct TruthTarget {
  int id = 0;
  std::string name;
  std::string cls;
  std::vector<double> t_s;
  std::vector<Vec6> states;

  /// Linear interpolation between samples; empty outside the target's lifetime.
  std::optional<Vec6> at(double t) const;
};

std::vector<TruthTarget> read_truth_targets(const std::filesystem::path& targets_csv);
/// The same truth, sampled straight from a scenario's trajectories.
std::vector<TruthTarget> truth_from_scenario(const Scenario& scenario);
struct Origin {
  int target_id = -1;  // -1 = false alarm
  double t_true_s = 0.0;
};
/// meas_id -> where the detection came from, from truth/measurement_origins.csv.
std::map<std::int64_t, Origin> read_measurement_origins(const std::filesystem::path& origins_csv);
/// meas_id -> (track id, outcome), from the tracker's associations.csv.
std::map<std::int64_t, std::pair<int, std::string>> read_associations(const std::filesystem::path& associations_csv);

/// Generalised optimal sub-pattern assignment metric (Rahmathullah et al.,
/// 2017) with alpha = 2: localisation error of matched pairs, plus c^p / 2
/// for every missed target and every false track. Pairs further apart than
/// c count as one miss and one false track.
struct Gospa {
  double total = 0.0;              // metres
  double localisation_sum = 0.0;   // sum of d^p over matched pairs
  int matched = 0;
  int missed = 0;
  int false_tracks = 0;
  std::vector<std::pair<int, int>> pairs;  // (truth index, estimate index), matched only
};
Gospa gospa(const std::vector<Vec3>& truth, const std::vector<Vec3>& estimates, double c, double p);

struct EvalConfig {
  double gospa_c_m = 100.0;
  double gospa_p = 2.0;
};

struct TargetScore {
  int id = 0;
  std::string name;
  std::string cls;
  double present_s = 0.0;
  double tracked_fraction = 0.0;
  std::optional<double> first_detection_s;  // first time any sensor reported it
  std::optional<double> first_track_s;      // first output time a confirmed track was on it
  std::optional<double> first_classified_s; // first output time its track had the right class
  std::vector<int> track_ids;               // in the order they followed it
  int id_switches = 0;                      // the track following it changed
  int gaps = 0;                             // it lost its track and got one back later
  double rmse_horizontal_m = 0.0;
  double rmse_vertical_m = 0.0;
  double rmse_velocity_mps = 0.0;
  double nees_mean = 0.0;          // position NEES; 3 if the covariance is honest
  double nees_within_95 = 0.0;     // fraction below the chi-square(3) 95% bound
  double class_correct = 0.0;      // fraction of tracked time with the right class
};

struct TrackScore {
  int id = 0;
  double confirmed_s = 0.0;      // time reported as confirmed
  double on_target_s = 0.0;      // ... of which matched to a real target
  std::map<int, int> matched_ticks;  // target id -> output times matched to it
  bool is_false() const { return on_target_s < 0.5 * confirmed_s; }
};

struct AssociationScore {
  std::int64_t target_detections = 0;
  std::int64_t target_detections_used = 0;  // updated or started a confirmed track
  std::int64_t false_alarms = 0;
  std::int64_t false_alarms_used = 0;       // updated or started a confirmed track
  std::int64_t wrong_target_updates = 0;    // used by a track whose main target is another one
};

/// One confirmed track matched to one target at one output time.
struct Match {
  double t_s = 0.0;
  int target_id = 0;
  int track_id = 0;
  double error_horizontal_m = 0.0;
  double error_vertical_m = 0.0;
  double nees = 0.0;
};

struct Evaluation {
  EvalConfig config;
  std::vector<double> times;
  std::vector<Gospa> gospa_series;
  std::vector<TargetScore> targets;
  std::vector<TrackScore> tracks;
  std::vector<Match> matches;
  std::optional<AssociationScore> association;
  double duration_s = 0.0;

  double mean_gospa() const;
  int false_track_count() const;
  double false_tracks_per_hour() const;
  Json to_json() const;
};

/// Scores confirmed tracks at every output time in [first_s, last_s].
/// Origins and associations are optional; without them there is no association score.
Evaluation evaluate(const std::vector<TruthTarget>& truth, const std::vector<TrackReport>& reports, double first_s,
                    double last_s, double rate_hz, const EvalConfig& config,
                    const std::map<std::int64_t, Origin>* origins = nullptr,
                    const std::map<std::int64_t, std::pair<int, std::string>>* associations = nullptr);

/// Writes metrics.json, gospa.csv and matches.csv into dir.
void write_evaluation(const Evaluation& evaluation, const std::filesystem::path& dir);

}  // namespace skytrack::eval

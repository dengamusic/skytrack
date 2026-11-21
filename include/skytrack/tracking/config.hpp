#pragma once

#include <map>
#include <optional>
#include <string>

#include "skytrack/json.hpp"
#include "skytrack/tracking/motion.hpp"

namespace skytrack {

/// What the operator knows about one sensor beyond sensors.json.
struct SensorOverrides {
  double clock_offset_s = 0.0;      // known clock error: tracker time = sensor stamp - offset
  double assumed_latency_s = 0.0;   // sensors without timestamps: tracker time = arrival - latency
  std::optional<double> position_std_h_m;  // cooperative reports: overrides the global value
  std::optional<double> position_std_v_m;
};

struct TrackerConfig {
  // Time handling.
  double reorder_window_s = 0.6;       // hold messages this long so they can be processed in time order
  double max_measurement_age_s = 10.0; // older than the track it would update: dropped
  double late_initiation_s = 3.0;      // data this far behind what was processed may still start tracks
  double output_rate_hz = 2.0;

  // Filter.
  ProcessNoise process_noise;
  double init_velocity_std_mps = 25.0;  // unknown velocity of a new radar track (per axis)
  /// GNSS error of broadcast positions (ADS-B, Remote ID) when a report
  /// does not state its own accuracy.
  double cooperative_std_h_m = 10.0;
  double cooperative_std_v_m = 15.0;
  double cooperative_white_std_m = 1.0;  // jitter from one report to the next, on top of it
  /// The GNSS error drifts slowly, so consecutive reports of one target share
  /// most of it: a first-order Gauss-Markov process with this time constant.
  /// The tracker carries each track's correlation with it (a Schmidt or
  /// "consider" filter) instead of averaging it away. Online registration
  /// uses the same time for its references.
  double broadcast_error_correlation_s = 30.0;

  // Association.
  /// A broadcast carrying the track's own identity may be this many times
  /// further off (in normalised distance) and still update it.
  double same_identity_gate_scale = 4.0;

  // Track management.
  int confirm_hits = 2;   // M of N: confirmed after M radar hits ...
  int confirm_window = 3; // ... within the last N radar scans that should have seen it
  int cooperative_confirm_hits = 2;
  double tentative_coast_s = 3.0;   // deleted after this long without an update
  double confirmed_coast_s = 8.0;
  double max_position_std_m = 500.0;
  bool merge_duplicates = true;
  double merge_gate_chi2 = 16.81;  // chi-square, 6 degrees of freedom, 0.99: two confirmed tracks are one
  /// Line-of-sight ghosts: a confirmed track without identity and without a
  /// radar update for ghost_stale_radar_s, within ghost_direction_deg of a
  /// track the radar updated within ghost_live_radar_s, is a copy of it that
  /// angle-only sensors keep alive at the wrong range.
  double ghost_direction_deg = 1.5;
  double ghost_live_radar_s = 3.0;
  double ghost_stale_radar_s = 6.0;

  // Radar specifics.
  double radar_sector_s = 0.1;              // a rotating scan is processed in sectors this long
  double radar_assumed_rcs_dbsm = -20.0;    // when deciding whether the radar should have seen a track
  double radar_min_expected_pd = 0.5;       // M of N counts only scans at least this likely to see the track
  double radar_max_assumed_pd = 0.95;       // the Pd model is approximate: no single miss is impossible
  /// When a radar keeps missing a confirmed track that other sensors still
  /// see (within notch_evidence_s), and misses that many would happen with
  /// probability below this if the target were moving, assume it hovers in
  /// the Doppler notch (range rate ~ 0). 0 turns this off.
  double notch_inference_probability = 1e-3;
  double notch_evidence_s = 2.0;
  /// ... unless the track's own range rate rules hovering out (chi-square,
  /// 1 degree of freedom, 0.99). Runs of misses that unlikely still happen
  /// by chance over hundreds of scans; a wrong "hovering" pushes the track
  /// off along the line of sight, so the kinematics must agree too.
  double notch_gate_chi2 = 6.63;
  /// The radar is not blind along a direction where it detected another track
  /// within this angle in the same scan: misses there are not the notch.
  double radar_conflict_deg = 3.0;
  /// When no other sensor sees a track and the radar's misses would happen
  /// with probability below this, delete it: nothing is there.
  double refute_probability = 1e-4;

  // Classification: votes per piece of evidence (camera labels vote their confidence).
  double class_vote_rf_signature = 0.5;  // a recognised drone control link
  double class_vote_broadcast = 5.0;     // ADS-B means aircraft, Remote ID means drone

  // Sensor registration.
  bool registration = true;         // learn mount and clock errors from cooperative targets
  double mount_accuracy_deg = 1.0;  // survey accuracy of heading and pitch (1 sigma)
  double clock_accuracy_s = 0.25;   // timestamp (or assumed latency) accuracy (1 sigma)
  /// Random walk of the errors, so the estimate can follow a mount that
  /// settles or a clock that drifts instead of freezing.
  double mount_drift_deg_per_sqrt_h = 0.1;
  double clock_drift_s_per_sqrt_h = 0.01;
  double registration_reference_max_age_s = 2.0;  // a broadcast this old still serves as reference
  /// What is still unknown about a sensor's mount and clock is the same for
  /// all its detections. true: carry it as a consider parameter, so a fast
  /// camera cannot average it away (honest covariance). false: treat it as
  /// fresh noise in every detection (the classic approach: simpler, and
  /// overconfident once registration has converged).
  bool consider_registration = true;

  std::map<std::string, SensorOverrides> sensors;
};

/// Strict: unknown fields are errors. Missing fields keep their defaults.
TrackerConfig parse_tracker_config(const Json& j);
Json to_json(const TrackerConfig& config);

}  // namespace skytrack

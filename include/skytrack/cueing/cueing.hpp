#pragma once

#include <map>
#include <optional>
#include <string>
#include <vector>

#include "skytrack/scenario.hpp"
#include "skytrack/sensor.hpp"
#include "skytrack/simulator.hpp"
#include "skytrack/tracking/config.hpp"
#include "skytrack/tracking/tracker.hpp"

namespace skytrack {

/// Closed-loop cueing: the tracker's published tracks decide where a
/// narrow-field gimbal camera looks, and what it then sees goes back into the
/// tracker. The usual job is to put a high-resolution camera on each
/// unidentified track long enough for its detector to say what it is.
struct CueConfig {
  std::string camera_id = "cam-ptz";
  double max_slew_deg_s = 60.0;  // gimbal rate limit, pan and tilt each
  /// Once the camera is on a track it stays at least min_dwell_s, and at
  /// most max_dwell_s if the track is still unclassified by then.
  double min_dwell_s = 1.5;
  double max_dwell_s = 6.0;
  /// Give up on a track the camera has not reached in this long.
  double max_slew_time_s = 4.0;
  /// A track already classified is worth another look after this long.
  double revisit_s = 30.0;
  /// Pointing error that counts as on the track (well inside the narrow field).
  double on_target_deg = 1.0;
};

/// Priority policy, over tracks with no identity (cooperative targets say
/// who they are), in order:
///   1. no class yet and never looked at, the most urgent first;
///   2. no class yet, last looked at over revisit_s / 3 ago, most urgent first;
///   3. classified as something other than a drone, not looked at for
///      revisit_s (a check on the classification), the longest unseen first;
///   4. custody: with nothing else to do, stay on the most urgent drone,
///      which gives the tracker precise bearings on the threat.
/// Urgency is the time the target would need to reach the camera at its
/// current closing speed (at least 1 m/s), plus the time to slew to it.
/// The choice is revisited every min_dwell_s, so new tracks pre-empt custody.
class CueController {
 public:
  /// `mount` is the camera's nominal mount: the controller knows no more
  /// than the tracker does.
  CueController(CueConfig config, const Mount& mount, GimbalReport start);

  /// Where the gimbal points for a frame at time t, after moving (rate
  /// limited) towards the track chosen from `picture`, the tracker's latest
  /// published reports.
  GimbalReport step(double t, const std::vector<TrackReport>& picture);

  std::optional<int> target() const { return target_; }

  struct Stats {
    std::size_t frames = 0;
    std::size_t frames_on_target = 0;  // pointing within on_target_deg of the chosen track
    std::size_t looks = 0;             // tracks the camera was put on
    std::size_t gave_up = 0;           // tracks it could not reach in max_slew_time_s
  };
  const Stats& stats() const { return stats_; }

  /// Pan and tilt (nominal mount) that put a world point in the centre of the image.
  GimbalReport pointing_to(const Vec3& p_enu) const;

 private:
  std::optional<int> choose(double t, const std::vector<TrackReport>& picture) const;

  CueConfig cfg_;
  Mount mount_;
  GimbalReport pointing_;
  double last_t_ = 0.0;
  bool started_ = false;
  std::optional<int> target_;
  double chosen_at_ = 0.0;
  std::optional<double> on_target_since_;
  std::map<int, double> last_look_;  // track id -> when the camera last left it
  Stats stats_;
};

/// A run of the scenario with the camera pointed in closed loop.
struct CuedRun {
  SimulationResult sim;  // every message, the camera's included, in arrival order
  Tracker tracker;       // the tracker that ran live and pointed the camera
  CueController::Stats cue;
};

/// Simulates every other sensor as usual; then, frame by frame, feeds the
/// tracker everything that has arrived, asks the controller where to point,
/// and renders the camera frame there. The tracker's picture lags the truth
/// by the sensors' latency and its reorder window, as it would live.
CuedRun run_cued(const Scenario& scenario, const TrackerConfig& tracker_config, const CueConfig& cue_config);

}  // namespace skytrack

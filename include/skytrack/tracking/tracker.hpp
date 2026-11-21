#pragma once

#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "skytrack/tracking/config.hpp"
#include "skytrack/tracking/registration.hpp"
#include "skytrack/tracking/sensor_model.hpp"

namespace skytrack {

enum class TrackStatus { Tentative, Confirmed };
std::string_view to_string(TrackStatus status);

/// Errors that many measurements share, which the tracker accounts for but
/// does not estimate (Schmidt "consider" parameters): a target's own GNSS
/// error, which all its broadcasts carry, and each sensor's remaining
/// registration error, which all its detections carry. Treating them as
/// fresh noise in every measurement would average them away and make tracks
/// overconfident. A track keeps Cov(its error, parameter) for each, keyed by
/// kGnssError or by sensor id.
using ConsiderCross = std::map<std::string, Eigen::Matrix<double, 6, 3>, std::less<>>;
inline constexpr std::string_view kGnssError = "#gnss";

struct Track {
  int id = 0;
  TrackStatus status = TrackStatus::Tentative;
  Gaussian state;
  double t_s = 0.0;  // time of `state`
  double created_s = 0.0;
  double last_update_s = 0.0;
  std::optional<double> confirmed_s;
  int updates = 0;
  int cooperative_updates = 0;

  /// Outcomes of the last few radar scans that should have seen it: true = hit.
  std::deque<bool> radar_history;
  std::map<std::string, std::uint64_t> last_radar_hit_seq;  // radar id -> scan
  /// log P(this many misses in a row | a target is really there and moving),
  /// per radar. Very negative: the radar should have seen it by now.
  std::map<std::string, double> radar_miss_log_prob;
  /// Running radar cross section estimate (linear samples), all radars together.
  double rcs_sum_m2 = 0.0;
  int rcs_samples = 0;
  /// Estimated radar cross section in dBsm, or `fallback` until a few samples are in.
  double rcs_dbsm(double fallback) const;
  std::map<std::string, double> last_update_by_sensor;      // sensor id -> measurement time
  double last_radar_update_s = -1e9;
  double last_notch_s = -1e9;  // last time radar silence was explained as hovering

  /// The target's latest broadcast of its own state (ADS-B, Remote ID): the
  /// reference that sensor registration compares other sensors against.
  std::optional<Gaussian> broadcast;
  double broadcast_t_s = 0.0;
  Eigen::Matrix3d broadcast_bias = Eigen::Matrix3d::Zero();  // covariance of its GNSS error (east, north, up)
  ConsiderCross consider;  // at time t_s, like `state`

  // Evidence about what it is.
  std::map<std::string, double> class_votes;
  std::map<std::string, int> emitter_counts;
  std::string identity;  // "adsb:3c6589", "remote_id:...", or empty

  /// "drone", "aircraft", "bird" or "unknown".
  std::string classification() const;
  /// Most often heard RF emitter type, or empty.
  std::string emitter() const;
};

/// A track as published at one output time.
struct TrackReport {
  double t_s = 0.0;
  int track_id = 0;
  TrackStatus status = TrackStatus::Tentative;
  std::string classification;
  std::string identity;
  std::string emitter;
  Gaussian state;
  double last_update_s = 0.0;
  std::string sensors;  // sensors that updated it in the last few seconds, joined by '+'
  /// Host clock when the report was published. t_published_s - t_s is how
  /// stale the report is when a user (display, effector, cueing) gets it.
  double t_published_s = 0.0;
};

enum class AssociationOutcome { Updated, Initiated, Unassigned, TooOld };
std::string_view to_string(AssociationOutcome outcome);

/// What happened to one measurement.
struct AssociationRecord {
  std::int64_t meas_id = -1;
  std::string sensor_id;
  double t_s = 0.0;
  int track_id = -1;
  AssociationOutcome outcome = AssociationOutcome::Unassigned;
  double d2 = 0.0;  // normalised distance to the track it updated
};

struct TrackerStats {
  std::uint64_t messages = 0;
  std::uint64_t measurements = 0;
  std::uint64_t updates = 0;
  std::uint64_t late_updates = 0;  // measurements older than the track they updated
  std::uint64_t initiations = 0;
  std::uint64_t unassigned = 0;
  std::uint64_t too_old = 0;
  std::uint64_t notch_inferences = 0;   // radar silence taken as "hovering"
  std::uint64_t notch_rejections = 0;   // ... except the track was sure it was moving
  std::uint64_t inconsistent_innovations = 0;  // pairings skipped: S not positive definite
  std::uint64_t covariance_repairs = 0;        // updates whose P had to be clipped back to positive
  int tracks_refuted = 0;               // deleted because the radar should have seen them
  std::uint64_t registration_samples = 0;
  int tracks_confirmed = 0;
  int tracks_merged = 0;
  int ghosts_removed = 0;  // same line of sight as a radar-supported track, no radar of their own
  int tracks_deleted = 0;
  std::uint64_t rejected_messages = 0;      // from a sensor the site does not have
  std::uint64_t rejected_measurements = 0;  // non-finite values
};

/// What the tracker can tell about one sensor's data stream from the data
/// alone: how often it reports, what it loses, how late its data arrives.
struct SensorHealth {
  std::uint64_t messages = 0;
  std::uint64_t detections = 0;
  std::uint64_t rejected = 0;       // detections with non-finite values
  std::uint64_t lost_messages = 0;  // gaps in the message sequence counter
  std::uint64_t out_of_order = 0;   // sequence counter went backwards or repeated
  std::uint64_t late_messages = 0;  // arrived after newer data was already processed
  double latency_sum_s = 0.0;       // arrival minus the end of the data, on the tracker clock
  double latency_max_s = 0.0;
  double first_arrival_s = 0.0;
  double last_arrival_s = 0.0;
  std::optional<std::uint64_t> last_seq;

  double mean_latency_s() const { return messages ? latency_sum_s / static_cast<double>(messages) : 0.0; }
  double rate_hz() const {
    const double span = last_arrival_s - first_arrival_s;
    return messages > 1 && span > 0.0 ? static_cast<double>(messages - 1) / span : 0.0;
  }
};

/// Multi-sensor tracker: extended Kalman filter (nearly constant velocity,
/// ENU), global nearest neighbour association, M-of-N confirmation.
///
/// Feed it messages in arrival order. It holds them for reorder_window_s so
/// it can process measurements in the order they were taken; a measurement
/// that still arrives late is applied to the newer track state by predicting
/// the track back to the measurement time (an approximate out-of-sequence
/// update). Track states are published every 1 / output_rate_hz seconds.
class Tracker {
 public:
  /// The tracker shares ownership of the sensor setup: its measurements
  /// point at the setup's sensor models for as long as it lives.
  Tracker(std::shared_ptr<const SensorSetup> setup, TrackerConfig config);

  /// One message from the network. Throws if its sensor is not in the setup.
  void ingest(const SensorMessage& msg);
  /// Processes everything still held back and publishes up to the last measurement.
  void finish();

  const std::vector<Track>& tracks() const { return tracks_; }
  const std::vector<TrackReport>& reports() const { return reports_; }
  const std::vector<AssociationRecord>& associations() const { return associations_; }
  const TrackerStats& stats() const { return stats_; }
  const std::map<std::string, SensorHealth>& health() const { return health_; }
  /// Wall-clock time each ingest() call took, in microseconds (varies run to run).
  const std::vector<float>& ingest_time_us() const { return ingest_time_us_; }
  const TrackerConfig& config() const { return cfg_; }
  const Registration& registration() const { return registration_; }
  /// First and last time a report was published for (empty before any).
  std::optional<std::pair<double, double>> output_span() const;

 private:
  struct Event {
    enum class Kind { Batch, ScanEnd };
    Kind kind = Kind::Batch;
    const SensorModel* sensor = nullptr;
    std::uint64_t seq = 0;
    double t_s = 0.0;  // measurements in a batch are no later than this
    std::vector<Measurement> measurements;
  };

  struct Innovation {
    Gaussian prior;      // track predicted to t_ref
    double t_ref = 0.0;  // max(track time, measurement time)
    Eigen::VectorXd nu;
    Eigen::MatrixXd H;
    Eigen::MatrixXd R;
    Eigen::MatrixXd S;
    double d2 = 0.0;
    bool late = false;
    /// nu = -H e + G q + w: the measurement carries one consider parameter q
    /// (the target's GNSS error, or the sensor's registration error), or none.
    std::string consider;
    Eigen::MatrixXd G;
    Eigen::Matrix3d sigma = Eigen::Matrix3d::Zero();  // Cov(q)
    ConsiderCross cross;                              // Cov(prior error, each parameter)
  };

  void note_health(const SensorModel& sensor, const SensorMessage& msg, double data_end_s);
  void start_consider(Track& track, const Measurement& m) const;
  Mat6 difference_covariance(const Track& a, const Gaussian& ga, const Track& b, const Gaussian& gb, double t) const;
  /// Folds `drop` into `keep`: gk and gd are both predicted to time `at`.
  void merge_into(Track& keep, const Track& drop, const Gaussian& gk, const Gaussian& gd, double at) const;
  void enqueue(Event event);
  void release(double up_to);
  void process_batch(Event& event);
  void process_scan_end(const Event& event);
  std::optional<Innovation> innovate(const Track& track, const Measurement& m) const;
  void update(Track& track, const Measurement& m, const Innovation& innovation, std::uint64_t seq);
  void start_track(const Measurement& m, const Gaussian& g, std::uint64_t seq);
  void confirm(Track& track, double t);
  bool infer_notch(Track& track, const RadarModel& radar, double t);
  bool radar_sees_along(const RadarModel& radar, const Vec6& x, std::uint64_t seq, int except_track) const;
  void maintain(double t);
  void remove_line_of_sight_ghosts(double t);
  void publish_before(double t, bool inclusive);
  void record(const Measurement& m, int track_id, AssociationOutcome outcome, double d2 = 0.0);

  std::shared_ptr<const SensorSetup> setup_;
  TrackerConfig cfg_;
  double output_period_s_;

  std::map<std::pair<double, std::uint64_t>, Event> queue_;  // (time, arrival order)
  std::uint64_t next_order_ = 0;
  double latest_arrival_s_;
  double processed_until_s_;
  std::optional<double> next_output_index_;
  std::optional<double> first_output_s_;
  double last_output_s_ = 0.0;

  Registration registration_;
  std::vector<Track> tracks_;
  int next_track_id_ = 1;
  std::vector<TrackReport> reports_;
  std::vector<AssociationRecord> associations_;
  mutable TrackerStats stats_;  // counters; const helpers count what they skip
  std::map<std::string, SensorHealth> health_;
  std::vector<float> ingest_time_us_;
};

}  // namespace skytrack

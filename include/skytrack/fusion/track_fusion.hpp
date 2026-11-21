#pragma once

#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "skytrack/tracking/motion.hpp"
#include "skytrack/tracking/tracker.hpp"

namespace skytrack {

/// Track-to-track fusion: a fusion centre that takes the confirmed tracks
/// each sensor node publishes (its tracks.csv: state and full covariance
/// at every output time) and turns them into one picture.
///
/// The nodes' errors are correlated, and nobody knows by how much: they saw
/// the same target manoeuvre (common process noise) and may share inputs
/// (both take the same online flight feed). How the centre combines two
/// estimates of one target is therefore a choice, and the point of the
/// comparison this module exists for:
enum class Combine {
  /// Covariance intersection: consistent whatever the cross-correlation.
  CovarianceIntersection,
  /// Information sum, as if the nodes' errors were independent. Optimal if
  /// they were; with shared data it counts that data twice and is overconfident.
  Independent,
  /// No fusion: the estimate of whichever node is most certain.
  Best,
};
std::string_view to_string(Combine combine);
/// "ci", "independent" or "best".
std::optional<Combine> parse_combine(std::string_view name);

struct FusionConfig {
  Combine combine = Combine::CovarianceIntersection;
  /// Two node tracks this close (chi-square, 6 degrees of freedom, 0.99) may be one target.
  double gate_chi2 = 16.81;
  /// An associated node track this far from the rest of its global track
  /// (chi-square 6 dof, 0.9999) on `release_after` output times in a row is
  /// split off again: the association was wrong, or the targets parted.
  double release_chi2 = 27.86;
  int release_after = 3;
  /// A node track vaguer than this (horizontal position, 1 sigma, worst axis)
  /// cannot start a global track: a bearing-only track at an unknown range
  /// is not a target to report. It can still join a global track.
  double start_max_position_std_m = 50.0;
  /// A node track not reported for this long has ended at its node.
  double max_report_age_s = 1.0;
  /// To predict a node report to the fusion time when it is older.
  ProcessNoise process_noise;
};

/// One node's published track reports, in time order.
struct NodeTracks {
  std::string node;
  std::vector<TrackReport> reports;
};

struct FusionStats {
  std::size_t node_reports = 0;   // confirmed node reports taken in
  std::size_t global_tracks = 0;  // global track ids handed out
  std::size_t joins = 0;          // a node track associated with an existing global track
  std::size_t releases = 0;       // an association undone
  std::size_t merges = 0;         // two global tracks found to be one target
  std::size_t held_back = 0;      // node reports too vague to start a global track
  std::size_t withheld = 0;       // global track reports withheld as uncorroborated split-offs
};

/// Keeps a persistent map from (node, local track id) to a global track id,
/// so a global track keeps its id while any node still follows the target,
/// even across a break in one node's track.
///
/// At each output time: node tracks already associated stay so unless they
/// drift apart; new node tracks are assigned to global tracks that have no
/// track from that node yet (global nearest neighbour, chi-square gate,
/// matching identities always associate, conflicting ones never); the rest
/// start global tracks. A node track split off because it drifted from the
/// track it was associated with is presumed corrupted (the longer-held one
/// wins): it is not published on its own until another node corroborates it. Association always gates on the covariance
/// intersection estimate, so the three Combine rules see the same
/// associations and differ only in how they fuse.
class FusionCentre {
 public:
  explicit FusionCentre(FusionConfig config);

  /// Everything the nodes reported for output time t, as (node, report).
  /// Tentative reports are ignored. Calls must come in time order.
  void fuse(double t, const std::vector<std::pair<std::string, TrackReport>>& reports);

  /// One report per global track per output time, as a node would publish them.
  const std::vector<TrackReport>& reports() const { return reports_; }
  const FusionStats& stats() const { return stats_; }
  const FusionConfig& config() const { return cfg_; }

 private:
  struct Member {
    std::string node;
    int local_id = 0;
    TrackReport last;      // its latest report
    int disagreements = 0;  // output times in a row beyond release_chi2
  };
  struct Global {
    int id = 0;
    std::vector<Member> members;  // in the order they joined; at most one per node
  };

  Gaussian member_estimate(const Member& m, double t) const;
  /// The global track's estimate at t, by the given rule, from the node
  /// tracks that currently agree with it.
  Gaussian estimate(const Global& g, double t, Combine combine) const;
  bool has_node(const Global& g, const std::string& node) const;

  FusionConfig cfg_;
  std::vector<Global> globals_;
  /// Split-off node tracks not corroborated since: (node, local id) -> last seen.
  std::map<std::pair<std::string, int>, double> suspects_;
  int next_id_ = 1;
  std::vector<TrackReport> reports_;
  FusionStats stats_;
};

/// Runs a fusion centre over whole node runs, one output time at a time.
FusionCentre fuse_nodes(const std::vector<NodeTracks>& nodes, const FusionConfig& config);

}  // namespace skytrack

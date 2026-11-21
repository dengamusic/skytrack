#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "skytrack/eval/metrics.hpp"
#include "skytrack/fusion/track_fusion.hpp"
#include "skytrack/rng.hpp"
#include "skytrack/run_writer.hpp"
#include "skytrack/scenario.hpp"
#include "skytrack/simulator.hpp"
#include "skytrack/tracking/covariance_intersection.hpp"
#include "skytrack/tracking/track_io.hpp"

namespace skytrack {
namespace {

/// A random covariance with eigenvalues spread over a few orders of magnitude.
Eigen::MatrixXd random_covariance(int n, Rng& rng) {
  Eigen::MatrixXd A(n, n);
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j < n; ++j) A(i, j) = rng.normal();
  }
  return A * A.transpose() + 0.01 * Eigen::MatrixXd::Identity(n, n);
}

bool positive_semidefinite(const Mat6& M) {
  return Eigen::SelfAdjointEigenSolver<Mat6>(0.5 * (M + M.transpose())).eigenvalues().minCoeff() > -1e-9;
}

TEST(CovarianceIntersection, OfAnEstimateWithItselfIsThatEstimate) {
  Rng rng(1);
  Gaussian a;
  a.P = random_covariance(6, rng);
  for (int k = 0; k < 6; ++k) a.x[k] = rng.normal();
  const Intersection ci = covariance_intersection(a, a);
  EXPECT_LT((ci.fused.x - a.x).norm(), 1e-9);
  EXPECT_LT((ci.fused.P - a.P).norm(), 1e-9 * a.P.norm());
}

/// Whatever the two errors' cross-covariance, the fused covariance must
/// cover the fused estimate's actual error covariance. Plain information
/// fusion (assuming independence) does not when the errors are correlated.
TEST(CovarianceIntersection, IsConsistentForAnyCrossCorrelation) {
  Rng rng(7);
  int naive_overconfident = 0;
  for (int trial = 0; trial < 200; ++trial) {
    // A random joint covariance of (error a, error b).
    const Eigen::MatrixXd joint = random_covariance(12, rng);
    Gaussian a;
    Gaussian b;
    a.P = joint.topLeftCorner<6, 6>();
    b.P = joint.bottomRightCorner<6, 6>();
    const Mat6 Pab = joint.topRightCorner<6, 6>();

    const Intersection ci = covariance_intersection(a, b);
    EXPECT_GE(ci.weight, 0.0);
    EXPECT_LE(ci.weight, 1.0);
    const Mat6 actual = ci.gain_a * a.P * ci.gain_a.transpose() + ci.gain_b * b.P * ci.gain_b.transpose() +
                        ci.gain_a * Pab * ci.gain_b.transpose() + ci.gain_b * Pab.transpose() * ci.gain_a.transpose();
    EXPECT_TRUE(positive_semidefinite(ci.fused.P - actual)) << "trial " << trial;

    // Information fusion: P = (Pa^-1 + Pb^-1)^-1, gains P Pa^-1 and P Pb^-1.
    const Mat6 P = (a.P.inverse() + b.P.inverse()).inverse();
    const Mat6 Ga = P * a.P.inverse();
    const Mat6 Gb = P * b.P.inverse();
    const Mat6 naive_actual =
        Ga * a.P * Ga.transpose() + Gb * b.P * Gb.transpose() + Ga * Pab * Gb.transpose() + Gb * Pab.transpose() * Ga.transpose();
    if (!positive_semidefinite(P - naive_actual)) ++naive_overconfident;
  }
  EXPECT_GT(naive_overconfident, 100);  // most random correlations break it
}

/// With one estimate much better in every direction, CI keeps (almost) only it.
TEST(CovarianceIntersection, PrefersTheBetterEstimate) {
  Gaussian a;
  Gaussian b;
  a.P = Mat6::Identity();
  b.P = 100.0 * Mat6::Identity();
  a.x.setConstant(1.0);
  b.x.setConstant(5.0);
  const Intersection ci = covariance_intersection(a, b);
  EXPECT_GT(ci.weight, 0.99);
  EXPECT_LT((ci.fused.x - a.x).norm(), 0.05);
}

TrackReport node_report(double t, int id, const Vec6& x, double sigma_m, const std::string& identity = "") {
  TrackReport r;
  r.t_s = t;
  r.track_id = id;
  r.status = TrackStatus::Confirmed;
  r.classification = "drone";
  r.identity = identity;
  r.state.x = x;
  r.state.P = sigma_m * sigma_m * Mat6::Identity();
  r.last_update_s = t;
  r.t_published_s = t;
  return r;
}

Vec6 at(double east_m, double north_m = 0.0) {
  Vec6 x = Vec6::Zero();
  x << east_m, north_m, 100.0, 0.0, 0.0, 0.0;
  return x;
}

std::set<int> global_ids(const FusionCentre& centre) {
  std::set<int> ids;
  for (const TrackReport& r : centre.reports()) ids.insert(r.track_id);
  return ids;
}

/// Two nodes whose errors share a common part (they saw the same manoeuvre,
/// took the same flight feed), with correlation 0.7. Covariance
/// intersection stays honest; the information sum, which assumes the errors
/// independent, claims half the variance it has.
TEST(FusionCentre, CovarianceIntersectionStaysHonestWhenNodesShareErrors) {
  constexpr double kSigma = 10.0;
  constexpr double kRho = 0.7;
  const auto mean_nees = [&](Combine combine) {
    Rng rng(7);
    FusionConfig config;
    config.combine = combine;
    FusionCentre centre(config);
    double sum = 0.0;
    constexpr int kSteps = 2000;
    for (int k = 0; k < kSteps; ++k) {
      const double t = 0.5 * k;
      Vec6 a;
      Vec6 b;
      for (int i = 0; i < 6; ++i) {
        const double shared = rng.normal(0.0, kSigma * std::sqrt(kRho));
        a[i] = shared + rng.normal(0.0, kSigma * std::sqrt(1.0 - kRho));
        b[i] = shared + rng.normal(0.0, kSigma * std::sqrt(1.0 - kRho));
      }
      centre.fuse(t, {{"a", node_report(t, 1, a, kSigma)}, {"b", node_report(t, 1, b, kSigma)}});
      const Gaussian& g = centre.reports().back().state;  // the truth is 0
      sum += g.x.dot(g.P.ldlt().solve(g.x));
    }
    EXPECT_EQ(global_ids(centre).size(), 1u);
    return sum / kSteps;
  };
  // 6 degrees of freedom: an honest covariance gives 6.
  EXPECT_LT(mean_nees(Combine::CovarianceIntersection), 6.0);   // (1 + rho) / 2 * 6 = 5.1
  EXPECT_GT(mean_nees(Combine::Independent), 9.0);              // (1 + rho) * 6 = 10.2
}

/// Node a's track breaks (a new local id); node b holds the target, so the
/// global track keeps its id.
TEST(FusionCentre, KeepsTheGlobalIdAcrossABreakAtOneNode) {
  FusionCentre centre{FusionConfig{}};
  for (int k = 0; k < 40; ++k) {
    const double t = 0.5 * k;
    std::vector<std::pair<std::string, TrackReport>> reports{{"b", node_report(t, 7, at(1.0), 10.0)}};
    if (k != 20) reports.emplace_back("a", node_report(t, k < 20 ? 1 : 2, at(-1.0), 10.0));
    centre.fuse(t, reports);
  }
  EXPECT_EQ(global_ids(centre), std::set<int>{1});
  EXPECT_EQ(centre.stats().joins, 2u);
}

TEST(FusionCentre, IdentitiesDecideAssociation) {
  FusionCentre centre{FusionConfig{}};
  for (int k = 0; k < 10; ++k) {
    const double t = 0.5 * k;
    centre.fuse(t, {// two aircraft side by side: same place, different transponders
                    {"a", node_report(t, 1, at(0.0), 10.0, "adsb:aaa111")},
                    {"b", node_report(t, 1, at(2.0), 10.0, "adsb:bbb222")},
                    // one aircraft that the nodes place 500 m apart: same transponder
                    {"a", node_report(t, 2, at(5000.0), 10.0, "adsb:ccc333")},
                    {"b", node_report(t, 2, at(5500.0), 10.0, "adsb:ccc333")}});
  }
  EXPECT_EQ(centre.stats().global_tracks, 3u);
}

/// Node b's track drifts off the target both nodes followed. It is split off
/// and withheld (the longer-held track wins) until a third node backs it.
TEST(FusionCentre, WithholdsASplitOffTrackUntilAnotherNodeBacksIt) {
  FusionCentre centre{FusionConfig{}};
  const auto published_at = [&](double t) {
    return std::count_if(centre.reports().begin(), centre.reports().end(),
                         [&](const TrackReport& r) { return r.t_s == t; });
  };
  for (int k = 0; k < 60; ++k) {
    const double t = 0.5 * k;
    const double b_east = k < 10 ? 0.0 : std::min(300.0, 30.0 * (k - 10));
    std::vector<std::pair<std::string, TrackReport>> reports{{"a", node_report(t, 1, at(0.0), 10.0)},
                                                             {"b", node_report(t, 1, at(b_east), 10.0)}};
    if (k >= 40) reports.emplace_back("c", node_report(t, 1, at(b_east + 5.0), 10.0));
    centre.fuse(t, reports);
    if (k == 5) {
      EXPECT_EQ(published_at(t), 1);
    }
    if (k == 30) {
      EXPECT_EQ(published_at(t), 1);  // b's split-off is withheld
    }
  }
  EXPECT_EQ(centre.stats().releases, 1u);
  EXPECT_GT(centre.stats().withheld, 0u);
  EXPECT_EQ(published_at(29.5), 2);  // c backs it: now two targets
  // While b drifted, a's estimate was not dragged along.
  for (const TrackReport& r : centre.reports()) {
    if (r.track_id == 1) {
      EXPECT_LT(std::abs(r.state.x[0]), 60.0) << "at t = " << r.t_s;
    }
  }
}

/// The two-site scene: each node tracks on its own sensors, the centre
/// fuses their tracks. The fused picture is at least as good as the better
/// node's, far better than the weaker one's, and covariance intersection
/// stays more honest than the information sum, which double counts what the
/// nodes share.
TEST(TrackFusion, TwoSitesFuseIntoOneHonestPicture) {
  const std::filesystem::path root = SKYTRACK_SOURCE_DIR;
  const Scenario scenario = load_scenario(root / "scenarios" / "two_sites.json");
  const SimulationResult sim = simulate(scenario);
  const TrackerConfig config;
  const auto setup = std::make_shared<const SensorSetup>(nominal_sensors_json(scenario), config);
  const auto truth = eval::truth_from_scenario(scenario);

  std::vector<NodeTracks> nodes;
  std::optional<std::pair<double, double>> span;
  const Json node_config = read_json_file(root / "configs" / "nodes" / "two_sites.json").at("nodes");
  for (auto it = node_config.begin(); it != node_config.end(); ++it) {
    const auto sensors = it.value().get<std::set<std::string>>();
    std::vector<SensorMessage> messages;
    for (const SensorMessage& m : sim.messages) {
      if (sensors.contains(m.sensor_id)) messages.push_back(m);
    }
    const Tracker tracker = run_tracker(setup, config, messages);
    if (!span) span = tracker.output_span();
    nodes.push_back({it.key(), tracker.reports()});
  }
  ASSERT_EQ(nodes.size(), 2u);
  ASSERT_TRUE(span);
  const auto score = [&](const std::vector<TrackReport>& reports) {
    return eval::evaluate(truth, reports, span->first, std::min(span->second, scenario.duration_s),
                          config.output_rate_hz, eval::EvalConfig{});
  };
  const auto nees = [](const eval::Evaluation& e, std::string_view name) {
    for (const auto& t : e.targets) {
      if (t.name == name) return t.nees_mean;
    }
    return -1.0;
  };

  const eval::Evaluation site_a = score(nodes[0].reports);
  const eval::Evaluation site_b = score(nodes[1].reports);
  FusionConfig fusion;
  const eval::Evaluation ci = score(fuse_nodes(nodes, fusion).reports());
  fusion.combine = Combine::Independent;
  const eval::Evaluation independent = score(fuse_nodes(nodes, fusion).reports());

  EXPECT_LT(ci.mean_gospa(), site_a.mean_gospa() + 1.0);
  EXPECT_LT(ci.mean_gospa(), 0.5 * site_b.mean_gospa());
  EXPECT_LE(ci.false_track_count(), 1);
  for (const auto& t : ci.targets) {
    EXPECT_GT(t.tracked_fraction, 0.9) << t.name;
    EXPECT_LE(t.id_switches + t.gaps, 1) << t.name;
  }
  EXPECT_LT(nees(ci, "intruder"), nees(independent, "intruder"));
  EXPECT_LT(nees(ci, "airliner"), nees(independent, "airliner"));
}

}  // namespace
}  // namespace skytrack

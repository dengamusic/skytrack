#include "skytrack/fusion/track_fusion.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>

#include "skytrack/tracking/assignment.hpp"
#include "skytrack/tracking/covariance_intersection.hpp"

namespace skytrack {
namespace {

constexpr std::string_view kUnknownClass = "unknown";

Gaussian predict_to(const TrackReport& r, double t, const ProcessNoise& q) {
  const double dt = t - r.t_s;
  return dt > 0.0 ? ncv_predict(r.state, dt, q) : r.state;
}

/// Normalised distance between two estimates of one state, ignoring their
/// cross-correlation. Positive correlation makes the true difference smaller,
/// so this errs towards associating.
double distance2(const Gaussian& a, const Gaussian& b) {
  const Vec6 d = a.x - b.x;
  const auto ldlt = (a.P + b.P).ldlt();
  if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) return kForbidden;
  return d.dot(ldlt.solve(d));
}

bool identities_conflict(const std::string& a, const std::string& b) { return !a.empty() && !b.empty() && a != b; }

Gaussian information_sum(const std::vector<Gaussian>& estimates) {
  Mat6 info = Mat6::Zero();
  Vec6 weighted = Vec6::Zero();
  for (const Gaussian& g : estimates) {
    const Mat6 inv = g.P.inverse();
    info += inv;
    weighted += inv * g.x;
  }
  Gaussian out;
  out.P = info.inverse();
  out.P = 0.5 * (out.P + out.P.transpose()).eval();
  out.x = out.P * weighted;
  return out;
}

}  // namespace

std::string_view to_string(Combine combine) {
  switch (combine) {
    case Combine::CovarianceIntersection: return "ci";
    case Combine::Independent: return "independent";
    case Combine::Best: return "best";
  }
  return "?";
}

std::optional<Combine> parse_combine(std::string_view name) {
  for (const Combine c : {Combine::CovarianceIntersection, Combine::Independent, Combine::Best}) {
    if (name == to_string(c)) return c;
  }
  return std::nullopt;
}

FusionCentre::FusionCentre(FusionConfig config) : cfg_(std::move(config)) {}

Gaussian FusionCentre::member_estimate(const Member& m, double t) const {
  return predict_to(m.last, t, cfg_.process_noise);
}

Gaussian FusionCentre::estimate(const Global& g, double t, Combine combine) const {
  std::vector<Gaussian> estimates;
  estimates.reserve(g.members.size());
  // A node track that disagrees right now may be about to be split off:
  // leave it out rather than let it drag the estimate (the first never disagrees).
  for (const Member& m : g.members) {
    if (m.disagreements == 0) estimates.push_back(member_estimate(m, t));
  }
  if (estimates.empty()) {  // cannot happen while the first member never disagrees; be safe anyway
    for (const Member& m : g.members) estimates.push_back(member_estimate(m, t));
  }
  switch (combine) {
    case Combine::CovarianceIntersection: {
      Gaussian fused = estimates.front();
      for (std::size_t i = 1; i < estimates.size(); ++i) fused = covariance_intersection(fused, estimates[i]).fused;
      return fused;
    }
    case Combine::Independent:
      return information_sum(estimates);
    case Combine::Best:
      return *std::min_element(estimates.begin(), estimates.end(), [](const Gaussian& a, const Gaussian& b) {
        return a.P.topLeftCorner<3, 3>().determinant() < b.P.topLeftCorner<3, 3>().determinant();
      });
  }
  return estimates.front();
}

bool FusionCentre::has_node(const Global& g, const std::string& node) const {
  return std::any_of(g.members.begin(), g.members.end(), [&](const Member& m) { return m.node == node; });
}

void FusionCentre::fuse(double t, const std::vector<std::pair<std::string, TrackReport>>& reports) {
  // Confirmed reports by (node, local id); ordered, so every run is the same.
  std::map<std::pair<std::string, int>, const TrackReport*> fresh;
  std::set<std::string> reporting;  // nodes that published at t: they list every track they hold
  for (const auto& [node, r] : reports) {
    reporting.insert(node);
    if (r.status != TrackStatus::Confirmed) continue;
    fresh[{node, r.track_id}] = &r;
    ++stats_.node_reports;
  }

  // 1. Bring associated node tracks up to date; forget those their node has dropped.
  for (Global& g : globals_) {
    for (Member& m : g.members) {
      const auto it = fresh.find({m.node, m.local_id});
      if (it == fresh.end()) continue;
      m.last = *it->second;
      fresh.erase(it);
    }
    std::erase_if(g.members, [&](const Member& m) {
      return m.last.t_s < t && (reporting.contains(m.node) || t - m.last.t_s > cfg_.max_report_age_s);
    });
  }
  std::erase_if(globals_, [](const Global& g) { return g.members.empty(); });

  // 2. Split off node tracks that keep disagreeing with the longest-held one.
  struct Loose {
    std::string node;
    TrackReport report;
    int not_with = 0;  // the global track it was just split from
  };
  std::vector<Loose> loose;
  for (Global& g : globals_) {
    // The longest-held member is the reference; if the previous one ended, a new one takes over.
    g.members.front().disagreements = 0;
    if (g.members.size() < 2) continue;
    const Gaussian anchor = member_estimate(g.members.front(), t);
    const std::string& identity = g.members.front().last.identity;
    for (auto it = g.members.begin() + 1; it != g.members.end();) {
      it->disagreements = distance2(member_estimate(*it, t), anchor) > cfg_.release_chi2 ? it->disagreements + 1 : 0;
      // The same transponder is the same target, however far apart the nodes
      // place it: it stays, but out of the estimate while it disagrees.
      const bool same_identity = !identity.empty() && it->last.identity == identity;
      if (it->disagreements < cfg_.release_after || same_identity) {
        ++it;
        continue;
      }
      loose.push_back({it->node, it->last, g.id});
      suspects_[{it->node, it->local_id}] = t;
      it = g.members.erase(it);
      ++stats_.releases;
    }
  }
  for (const auto& [key, r] : fresh) loose.push_back({key.first, *r, 0});

  // 3. Associate the rest, one node at a time: each may join a global track
  // that has no track from its node yet, or start one.
  std::set<std::string> nodes;
  for (const Loose& l : loose) nodes.insert(l.node);
  for (const std::string& node : nodes) {
    std::vector<const Loose*> rows;
    for (const Loose& l : loose) {
      if (l.node == node) rows.push_back(&l);
    }
    std::vector<std::size_t> cols;
    for (std::size_t i = 0; i < globals_.size(); ++i) {
      if (!has_node(globals_[i], node)) cols.push_back(i);
    }
    Eigen::MatrixXd cost(static_cast<Eigen::Index>(rows.size()), static_cast<Eigen::Index>(cols.size()));
    for (std::size_t c = 0; c < cols.size(); ++c) {
      const Global& g = globals_[cols[c]];
      const Gaussian fused = estimate(g, t, Combine::CovarianceIntersection);
      std::string identity;
      for (const Member& m : g.members) {
        if (identity.empty()) identity = m.last.identity;
      }
      for (std::size_t r = 0; r < rows.size(); ++r) {
        const TrackReport& report = rows[r]->report;
        double d2 = kForbidden;
        if (g.id == rows[r]->not_with || identities_conflict(identity, report.identity)) {
          d2 = kForbidden;
        } else if (!identity.empty() && identity == report.identity) {
          d2 = 0.0;  // the same transponder or Remote ID: one target, whatever the kinematics
        } else {
          d2 = distance2(predict_to(report, t, cfg_.process_noise), fused);
          if (d2 > cfg_.gate_chi2) d2 = kForbidden;
        }
        cost(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c)) = d2;
      }
    }
    const std::vector<int> assigned = solve_assignment_with_misses(
        cost, Eigen::VectorXd::Constant(static_cast<Eigen::Index>(rows.size()), cfg_.gate_chi2));
    for (std::size_t r = 0; r < rows.size(); ++r) {
      Member m{rows[r]->node, rows[r]->report.track_id, rows[r]->report, 0};
      if (assigned[r] >= 0) {
        globals_[cols[static_cast<std::size_t>(assigned[r])]].members.push_back(std::move(m));
        ++stats_.joins;
      } else if (std::sqrt(m.last.state.P.topLeftCorner<2, 2>().diagonal().maxCoeff()) >
                 cfg_.start_max_position_std_m) {
        ++stats_.held_back;
      } else {
        globals_.push_back(Global{next_id_++, {std::move(m)}});
        ++stats_.global_tracks;
      }
    }
  }

  // 4. Two global tracks with no node in common that agree are one target,
  // for example one each node started before the other was confirmed.
  for (std::size_t i = 0; i < globals_.size(); ++i) {
    for (std::size_t j = i + 1; j < globals_.size();) {
      Global& keep = globals_[i];
      const Global& drop = globals_[j];
      const bool disjoint = std::none_of(drop.members.begin(), drop.members.end(),
                                         [&](const Member& m) { return has_node(keep, m.node); });
      bool conflict = false;
      for (const Member& a : keep.members) {
        for (const Member& b : drop.members) conflict = conflict || identities_conflict(a.last.identity, b.last.identity);
      }
      if (!disjoint || conflict ||
          distance2(estimate(keep, t, Combine::CovarianceIntersection),
                    estimate(drop, t, Combine::CovarianceIntersection)) > cfg_.gate_chi2) {
        ++j;
        continue;
      }
      keep.members.insert(keep.members.end(), drop.members.begin(), drop.members.end());
      globals_.erase(globals_.begin() + static_cast<std::ptrdiff_t>(j));
      ++stats_.merges;
    }
  }

  // 5. Publish one report per global track, except split-offs no other node backs.
  std::erase_if(suspects_, [&](const auto& s) { return t - s.second > cfg_.max_report_age_s; });
  for (const Global& g : globals_) {
    bool corroborated = false;
    for (const Member& m : g.members) {
      const auto s = suspects_.find({m.node, m.local_id});
      if (s == suspects_.end()) {
        corroborated = true;
      } else {
        s->second = t;
      }
    }
    if (!corroborated) {
      ++stats_.withheld;
      continue;
    }
    for (const Member& m : g.members) suspects_.erase({m.node, m.local_id});
    TrackReport out;
    out.t_s = t;
    out.track_id = g.id;
    out.status = TrackStatus::Confirmed;
    out.classification = std::string(kUnknownClass);
    out.state = estimate(g, t, cfg_.combine);
    for (const Member& m : g.members) {
      const TrackReport& r = m.last;
      if (out.classification == kUnknownClass && !r.classification.empty()) out.classification = r.classification;
      if (out.identity.empty()) out.identity = r.identity;
      if (out.emitter.empty()) out.emitter = r.emitter;
      out.last_update_s = std::max(out.last_update_s, r.last_update_s);
      // The centre cannot publish before the last input it used arrived.
      out.t_published_s = std::max(out.t_published_s, r.t_published_s);
      out.sensors += (out.sensors.empty() ? "" : "+") + m.node;
    }
    reports_.push_back(std::move(out));
  }
}

FusionCentre fuse_nodes(const std::vector<NodeTracks>& nodes, const FusionConfig& config) {
  std::map<double, std::vector<std::pair<std::string, TrackReport>>> by_time;
  for (const NodeTracks& n : nodes) {
    for (const TrackReport& r : n.reports) by_time[r.t_s].emplace_back(n.node, r);
  }
  FusionCentre centre(config);
  for (const auto& [t, reports] : by_time) centre.fuse(t, reports);
  return centre;
}

}  // namespace skytrack

#include "skytrack/eval/metrics.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>
#include <stdexcept>

#include "skytrack/tracking/assignment.hpp"

namespace skytrack::eval {
namespace {

namespace fs = std::filesystem;

constexpr double kChi2_3dof_95 = 7.815;

std::vector<std::string> split(const std::string& line) {
  std::vector<std::string> fields;
  std::string field;
  std::istringstream in(line);
  while (std::getline(in, field, ',')) fields.push_back(field);
  if (!line.empty() && line.back() == ',') fields.emplace_back();
  return fields;
}

/// Reads a CSV file with a known header, calling row(fields) for each line.
template <typename RowFn>
void read_csv(const fs::path& path, const std::string& header, std::size_t columns, RowFn row) {
  std::ifstream file(path);
  if (!file) throw std::runtime_error("cannot open " + path.string());
  std::string line;
  if (!std::getline(file, line) || line != header) throw std::runtime_error(path.string() + ": unexpected header");
  std::size_t line_no = 1;
  while (std::getline(file, line)) {
    ++line_no;
    if (line.empty()) continue;
    const auto f = split(line);
    if (f.size() != columns) {
      throw std::runtime_error(path.string() + ":" + std::to_string(line_no) + ": expected " +
                               std::to_string(columns) + " fields");
    }
    try {
      row(f);
    } catch (const std::logic_error&) {
      throw std::runtime_error(path.string() + ":" + std::to_string(line_no) + ": bad number");
    }
  }
}

std::ofstream open_for_write(const fs::path& path) {
  std::ofstream file(path);
  if (!file) throw std::runtime_error("cannot write " + path.string());
  file << std::setprecision(8);
  return file;
}

double rms(double sum_sq, int n) { return n > 0 ? std::sqrt(sum_sq / n) : 0.0; }

}  // namespace

// ---- inputs --------------------------------------------------------------------

std::optional<Vec6> TruthTarget::at(double t) const {
  if (t_s.empty() || t < t_s.front() - 1e-9 || t > t_s.back() + 1e-9) return std::nullopt;
  const auto next = std::lower_bound(t_s.begin(), t_s.end(), t - 1e-9);
  const auto k = static_cast<std::size_t>(next - t_s.begin());
  if (k == 0 || std::abs(t_s[k] - t) < 1e-9) return states[k];
  const double f = (t - t_s[k - 1]) / (t_s[k] - t_s[k - 1]);
  return states[k - 1] + f * (states[k] - states[k - 1]);
}

std::vector<TruthTarget> read_truth_targets(const fs::path& path) {
  std::map<int, TruthTarget> by_id;
  read_csv(path, "t_s,target_id,name,class,east_m,north_m,up_m,vel_east_mps,vel_north_mps,vel_up_mps", 10,
           [&](const std::vector<std::string>& f) {
             TruthTarget& target = by_id[std::stoi(f[1])];
             target.id = std::stoi(f[1]);
             target.name = f[2];
             target.cls = f[3];
             target.t_s.push_back(std::stod(f[0]));
             Vec6 x;
             for (int k = 0; k < 6; ++k) x[k] = std::stod(f[static_cast<std::size_t>(4 + k)]);
             target.states.push_back(x);
           });
  std::vector<TruthTarget> targets;
  for (auto& [id, target] : by_id) targets.push_back(std::move(target));
  return targets;
}

std::vector<TruthTarget> truth_from_scenario(const Scenario& scenario) {
  std::vector<TruthTarget> truth;
  const auto steps = static_cast<long>(std::floor(scenario.duration_s * scenario.truth_rate_hz + 1e-9));
  for (const Target& target : scenario.targets) {
    TruthTarget tt{target.id, target.name, std::string(to_string(target.cls)), {}, {}};
    for (long i = 0; i <= steps; ++i) {
      const double t = static_cast<double>(i) / scenario.truth_rate_hz;
      if (!target.trajectory.active(t)) continue;
      const KinematicState s = target.trajectory.state(t);
      Vec6 x;
      x << s.position, s.velocity;
      tt.t_s.push_back(t);
      tt.states.push_back(x);
    }
    truth.push_back(std::move(tt));
  }
  return truth;
}

std::map<std::int64_t, Origin> read_measurement_origins(const fs::path& path) {
  std::map<std::int64_t, Origin> origins;
  read_csv(path, "meas_id,sensor_id,target_id,t_true_s", 4, [&](const std::vector<std::string>& f) {
    origins[std::stoll(f[0])] = Origin{std::stoi(f[2]), std::stod(f[3])};
  });
  return origins;
}

std::map<std::int64_t, std::pair<int, std::string>> read_associations(const fs::path& path) {
  std::map<std::int64_t, std::pair<int, std::string>> associations;
  read_csv(path, "meas_id,sensor_id,t_s,track_id,outcome,d2", 6, [&](const std::vector<std::string>& f) {
    associations[std::stoll(f[0])] = {std::stoi(f[3]), f[4]};
  });
  return associations;
}

// ---- GOSPA ---------------------------------------------------------------------------

Gospa gospa(const std::vector<Vec3>& truth, const std::vector<Vec3>& estimates, double c, double p) {
  Gospa g;
  const auto n = static_cast<Eigen::Index>(truth.size());
  const auto m = static_cast<Eigen::Index>(estimates.size());
  const double cp = std::pow(c, p);
  if (n > 0 && m > 0) {
    // Assigning on min(d, c)^p gives the optimal GOSPA assignment (alpha = 2).
    Eigen::MatrixXd cost(n, m);
    for (Eigen::Index i = 0; i < n; ++i) {
      for (Eigen::Index j = 0; j < m; ++j) {
        const double d = (truth[static_cast<std::size_t>(i)] - estimates[static_cast<std::size_t>(j)]).norm();
        cost(i, j) = std::pow(std::min(d, c), p);
      }
    }
    const std::vector<int> assigned = solve_assignment(cost);
    for (Eigen::Index i = 0; i < n; ++i) {
      const int j = assigned[static_cast<std::size_t>(i)];
      if (j < 0) continue;
      const double d = (truth[static_cast<std::size_t>(i)] - estimates[static_cast<std::size_t>(j)]).norm();
      if (d >= c) continue;  // too far apart: a miss and a false track
      g.localisation_sum += std::pow(d, p);
      g.pairs.emplace_back(static_cast<int>(i), j);
      ++g.matched;
    }
  }
  g.missed = static_cast<int>(n) - g.matched;
  g.false_tracks = static_cast<int>(m) - g.matched;
  g.total = std::pow(g.localisation_sum + 0.5 * cp * (g.missed + g.false_tracks), 1.0 / p);
  return g;
}

// ---- scoring -------------------------------------------------------------------------------

namespace {

struct TargetAccumulator {
  int present = 0;
  int tracked = 0;
  std::optional<int> last_track;
  bool lost_after_tracked = false;
  double sum_h2 = 0.0, sum_v2 = 0.0, sum_vel2 = 0.0, sum_nees = 0.0;
  int nees_ok = 0;
  int class_ok = 0;
};

}  // namespace

Evaluation evaluate(const std::vector<TruthTarget>& truth, const std::vector<TrackReport>& reports, double first_s,
                    double last_s, double rate_hz, const EvalConfig& config, const std::map<std::int64_t, Origin>* origins,
                    const std::map<std::int64_t, std::pair<int, std::string>>* associations) {
  if (!(rate_hz > 0.0) || last_s < first_s) throw std::invalid_argument("bad evaluation time span");
  Evaluation ev;
  ev.config = config;
  const double period = 1.0 / rate_hz;
  const long ticks = std::lround((last_s - first_s) * rate_hz) + 1;
  ev.duration_s = static_cast<double>(ticks) * period;

  std::map<long, std::vector<const TrackReport*>> by_tick;
  for (const TrackReport& r : reports) {
    if (r.status != TrackStatus::Confirmed) continue;
    const long k = std::lround((r.t_s - first_s) * rate_hz);
    if (k < 0 || k >= ticks || std::abs(first_s + static_cast<double>(k) * period - r.t_s) > 1e-6) continue;
    by_tick[k].push_back(&r);
  }

  std::vector<TargetAccumulator> acc(truth.size());
  ev.targets.resize(truth.size());
  std::map<int, TrackScore> track_scores;

  for (long k = 0; k < ticks; ++k) {
    const double t = first_s + static_cast<double>(k) * period;
    std::vector<int> present;
    std::vector<Vec6> truth_states;
    for (std::size_t i = 0; i < truth.size(); ++i) {
      if (const auto x = truth[i].at(t)) {
        present.push_back(static_cast<int>(i));
        truth_states.push_back(*x);
      }
    }
    const auto it = by_tick.find(k);
    static const std::vector<const TrackReport*> kNone;
    const auto& estimates = it == by_tick.end() ? kNone : it->second;

    std::vector<Vec3> tp, ep;
    for (const Vec6& x : truth_states) tp.push_back(x.head<3>());
    for (const TrackReport* r : estimates) ep.push_back(r->state.x.head<3>());
    Gospa g = gospa(tp, ep, config.gospa_c_m, config.gospa_p);

    for (const TrackReport* r : estimates) {
      TrackScore& ts = track_scores[r->track_id];
      ts.id = r->track_id;
      ts.confirmed_s += period;
    }
    std::vector<int> match_of_truth(present.size(), -1);
    for (const auto& [ti, ei] : g.pairs) match_of_truth[static_cast<std::size_t>(ti)] = ei;

    for (std::size_t pi = 0; pi < present.size(); ++pi) {
      const auto ti = static_cast<std::size_t>(present[pi]);
      TargetAccumulator& a = acc[ti];
      TargetScore& score = ev.targets[ti];
      ++a.present;
      const int ei = match_of_truth[pi];
      if (ei < 0) {
        if (a.tracked > 0) a.lost_after_tracked = true;
        continue;
      }
      const TrackReport& r = *estimates[static_cast<std::size_t>(ei)];
      const Vec6& x = truth_states[pi];
      ++a.tracked;
      if (!score.first_track_s) score.first_track_s = t;
      if (a.lost_after_tracked) {
        ++score.gaps;
        a.lost_after_tracked = false;
      }
      if (a.last_track && *a.last_track != r.track_id) ++score.id_switches;
      if (score.track_ids.empty() || score.track_ids.back() != r.track_id) score.track_ids.push_back(r.track_id);
      a.last_track = r.track_id;

      const Vec3 e = r.state.x.head<3>() - x.head<3>();
      const double nees = e.dot(r.state.P.topLeftCorner<3, 3>().ldlt().solve(e));
      a.sum_h2 += e.head<2>().squaredNorm();
      a.sum_v2 += e.z() * e.z();
      a.sum_vel2 += (r.state.x.tail<3>() - x.tail<3>()).squaredNorm();
      a.sum_nees += nees;
      if (nees <= kChi2_3dof_95) ++a.nees_ok;
      if (r.classification == truth[ti].cls) {
        ++a.class_ok;
        if (!score.first_classified_s) score.first_classified_s = t;
      }

      TrackScore& ts = track_scores[r.track_id];
      ts.on_target_s += period;
      ++ts.matched_ticks[truth[ti].id];
      ev.matches.push_back({t, truth[ti].id, r.track_id, e.head<2>().norm(), e.z(), nees});
    }
    ev.times.push_back(t);
    ev.gospa_series.push_back(std::move(g));
  }

  for (std::size_t i = 0; i < truth.size(); ++i) {
    const TargetAccumulator& a = acc[i];
    TargetScore& s = ev.targets[i];
    s.id = truth[i].id;
    s.name = truth[i].name;
    s.cls = truth[i].cls;
    s.present_s = a.present * period;
    s.tracked_fraction = a.present > 0 ? static_cast<double>(a.tracked) / a.present : 0.0;
    s.rmse_horizontal_m = rms(a.sum_h2, a.tracked);
    s.rmse_vertical_m = rms(a.sum_v2, a.tracked);
    s.rmse_velocity_mps = rms(a.sum_vel2, a.tracked);
    s.nees_mean = a.tracked > 0 ? a.sum_nees / a.tracked : 0.0;
    s.nees_within_95 = a.tracked > 0 ? static_cast<double>(a.nees_ok) / a.tracked : 0.0;
    s.class_correct = a.tracked > 0 ? static_cast<double>(a.class_ok) / a.tracked : 0.0;
  }
  for (auto& [id, ts] : track_scores) ev.tracks.push_back(ts);

  if (origins != nullptr) {
    for (const auto& [meas_id, origin] : *origins) {
      if (origin.target_id < 0) continue;
      for (TargetScore& s : ev.targets) {
        if (s.id == origin.target_id && (!s.first_detection_s || origin.t_true_s < *s.first_detection_s)) {
          s.first_detection_s = origin.t_true_s;
        }
      }
    }
  }

  if (origins != nullptr && associations != nullptr) {
    // Each confirmed track's main target is the one it was matched to most often.
    std::map<int, int> main_target;
    for (const TrackScore& ts : ev.tracks) {
      int best = -1, best_ticks = 0;
      for (const auto& [target, n] : ts.matched_ticks) {
        if (n > best_ticks) {
          best = target;
          best_ticks = n;
        }
      }
      main_target[ts.id] = best;
    }
    AssociationScore as;
    for (const auto& [meas_id, origin] : *origins) {
      const bool is_false = origin.target_id < 0;
      (is_false ? as.false_alarms : as.target_detections) += 1;
      const auto a = associations->find(meas_id);
      if (a == associations->end()) continue;
      const auto& [track_id, outcome] = a->second;
      if (outcome != "updated" && outcome != "initiated") continue;
      const auto main = main_target.find(track_id);
      if (main == main_target.end()) continue;  // never a confirmed track
      if (is_false) {
        ++as.false_alarms_used;
      } else {
        ++as.target_detections_used;
        if (main->second >= 0 && main->second != origin.target_id) ++as.wrong_target_updates;
      }
    }
    ev.association = as;
  }
  return ev;
}

double Evaluation::mean_gospa() const {
  if (gospa_series.empty()) return 0.0;
  double sum = 0.0;
  for (const Gospa& g : gospa_series) sum += g.total;
  return sum / static_cast<double>(gospa_series.size());
}

int Evaluation::false_track_count() const {
  return static_cast<int>(std::count_if(tracks.begin(), tracks.end(), [](const TrackScore& t) { return t.is_false(); }));
}

double Evaluation::false_tracks_per_hour() const {
  return duration_s > 0.0 ? false_track_count() * 3600.0 / duration_s : 0.0;
}

Json Evaluation::to_json() const {
  Json j;
  double loc = 0.0, missed = 0.0, false_tracks = 0.0;
  int matched = 0;
  for (const Gospa& g : gospa_series) {
    loc += g.localisation_sum;
    matched += g.matched;
    missed += g.missed;
    false_tracks += g.false_tracks;
  }
  const auto n = static_cast<double>(std::max<std::size_t>(gospa_series.size(), 1));
  j["gospa"] = {{"c_m", config.gospa_c_m},
                {"p", config.gospa_p},
                {"mean_m", mean_gospa()},
                {"mean_localisation_error_m", matched > 0 ? std::pow(loc / matched, 1.0 / config.gospa_p) : 0.0},
                {"mean_missed_targets", missed / n},
                {"mean_false_tracks", false_tracks / n}};
  j["duration_s"] = duration_s;

  Json targets_json = Json::array();
  for (const TargetScore& s : targets) {
    Json t;
    t["id"] = s.id;
    t["name"] = s.name;
    t["class"] = s.cls;
    t["present_s"] = s.present_s;
    t["tracked_fraction"] = s.tracked_fraction;
    t["first_detection_s"] = s.first_detection_s ? Json(*s.first_detection_s) : Json(nullptr);
    t["first_track_s"] = s.first_track_s ? Json(*s.first_track_s) : Json(nullptr);
    t["time_to_track_s"] =
        s.first_detection_s && s.first_track_s ? Json(*s.first_track_s - *s.first_detection_s) : Json(nullptr);
    t["first_classified_s"] = s.first_classified_s ? Json(*s.first_classified_s) : Json(nullptr);
    t["time_to_classify_s"] =
        s.first_detection_s && s.first_classified_s ? Json(*s.first_classified_s - *s.first_detection_s) : Json(nullptr);
    t["track_ids"] = s.track_ids;
    t["id_switches"] = s.id_switches;
    t["gaps"] = s.gaps;
    t["rmse_horizontal_m"] = s.rmse_horizontal_m;
    t["rmse_vertical_m"] = s.rmse_vertical_m;
    t["rmse_velocity_mps"] = s.rmse_velocity_mps;
    t["nees_mean"] = s.nees_mean;
    t["nees_within_95"] = s.nees_within_95;
    t["class_correct"] = s.class_correct;
    targets_json.push_back(std::move(t));
  }
  j["targets"] = std::move(targets_json);

  Json tracks_json = Json::array();
  for (const TrackScore& ts : tracks) {
    Json matched_to = Json::object();
    for (const auto& [target, ticks] : ts.matched_ticks) matched_to[std::to_string(target)] = ticks;
    tracks_json.push_back({{"id", ts.id},
                           {"confirmed_s", ts.confirmed_s},
                           {"on_target_s", ts.on_target_s},
                           {"false", ts.is_false()},
                           {"matched_ticks", matched_to}});
  }
  j["tracks"] = {{"confirmed", tracks.size()},
                 {"false", false_track_count()},
                 {"false_per_hour", false_tracks_per_hour()},
                 {"list", std::move(tracks_json)}};

  if (association) {
    const AssociationScore& a = *association;
    j["association"] = {{"target_detections", a.target_detections},
                        {"target_detections_used", a.target_detections_used},
                        {"false_alarms", a.false_alarms},
                        {"false_alarms_used", a.false_alarms_used},
                        {"wrong_target_updates", a.wrong_target_updates}};
  }
  return j;
}

void write_evaluation(const Evaluation& ev, const fs::path& dir) {
  fs::create_directories(dir);
  {
    auto file = open_for_write(dir / "metrics.json");
    file << ev.to_json().dump(2) << '\n';
  }
  {
    auto file = open_for_write(dir / "gospa.csv");
    file << "t_s,gospa_m,matched,missed,false_tracks,localisation_sum\n";
    for (std::size_t k = 0; k < ev.times.size(); ++k) {
      const Gospa& g = ev.gospa_series[k];
      file << ev.times[k] << ',' << g.total << ',' << g.matched << ',' << g.missed << ',' << g.false_tracks << ','
           << g.localisation_sum << '\n';
    }
  }
  {
    auto file = open_for_write(dir / "matches.csv");
    file << "t_s,target_id,track_id,error_horizontal_m,error_vertical_m,nees\n";
    for (const Match& m : ev.matches) {
      file << m.t_s << ',' << m.target_id << ',' << m.track_id << ',' << m.error_horizontal_m << ','
           << m.error_vertical_m << ',' << m.nees << '\n';
    }
  }
}

}  // namespace skytrack::eval

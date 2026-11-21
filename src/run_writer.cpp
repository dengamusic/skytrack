#include "skytrack/run_writer.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <stdexcept>

namespace skytrack {
namespace {

namespace fs = std::filesystem;

std::ofstream open_for_write(const fs::path& path) {
  std::ofstream file(path);
  if (!file) throw std::runtime_error("cannot write " + path.string());
  file << std::setprecision(10);
  return file;
}

void write_json(const fs::path& path, const Json& j) {
  auto file = open_for_write(path);
  file << j.dump(2) << '\n';
}

}  // namespace

Json summary_json(const Scenario& scenario, const SimulationResult& result) {
  Json j;
  j["scenario"] = scenario.name;
  j["seed"] = scenario.seed;
  j["duration_s"] = scenario.duration_s;
  j["messages"] = result.messages.size();
  Json sensors = Json::array();
  for (const SensorStats& s : result.stats) {
    Json per_target = Json::object();
    for (const auto& [target_id, count] : s.detections_per_target) per_target[std::to_string(target_id)] = count;
    sensors.push_back({{"id", s.sensor_id},
                       {"type", std::string(to_string(s.type))},
                       {"scans", s.scans},
                       {"messages", s.messages},
                       {"dropped", s.dropped},
                       {"detections", s.detections},
                       {"false_alarms", s.false_alarms},
                       {"mean_latency_s", s.mean_latency_s()},
                       {"detections_per_target", per_target}});
  }
  j["sensors"] = std::move(sensors);
  return j;
}

Json nominal_sensors_json(const Scenario& scenario) {
  // What the tracker may know: everything about the sensors except their errors.
  Json nominal;
  nominal["origin"] = scenario.source.at("origin");
  Json sensor_list = Json::array();
  for (Json s : scenario.source.at("sensors")) {
    s.erase("errors");
    sensor_list.push_back(std::move(s));
  }
  nominal["sensors"] = std::move(sensor_list);
  return nominal;
}

void write_run(const Scenario& scenario, const SimulationResult& result, const fs::path& out_dir) {
  fs::create_directories(out_dir / "truth");

  write_json(out_dir / "scenario.json", scenario.source);
  write_json(out_dir / "sensors.json", nominal_sensors_json(scenario));

  {
    auto file = open_for_write(out_dir / "measurements.jsonl");
    for (const SensorMessage& msg : result.messages) file << to_observed_json(msg).dump() << '\n';
  }

  write_json(out_dir / "summary.json", summary_json(scenario, result));

  // ---- ground truth ----
  {
    auto file = open_for_write(out_dir / "truth" / "messages.csv");
    file << "sensor_id,seq,t_true_s,t_arrival_s,detections\n";
    for (const SensorMessage& msg : result.messages) {
      file << msg.sensor_id << ',' << msg.seq << ',' << msg.t_true_s << ',' << msg.t_arrival_s << ','
           << msg.detections.size() << '\n';
    }
  }
  {
    auto file = open_for_write(out_dir / "truth" / "measurement_origins.csv");
    file << "meas_id,sensor_id,target_id,t_true_s\n";
    for (const SensorMessage& msg : result.messages) {
      for (const Detection& d : msg.detections) {
        file << d.meas_id << ',' << msg.sensor_id << ',' << d.origin_target_id << ',' << d.t_true_s << '\n';
      }
    }
  }
  {
    auto file = open_for_write(out_dir / "truth" / "targets.csv");
    file << "t_s,target_id,name,class,east_m,north_m,up_m,vel_east_mps,vel_north_mps,vel_up_mps\n";
    const auto steps = static_cast<long>(std::floor(scenario.duration_s * scenario.truth_rate_hz + 1e-9));
    for (long i = 0; i <= steps; ++i) {
      const double t = static_cast<double>(i) / scenario.truth_rate_hz;
      for (const Target& target : scenario.targets) {
        if (!target.trajectory.active(t)) continue;
        const KinematicState s = target.trajectory.state(t);
        file << t << ',' << target.id << ',' << target.name << ',' << to_string(target.cls) << ','
             << s.position.x() << ',' << s.position.y() << ',' << s.position.z() << ',' << s.velocity.x() << ','
             << s.velocity.y() << ',' << s.velocity.z() << '\n';
      }
    }
  }
  {
    Json errors = Json::object();
    for (const Json& s : scenario.source.at("sensors")) {
      errors[s.at("id").get<std::string>()] = s.contains("errors") ? s.at("errors") : Json::object();
    }
    write_json(out_dir / "truth" / "sensor_errors.json", errors);
  }
}

}  // namespace skytrack

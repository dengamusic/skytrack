#include "skytrack/simulator.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>

namespace skytrack {

Rng detection_rng(const Scenario& scenario, const Sensor& sensor) {
  return Rng(mix_seed(scenario.seed, fnv1a64(sensor.id())));
}

Rng network_rng(const Scenario& scenario, const Sensor& sensor) {
  return Rng(mix_seed(scenario.seed, fnv1a64(sensor.id() + "#network")));
}

SensorMessage make_message(const Sensor& sensor, Scan scan, std::uint64_t seq, double latency_s,
                           double& last_arrival_s, SensorStats& stats) {
  const bool stamped = sensor.common().timestamps == TimestampSource::Sensor;
  SensorMessage msg;
  msg.sensor_id = sensor.id();
  msg.sensor_type = sensor.type();
  msg.seq = seq;  // counts scans, so the receiver can spot lost messages
  msg.t_true_s = scan.t_true_s;
  msg.t_arrival_s = std::max(scan.t_ready_s + latency_s, last_arrival_s);
  last_arrival_s = msg.t_arrival_s;
  if (stamped) msg.t_sensor_s = sensor.sensor_clock(scan.t_true_s);
  msg.gimbal = scan.gimbal;
  for (Detection& d : scan.detections) {
    if (stamped) d.t_sensor_s = sensor.sensor_clock(d.t_true_s);
    if (d.origin_target_id < 0) {
      ++stats.false_alarms;
    } else {
      ++stats.detections_per_target[d.origin_target_id];
    }
  }
  stats.detections += scan.detections.size();
  stats.latency_sum_s += msg.t_arrival_s - scan.t_ready_s;
  ++stats.messages;
  msg.detections = std::move(scan.detections);
  return msg;
}

SimulationResult simulate(const Scenario& scenario, const std::set<std::string>& skip) {
  SimulationResult result;
  const World world{scenario.targets, scenario.frame};

  for (const auto& sensor_ptr : scenario.sensors) {
    const Sensor& sensor = *sensor_ptr;
    const SensorCommon& c = sensor.common();
    if (skip.contains(c.id)) continue;
    Rng detections = detection_rng(scenario, sensor);
    Rng network = network_rng(scenario, sensor);

    SensorStats stats;
    stats.sensor_id = c.id;
    stats.type = sensor.type();
    double last_arrival = -std::numeric_limits<double>::infinity();

    for (std::uint64_t k = 0;; ++k) {
      const double t = c.phase_s + static_cast<double>(k) * sensor.period_s();
      if (t > scenario.duration_s) break;
      Scan scan = sensor.observe(t, world, detections);
      ++stats.scans;

      // Draw both network values every scan so the stream stays aligned.
      const bool dropped = network.bernoulli(c.errors.dropout_prob);
      const double latency = std::max(0.0, network.normal(c.errors.latency_mean_s, c.errors.latency_jitter_s));
      if (dropped) {
        ++stats.dropped;
        continue;
      }
      result.messages.push_back(make_message(sensor, std::move(scan), k, latency, last_arrival, stats));
    }
    result.stats.push_back(std::move(stats));
  }

  // Merge into arrival order. Messages were appended sensor by sensor and in
  // scan order, so a stable sort breaks ties by scenario order, then seq.
  std::stable_sort(result.messages.begin(), result.messages.end(),
                   [](const SensorMessage& a, const SensorMessage& b) { return a.t_arrival_s < b.t_arrival_s; });

  std::int64_t next_id = 0;
  for (SensorMessage& msg : result.messages) {
    for (Detection& d : msg.detections) d.meas_id = next_id++;
  }
  return result;
}

}  // namespace skytrack

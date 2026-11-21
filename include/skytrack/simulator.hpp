#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "skytrack/measurements.hpp"
#include "skytrack/scenario.hpp"

namespace skytrack {

struct SensorStats {
  std::string sensor_id;
  SensorType type = SensorType::Radar;
  std::size_t scans = 0;
  std::size_t messages = 0;
  std::size_t dropped = 0;
  std::size_t detections = 0;
  std::size_t false_alarms = 0;
  double latency_sum_s = 0.0;
  std::map<int, std::size_t> detections_per_target;

  double mean_latency_s() const { return messages ? latency_sum_s / static_cast<double>(messages) : 0.0; }
};

struct SimulationResult {
  std::vector<SensorMessage> messages;  // in arrival order, as a fusion host would receive them
  std::vector<SensorStats> stats;       // one per sensor, in scenario order
};

/// Runs every sensor over the scenario and merges their messages.
///
/// Each sensor gets two random streams derived from the scenario seed and its
/// id: one for what it detects and one for its network (latency, dropouts).
/// Changing one sensor's settings therefore leaves the others' output alone,
/// and changing only the latency leaves the detections alone.
///
/// Sensors named in `skip` are left out, for a caller that runs them itself
/// (a camera pointed in closed loop by the tracker's output).
SimulationResult simulate(const Scenario& scenario, const std::set<std::string>& skip = {});

/// Random streams of one sensor: what it detects, and its network.
Rng detection_rng(const Scenario& scenario, const Sensor& sensor);
Rng network_rng(const Scenario& scenario, const Sensor& sensor);

/// The message a scan becomes: stamped with the sensor clock (if it stamps),
/// delivered `latency_s` after it was ready, never before the sensor's
/// previous message (one ordered stream, as over TCP). Updates stats.
SensorMessage make_message(const Sensor& sensor, Scan scan, std::uint64_t seq, double latency_s,
                           double& last_arrival_s, SensorStats& stats);

}  // namespace skytrack

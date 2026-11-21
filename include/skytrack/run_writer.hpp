#pragma once

#include <filesystem>

#include "skytrack/json.hpp"
#include "skytrack/scenario.hpp"
#include "skytrack/simulator.hpp"

namespace skytrack {

/// Writes one run to out_dir:
///
///   scenario.json           the scenario as run (seed included)
///   sensors.json            nominal sensor setup: what a tracker is allowed to know
///   measurements.jsonl      one message per line, in arrival order
///   summary.json            counts per sensor and target
///   truth/targets.csv       target states at truth_rate_hz
///   truth/measurement_origins.csv  which target (or false alarm) each detection came from
///   truth/messages.csv      true scan time and arrival time of every message
///   truth/sensor_errors.json       the injected errors per sensor
void write_run(const Scenario& scenario, const SimulationResult& result, const std::filesystem::path& out_dir);

Json summary_json(const Scenario& scenario, const SimulationResult& result);
/// The nominal setup a tracker may know (sensors.json): origin and sensors, without their errors.
Json nominal_sensors_json(const Scenario& scenario);

}  // namespace skytrack

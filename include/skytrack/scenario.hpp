#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "skytrack/geo.hpp"
#include "skytrack/json.hpp"
#include "skytrack/sensor.hpp"
#include "skytrack/target.hpp"

namespace skytrack {

struct Scenario {
  std::string name;
  std::uint64_t seed = 1;
  double duration_s = 60.0;
  double truth_rate_hz = 10.0;
  geo::LocalFrame frame;
  std::vector<Target> targets;
  std::vector<std::unique_ptr<Sensor>> sensors;
  Json source;  // the scenario exactly as given
};

Json read_json_file(const std::filesystem::path& path);

/// Builds a scenario and checks it. Unknown fields are errors, so a typo in
/// a scenario file fails loudly instead of silently falling back to a default.
Scenario parse_scenario(const Json& j);
Scenario load_scenario(const std::filesystem::path& path);

/// The scenario origin {"lat_deg", "lon_deg", "alt_m"} as a local ENU frame.
geo::LocalFrame parse_origin(const Json& j);

/// Builds one sensor from its JSON block. The tracker uses this to read the
/// nominal sensor setup from sensors.json.
std::unique_ptr<Sensor> parse_sensor(const Json& j);

}  // namespace skytrack

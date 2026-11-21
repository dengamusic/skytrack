#include <cstdint>
#include <exception>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include "skytrack/run_writer.hpp"
#include "skytrack/scenario.hpp"
#include "skytrack/simulator.hpp"

namespace {

constexpr const char* kUsage =
    R"(usage: skytrack_sim <scenario.json> <output-dir> [--seed N] [--duration SECONDS]

Simulates the targets and sensors in a scenario file. Writes what the sensors
report (measurements.jsonl, sensors.json) and the ground truth (truth/) to the
output directory.
)";

void print_summary(const skytrack::Scenario& sc, const skytrack::SimulationResult& res) {
  std::cout << "scenario '" << sc.name << "': " << sc.duration_s << " s, " << sc.targets.size() << " targets, "
            << sc.sensors.size() << " sensors, seed " << sc.seed << "\n\n";

  std::cout << std::left << std::setw(14) << "sensor" << std::setw(13) << "type" << std::right << std::setw(7)
            << "msgs" << std::setw(7) << "lost" << std::setw(8) << "dets" << std::setw(8) << "false"
            << std::setw(13) << "latency (s)" << '\n';
  for (const auto& st : res.stats) {
    std::cout << std::left << std::setw(14) << st.sensor_id << std::setw(13) << skytrack::to_string(st.type)
              << std::right << std::setw(7) << st.messages << std::setw(7) << st.dropped << std::setw(8)
              << st.detections << std::setw(8) << st.false_alarms << std::setw(13) << std::fixed
              << std::setprecision(3) << st.mean_latency_s() << '\n';
  }

  std::cout << "\ndetections per target\n" << std::left << std::setw(14) << "target";
  for (const auto& st : res.stats) std::cout << std::right << std::setw(13) << st.sensor_id;
  std::cout << '\n';
  for (const auto& target : sc.targets) {
    std::cout << std::left << std::setw(14) << target.name;
    for (const auto& st : res.stats) {
      const auto it = st.detections_per_target.find(target.id);
      std::cout << std::right << std::setw(13) << (it == st.detections_per_target.end() ? 0 : it->second);
    }
    std::cout << '\n';
  }
}

}  // namespace

int main(int argc, char** argv) {
  std::vector<std::string> positional;
  std::optional<std::uint64_t> seed;
  std::optional<double> duration;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "-h" || arg == "--help") {
      std::cout << kUsage;
      return 0;
    }
    if (arg == "--seed" || arg == "--duration") {
      if (i + 1 >= argc) {
        std::cerr << "skytrack_sim: " << arg << " needs a value\n";
        return 2;
      }
      const std::string value = argv[++i];
      try {
        if (arg == "--seed") {
          seed = std::stoull(value);
        } else {
          duration = std::stod(value);
        }
      } catch (const std::exception&) {
        std::cerr << "skytrack_sim: bad value for " << arg << ": " << value << '\n';
        return 2;
      }
      continue;
    }
    if (arg.rfind("--", 0) == 0) {
      std::cerr << "skytrack_sim: unknown option " << arg << "\n\n" << kUsage;
      return 2;
    }
    positional.push_back(arg);
  }
  if (positional.size() != 2) {
    std::cerr << kUsage;
    return 2;
  }

  try {
    skytrack::Json j = skytrack::read_json_file(positional[0]);
    if (seed) j["seed"] = *seed;
    if (duration) j["duration_s"] = *duration;
    const skytrack::Scenario scenario = skytrack::parse_scenario(j);
    const skytrack::SimulationResult result = skytrack::simulate(scenario);
    skytrack::write_run(scenario, result, positional[1]);
    print_summary(scenario, result);
    std::cout << "\nwrote " << result.messages.size() << " messages to " << positional[1] << '\n';
  } catch (const std::exception& e) {
    std::cerr << "skytrack_sim: " << e.what() << '\n';
    return 1;
  }
  return 0;
}

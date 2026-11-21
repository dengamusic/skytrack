// Runs a scenario with a gimbal camera pointed in closed loop by the
// tracker's own output, and writes the run and the live tracker's output.
//
//   skytrack_cue scenarios/cuas_baseline.json runs/cued
//   skytrack_eval runs/cued

#include <cstdio>
#include <exception>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <string_view>

#include "skytrack/cueing/cueing.hpp"
#include "skytrack/run_writer.hpp"
#include "skytrack/scenario.hpp"
#include "skytrack/tracking/track_io.hpp"

namespace {

int usage() {
  std::fprintf(stderr,
               "usage: skytrack_cue <scenario.json> <output-dir> [--seed N] [--config tracker.json] [--camera ID]\n"
               "  points the camera (default cam-ptz) at the tracks the tracker publishes while it runs;\n"
               "  writes the run as skytrack_sim does, and the live tracker's output in <output-dir>/tracker/\n");
  return 2;
}

}  // namespace

int main(int argc, char** argv) {
  namespace fs = std::filesystem;
  using namespace skytrack;
  if (argc < 3) return usage();
  const fs::path scenario_path = argv[1];
  const fs::path out_dir = argv[2];
  std::optional<std::uint64_t> seed;
  std::optional<fs::path> config_path;
  CueConfig cue;
  for (int i = 3; i < argc; ++i) {
    const std::string_view arg = argv[i];
    if (arg == "--seed" && i + 1 < argc) {
      seed = std::stoull(argv[++i]);
    } else if (arg == "--config" && i + 1 < argc) {
      config_path = argv[++i];
    } else if (arg == "--camera" && i + 1 < argc) {
      cue.camera_id = argv[++i];
    } else {
      return usage();
    }
  }

  try {
    Json j = read_json_file(scenario_path);
    if (seed) j["seed"] = *seed;
    const Scenario scenario = parse_scenario(j);
    const TrackerConfig config = config_path ? parse_tracker_config(read_json_file(*config_path)) : TrackerConfig{};
    const CuedRun run = run_cued(scenario, config, cue);
    write_run(scenario, run.sim, out_dir);
    write_tracker_output(run.tracker, out_dir / "tracker");
    const auto& s = run.cue;
    std::ofstream(out_dir / "tracker" / "cueing.json")
        << Json{{"camera", cue.camera_id},
                {"frames", s.frames},
                {"frames_on_target", s.frames_on_target},
                {"looks", s.looks},
                {"gave_up", s.gave_up}}
               .dump(2)
        << '\n';
    std::printf("%s: %zu frames, %.0f%% on a chosen track, %zu looks, %zu tracks not reached in time\nwrote %s\n",
                cue.camera_id.c_str(), s.frames,
                s.frames > 0 ? 100.0 * static_cast<double>(s.frames_on_target) / static_cast<double>(s.frames) : 0.0,
                s.looks, s.gave_up, out_dir.string().c_str());
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
  return 0;
}

// The fusion centre: reads the tracks each sensor node published and fuses
// them into one picture (track-to-track fusion).
//
//   skytrack_track runs/two_sites --sensors radar-1,cam-wide,... --out runs/two_sites/site-a
//   skytrack_track runs/two_sites --sensors radar-2,cam-b,...    --out runs/two_sites/site-b
//   skytrack_fuse runs/two_sites/site-a runs/two_sites/site-b --out runs/two_sites/fused
//   skytrack_eval runs/two_sites --tracks runs/two_sites/fused

#include <cstdio>
#include <exception>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "skytrack/fusion/track_fusion.hpp"
#include "skytrack/json.hpp"
#include "skytrack/scenario.hpp"
#include "skytrack/tracking/track_io.hpp"

namespace {

int usage() {
  std::fprintf(stderr,
               "usage: skytrack_fuse <node-dir>... --out DIR [--combine ci|independent|best]\n"
               "  reads  <node-dir>/tracks.csv and summary.json for each node (named after its directory)\n"
               "  writes DIR: tracks.csv and summary.json, which skytrack_eval reads like a tracker's\n");
  return 2;
}

/// What a node sends the centre: its confirmed lines of tracks.csv.
std::size_t confirmed_bytes(const std::filesystem::path& tracks_csv) {
  std::ifstream file(tracks_csv);
  std::string line;
  std::size_t bytes = 0;
  while (std::getline(file, line)) {
    if (line.find(",confirmed,") != std::string::npos) bytes += line.size() + 1;
  }
  return bytes;
}

}  // namespace

int main(int argc, char** argv) {
  namespace fs = std::filesystem;
  using namespace skytrack;
  std::vector<fs::path> node_dirs;
  fs::path out_dir;
  FusionConfig config;
  for (int i = 1; i < argc; ++i) {
    const std::string_view arg = argv[i];
    if (arg == "--out" && i + 1 < argc) {
      out_dir = argv[++i];
    } else if (arg == "--combine" && i + 1 < argc) {
      const auto combine = parse_combine(argv[++i]);
      if (!combine) return usage();
      config.combine = *combine;
    } else if (!arg.starts_with("--")) {
      node_dirs.emplace_back(arg);
    } else {
      return usage();
    }
  }
  if (node_dirs.empty() || out_dir.empty()) return usage();

  try {
    std::vector<NodeTracks> nodes;
    Json node_summaries = Json::array();
    double rate_hz = 0.0;
    for (const fs::path& dir : node_dirs) {
      const Json summary = read_json_file(dir / "summary.json");
      const double rate = summary.at("output_rate_hz").get<double>();
      if (rate_hz > 0.0 && rate != rate_hz) throw std::runtime_error("nodes publish at different rates");
      rate_hz = rate;
      // The centre predicts node tracks with the nodes' own motion model.
      const Json& q = summary.at("config").at("process_noise");
      config.process_noise.horizontal_m2_s3 = q.at("horizontal_m2_s3").get<double>();
      config.process_noise.vertical_m2_s3 = q.at("vertical_m2_s3").get<double>();

      NodeTracks node{fs::path(dir).lexically_normal().filename().string(), read_track_reports(dir / "tracks.csv")};
      if (node.node.empty()) node.node = dir.parent_path().filename().string();
      std::size_t confirmed = 0;
      for (const TrackReport& r : node.reports) confirmed += r.status == TrackStatus::Confirmed ? 1 : 0;
      const double span = summary.at("last_output_s").get<double>() - summary.at("first_output_s").get<double>();
      const std::size_t bytes = confirmed_bytes(dir / "tracks.csv");
      node_summaries.push_back({{"node", node.node},
                                {"confirmed_reports", confirmed},
                                {"uplink_bytes", bytes},
                                {"uplink_bytes_per_s", span > 0.0 ? static_cast<double>(bytes) / span : 0.0}});
      std::printf("%-10s %6zu confirmed reports, %.1f kB/s to the centre\n", node.node.c_str(), confirmed,
                  span > 0.0 ? 1e-3 * static_cast<double>(bytes) / span : 0.0);
      nodes.push_back(std::move(node));
    }

    const FusionCentre centre = fuse_nodes(nodes, config);
    fs::create_directories(out_dir);
    write_track_reports(centre.reports(), out_dir / "tracks.csv");

    const FusionStats& s = centre.stats();
    Json summary;
    summary["combine"] = std::string(to_string(config.combine));
    summary["nodes"] = node_summaries;
    summary["node_reports"] = s.node_reports;
    summary["global_tracks"] = s.global_tracks;
    summary["joins"] = s.joins;
    summary["releases"] = s.releases;
    summary["merges"] = s.merges;
    summary["held_back"] = s.held_back;
    summary["withheld"] = s.withheld;
    summary["output_rate_hz"] = rate_hz;
    if (!centre.reports().empty()) {
      summary["first_output_s"] = centre.reports().front().t_s;
      summary["last_output_s"] = centre.reports().back().t_s;
    }
    std::ofstream(out_dir / "summary.json") << summary.dump(2) << '\n';

    std::printf("fused (%s): %zu global tracks, %zu joins, %zu releases, %zu merges\nwrote %s\n",
                std::string(to_string(config.combine)).c_str(), s.global_tracks, s.joins, s.releases, s.merges,
                out_dir.string().c_str());
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
  return 0;
}

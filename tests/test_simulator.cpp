#include <gtest/gtest.h>

#include <filesystem>
#include <map>
#include <stdexcept>
#include <string>

#include "skytrack/scenario.hpp"
#include "skytrack/simulator.hpp"

using namespace skytrack;

namespace {

std::filesystem::path scenario_path(const std::string& name) {
  return std::filesystem::path(SKYTRACK_SOURCE_DIR) / "scenarios" / (name + ".json");
}

std::string dump_all(const SimulationResult& result) {
  std::string out;
  for (const SensorMessage& msg : result.messages) out += to_observed_json(msg).dump() + "\n";
  return out;
}

}  // namespace

TEST(Simulator, BundledScenariosLoad) {
  for (const char* name : {"single_drone", "cuas_baseline"}) {
    EXPECT_NO_THROW(load_scenario(scenario_path(name))) << name;
  }
}

TEST(Simulator, MessagesArriveInOrderWithConsistentIds) {
  const Scenario sc = load_scenario(scenario_path("cuas_baseline"));
  const SimulationResult result = simulate(sc);
  ASSERT_FALSE(result.messages.empty());

  std::map<std::string, std::uint64_t> last_seq;
  std::int64_t next_id = 0;
  for (std::size_t i = 0; i < result.messages.size(); ++i) {
    const SensorMessage& msg = result.messages[i];
    if (i > 0) {
      EXPECT_LE(result.messages[i - 1].t_arrival_s, msg.t_arrival_s);
    }
    EXPECT_GE(msg.t_arrival_s, msg.t_true_s);
    if (const auto it = last_seq.find(msg.sensor_id); it != last_seq.end()) {
      EXPECT_GT(msg.seq, it->second) << msg.sensor_id;  // each sensor's stream stays in order
    }
    last_seq[msg.sensor_id] = msg.seq;
    for (const Detection& d : msg.detections) {
      EXPECT_EQ(d.meas_id, next_id++);
      EXPECT_LE(d.t_true_s, msg.t_arrival_s);
    }
  }
}

TEST(Simulator, TimestampsCarryTheClockError) {
  const Scenario sc = load_scenario(scenario_path("cuas_baseline"));
  const SimulationResult result = simulate(sc);
  std::map<std::string, const Sensor*> sensors;
  for (const auto& s : sc.sensors) sensors[s->id()] = s.get();

  for (const SensorMessage& msg : result.messages) {
    const Sensor& sensor = *sensors.at(msg.sensor_id);
    if (sensor.common().timestamps == TimestampSource::Arrival) {
      EXPECT_FALSE(msg.t_sensor_s);
      for (const Detection& d : msg.detections) EXPECT_FALSE(d.t_sensor_s);
    } else {
      ASSERT_TRUE(msg.t_sensor_s);
      EXPECT_DOUBLE_EQ(*msg.t_sensor_s, sensor.sensor_clock(msg.t_true_s));
      for (const Detection& d : msg.detections) {
        ASSERT_TRUE(d.t_sensor_s);
        EXPECT_DOUBLE_EQ(*d.t_sensor_s, sensor.sensor_clock(d.t_true_s));
      }
    }
  }
}

TEST(Simulator, SameSeedSameRunDifferentSeedDifferentRun) {
  Json j = read_json_file(scenario_path("cuas_baseline"));
  const std::string first = dump_all(simulate(parse_scenario(j)));
  EXPECT_EQ(first, dump_all(simulate(parse_scenario(j))));
  j["seed"] = 12345;
  EXPECT_NE(first, dump_all(simulate(parse_scenario(j))));
}

TEST(Simulator, ChangingOneSensorLeavesTheOthersAlone) {
  Json j = read_json_file(scenario_path("cuas_baseline"));
  const SimulationResult before = simulate(parse_scenario(j));
  j["sensors"].at(std::size_t{0})["errors"]["latency_mean_s"] = 0.5;  // radar-1 only
  const SimulationResult after = simulate(parse_scenario(j));

  const auto detections_of = [](const SimulationResult& r, const std::string& id) {
    std::string out;
    for (const SensorMessage& m : r.messages) {
      if (m.sensor_id != id) continue;
      for (const Detection& d : m.detections) out += std::to_string(d.t_true_s) + ";";
    }
    return out;
  };
  EXPECT_EQ(detections_of(before, "cam-ptz"), detections_of(after, "cam-ptz"));
  EXPECT_EQ(detections_of(before, "radar-1"), detections_of(after, "radar-1"));  // only the timing moved
}

TEST(Simulator, ObservedJsonRoundTrips) {
  const SimulationResult result = simulate(load_scenario(scenario_path("cuas_baseline")));
  for (std::size_t i = 0; i < std::min<std::size_t>(500, result.messages.size()); ++i) {
    const Json once = to_observed_json(result.messages[i]);
    EXPECT_EQ(to_observed_json(from_observed_json(once)), once);
  }
}

TEST(Scenario, UnknownFieldIsAnError) {
  Json j = read_json_file(scenario_path("single_drone"));
  j["sensors"].at(std::size_t{0})["radar"]["max_rnage_m"] = 5000;
  try {
    parse_scenario(j);
    FAIL() << "a misspelled field should be rejected";
  } catch (const std::runtime_error& e) {
    EXPECT_NE(std::string(e.what()).find("max_rnage_m"), std::string::npos) << e.what();
  }
}

TEST(Scenario, ParameterBlockMustMatchSensorType) {
  Json j = read_json_file(scenario_path("single_drone"));
  j["sensors"].at(std::size_t{0})["camera"] = Json::object();  // sensor 0 is a radar
  EXPECT_THROW(parse_scenario(j), std::runtime_error);
}

#pragma once

#include <nlohmann/json.hpp>

namespace skytrack {

/// JSON type used everywhere. ordered_json keeps fields in the order they were
/// written, so output files read in the same order as the scenario files.
using Json = nlohmann::ordered_json;

}  // namespace skytrack

#include "skytrack/json_fields.hpp"

#include <algorithm>
#include <stdexcept>

namespace skytrack::json {

using geo::deg2rad;

[[noreturn]] void fail(const std::string& ctx, const std::string& what) {
  throw std::runtime_error(ctx + ": " + what);
}

void require(bool ok, const std::string& ctx, const std::string& what) {
  if (!ok) fail(ctx, what);
}

void check_object(const Json& j, const std::string& ctx, std::span<const std::string_view> allowed) {
  if (!j.is_object()) fail(ctx, "expected a JSON object");
  for (auto it = j.begin(); it != j.end(); ++it) {
    if (std::find(allowed.begin(), allowed.end(), std::string_view(it.key())) == allowed.end()) {
      fail(ctx, "unknown field '" + it.key() + "'");
    }
  }
}

void check_object(const Json& j, const std::string& ctx, std::initializer_list<std::string_view> allowed) {
  check_object(j, ctx, std::span<const std::string_view>(allowed.begin(), allowed.size()));
}

double number(const Json& j, const char* key, const std::string& ctx) {
  const auto it = j.find(key);
  if (it == j.end()) fail(ctx, std::string("missing field '") + key + "'");
  if (!it->is_number()) fail(ctx, std::string("field '") + key + "' must be a number");
  return it->get<double>();
}

double number_or(const Json& j, const char* key, double fallback, const std::string& ctx) {
  return j.contains(key) ? number(j, key, ctx) : fallback;
}

double angle_or(const Json& j, const char* key_deg, double fallback_rad, const std::string& ctx) {
  return j.contains(key_deg) ? deg2rad(number(j, key_deg, ctx)) : fallback_rad;
}

double probability_or(const Json& j, const char* key, double fallback, const std::string& ctx) {
  const double p = number_or(j, key, fallback, ctx);
  require(p >= 0.0 && p <= 1.0, ctx, std::string(key) + " must be between 0 and 1");
  return p;
}

double non_negative_or(const Json& j, const char* key, double fallback, const std::string& ctx) {
  const double v = number_or(j, key, fallback, ctx);
  require(v >= 0.0, ctx, std::string(key) + " must not be negative");
  return v;
}

std::int64_t integer(const Json& j, const char* key, const std::string& ctx) {
  const auto it = j.find(key);
  if (it == j.end()) fail(ctx, std::string("missing field '") + key + "'");
  if (!it->is_number_integer()) fail(ctx, std::string("field '") + key + "' must be an integer");
  return it->get<std::int64_t>();
}

std::string text(const Json& j, const char* key, const std::string& ctx) {
  const auto it = j.find(key);
  if (it == j.end()) fail(ctx, std::string("missing field '") + key + "'");
  if (!it->is_string()) fail(ctx, std::string("field '") + key + "' must be a string");
  return it->get<std::string>();
}

std::string text_or(const Json& j, const char* key, const std::string& fallback, const std::string& ctx) {
  return j.contains(key) ? text(j, key, ctx) : fallback;
}

Vec3 vec3_or(const Json& j, const char* key, const Vec3& fallback, const std::string& ctx) {
  const auto it = j.find(key);
  if (it == j.end()) return fallback;
  if (!it->is_array() || it->size() != 3) fail(ctx, std::string("field '") + key + "' must be an array of 3 numbers");
  Vec3 v;
  for (std::size_t i = 0; i < 3; ++i) {
    const Json& element = it->at(i);
    if (!element.is_number()) fail(ctx, std::string("field '") + key + "' must be an array of 3 numbers");
    v[static_cast<Eigen::Index>(i)] = element.get<double>();
  }
  return v;
}

Vec3 vec3(const Json& j, const char* key, const std::string& ctx) {
  if (!j.contains(key)) fail(ctx, std::string("missing field '") + key + "'");
  return vec3_or(j, key, Vec3::Zero(), ctx);
}

const Json& object_or_empty(const Json& j, const char* key) {
  static const Json kEmpty = Json::object();
  const auto it = j.find(key);
  return it == j.end() ? kEmpty : *it;
}

const Json& array_field(const Json& j, const char* key, const std::string& ctx) {
  const auto it = j.find(key);
  if (it == j.end()) fail(ctx, std::string("missing field '") + key + "'");
  if (!it->is_array()) fail(ctx, std::string("field '") + key + "' must be an array");
  return *it;
}

}  // namespace skytrack::json

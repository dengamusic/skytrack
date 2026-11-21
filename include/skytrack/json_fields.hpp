#pragma once

#include <cstdint>
#include <initializer_list>
#include <span>
#include <string>
#include <string_view>

#include "skytrack/geo.hpp"
#include "skytrack/json.hpp"

/// Small, strict readers for scenario and tracker files. `ctx` names the part
/// of the file being read, so every error says where it is. check_object
/// rejects unknown fields: a misspelled field fails loudly instead of
/// silently falling back to a default.
namespace skytrack::json {

using geo::Vec3;

[[noreturn]] void fail(const std::string& ctx, const std::string& what);
void require(bool ok, const std::string& ctx, const std::string& what);
void check_object(const Json& j, const std::string& ctx, std::initializer_list<std::string_view> allowed);
void check_object(const Json& j, const std::string& ctx, std::span<const std::string_view> allowed);

double number(const Json& j, const char* key, const std::string& ctx);
double number_or(const Json& j, const char* key, double fallback, const std::string& ctx);
/// Reads degrees, returns radians.
double angle_or(const Json& j, const char* key_deg, double fallback_rad, const std::string& ctx);
double probability_or(const Json& j, const char* key, double fallback, const std::string& ctx);
double non_negative_or(const Json& j, const char* key, double fallback, const std::string& ctx);
std::int64_t integer(const Json& j, const char* key, const std::string& ctx);
std::string text(const Json& j, const char* key, const std::string& ctx);
std::string text_or(const Json& j, const char* key, const std::string& fallback, const std::string& ctx);
Vec3 vec3(const Json& j, const char* key, const std::string& ctx);
Vec3 vec3_or(const Json& j, const char* key, const Vec3& fallback, const std::string& ctx);
const Json& object_or_empty(const Json& j, const char* key);
const Json& array_field(const Json& j, const char* key, const std::string& ctx);

}  // namespace skytrack::json

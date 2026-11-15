#ifndef SAFETY_PKG__SAFETY_CORE_HPP_
#define SAFETY_PKG__SAFETY_CORE_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace safety_pkg
{
constexpr double kPi = 3.14159265358979323846;
struct SafetyConfig
{
  double range_min = 0.24;
  double range_max = 10.0;
  double sensor_x = 0.0;
  double sensor_y = 0.0;
  double sensor_yaw = 0.0;
  double radius = 0.226;
  double margin = 0.04;
  double reaction_time = 0.2;
  double brake_deceleration = 0.8;
  bool strict_scan_coverage = false;
};

struct FilterResult
{
  double v;
  double w;
  bool intervened;
  std::string reason;
};

inline bool valid_hit(float value, const SafetyConfig & cfg)
{
  return std::isfinite(value) && value >= static_cast<float>(cfg.range_min) &&
         value <= static_cast<float>(cfg.range_max);
}

inline bool no_return(float value)
{
  return std::isinf(value) && value > 0.0F;
}

inline std::vector<float> canonicalize_scan(
  const std::vector<float> & ranges, double angle_min, double angle_increment, int beams)
{
  std::vector<float> result(static_cast<size_t>(beams), std::numeric_limits<float>::quiet_NaN());
  if (ranges.size() < 2 || !std::isfinite(angle_min) || !std::isfinite(angle_increment) || angle_increment <= 0) {
    return result;
  }
  std::vector<double> best(static_cast<size_t>(beams), std::numeric_limits<double>::infinity());
  for (int turn = -1; turn <= 1; ++turn) {
    for (int i = 0; i < beams; ++i) {
      const double target = -kPi + static_cast<double>(i) * 2 * kPi / beams + turn * 2 * kPi;
      const long long index = static_cast<long long>(std::rint((target - angle_min) / angle_increment));
      if (index < 0 || index >= static_cast<long long>(ranges.size())) {continue;}
      const double error = std::fabs(angle_min + index * angle_increment - target);
      if (error <= angle_increment * 0.501 && error < best[static_cast<size_t>(i)]) {
        result[static_cast<size_t>(i)] = ranges[static_cast<size_t>(index)];
        best[static_cast<size_t>(i)] = error;
      }
    }
  }
  return result;
}

inline bool has_valid_return(const std::vector<float> & ranges, const SafetyConfig & cfg)
{
  return std::any_of(ranges.begin(), ranges.end(), [&cfg](float r) {return valid_hit(r, cfg) || no_return(r);});
}

inline FilterResult filter_velocity(
  double requested_v, double requested_w, const std::vector<float> & ranges, const SafetyConfig & cfg)
{
  if (!std::isfinite(requested_v) || !std::isfinite(requested_w)) {
    return {0.0, 0.0, true, "command_invalid"};
  }
  if (std::fabs(requested_v) < 1e-12 && std::fabs(requested_w) < 1e-12) {
    return {0.0, 0.0, false, "none"};
  }
  if (ranges.empty() || !has_valid_return(ranges, cfg)) {
    return {0.0, 0.0, true, "no_valid_scan"};
  }
  double v = requested_v;
  double w = requested_w;
  std::string reason;
  bool any_unknown = false;
  bool swept_unknown = false;
  const double brake_time = cfg.reaction_time + std::fabs(v) / cfg.brake_deceleration;
  const double half_width = std::min(kPi, kPi / 2 + std::fabs(w) * brake_time);
  const double center = v >= 0 ? 0.0 : kPi;
  std::vector<double> points;
  for (size_t i = 0; i < ranges.size(); ++i) {
    const float r = ranges[i];
    const double angle = -kPi + static_cast<double>(i) * 2 * kPi / ranges.size() + cfg.sensor_yaw;
    if (!valid_hit(r, cfg) && !no_return(r)) {
      any_unknown = true;
      const double offset = std::atan2(std::sin(angle - center), std::cos(angle - center));
      if (std::fabs(offset) <= half_width + 1e-9) {swept_unknown = true;}
    }
    if (valid_hit(r, cfg)) {
      points.push_back(r * std::cos(angle) + cfg.sensor_x);
      points.push_back(r * std::sin(angle) + cfg.sensor_y);
    }
  }
  if (cfg.strict_scan_coverage) {
    if (std::fabs(v) > 1e-9 && swept_unknown) {
      reason = v > 0 ? "scan_unknown_front" : "scan_unknown_rear";
      v = 0.0;
    }
    if (std::fabs(w) > 1e-9 && any_unknown) {
      w = 0.0;
      if (reason.empty()) {reason = "scan_unknown_rotation";}
    }
  }
  const double limit = cfg.radius + cfg.margin;
  for (size_t k = 0; k + 1 < points.size(); k += 2) {
    if (std::hypot(points[k], points[k + 1]) < limit) {
      return {0.0, 0.0, true, cfg.strict_scan_coverage ? "footprint_overlap" : "clearance"};
    }
  }
  if (std::fabs(v) > 1e-9) {
    const double stopping = std::fabs(v) * cfg.reaction_time + v * v / (2 * cfg.brake_deceleration);
    const double duration = stopping / std::max(std::fabs(v), 1e-8);
    for (int step = 0; step < 7; ++step) {
      const double time = duration * (0.15 + step * (0.85 / 6.0));
      const double x = std::fabs(w) < 1e-8 ? v * time : v / w * std::sin(w * time);
      const double y = std::fabs(w) < 1e-8 ? 0.0 : v / w * (1 - std::cos(w * time));
      for (size_t k = 0; k + 1 < points.size(); k += 2) {
        if (std::hypot(points[k] - x, points[k + 1] - y) < limit) {
          v = 0.0;
          if (reason.empty()) {reason = cfg.strict_scan_coverage ? "braking_sweep" : "braking";}
          break;
        }
      }
      if (v == 0.0) {break;}
    }
  }
  const bool changed = !reason.empty() || v != requested_v || w != requested_w;
  return {v, w, changed, reason.empty() ? "none" : reason};
}
}  // namespace safety_pkg
#endif  // SAFETY_PKG__SAFETY_CORE_HPP_

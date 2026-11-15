#include <cassert>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <limits>
#include <string>
#include <vector>
#include "safety_pkg/safety_core.hpp"

// Optional portable parity driver. Each input record is whitespace separated:
// v w min max sensor_x sensor_y yaw radius margin reaction deceleration strict N ranges...
// nan/inf are accepted. Output: v w intervened reason, one record per line.
int check_stdin()
{
  std::string token;
  const auto number = []() {
      std::string value;
      if (!(std::cin >> value)) {throw std::runtime_error("truncated parity record");}
      return std::stod(value);
    };
  while (std::cin >> token) {
    const double v = std::stod(token);
    const double w = number();
    safety_pkg::SafetyConfig cfg;
    cfg.range_min = number(); cfg.range_max = number();
    cfg.sensor_x = number(); cfg.sensor_y = number(); cfg.sensor_yaw = number();
    cfg.radius = number(); cfg.margin = number(); cfg.reaction_time = number();
    cfg.brake_deceleration = number(); cfg.strict_scan_coverage = number() != 0;
    const auto count = static_cast<size_t>(number());
    if (count > 4096) {throw std::runtime_error("too many parity rays");}
    std::vector<float> ranges(count);
    for (float & range : ranges) {range = static_cast<float>(number());}
    const auto result = safety_pkg::filter_velocity(v, w, ranges, cfg);
    std::cout << std::setprecision(17) << result.v << '\t' << result.w << '\t'
              << result.intervened << '\t' << result.reason << '\n';
  }
  return 0;
}

int main(int argc, char ** argv)
{
  if (argc == 2 && std::string(argv[1]) == "--stdin") {
    try {return check_stdin();} catch (const std::exception & ex) {
      std::cerr << ex.what() << '\n'; return 2;
    }
  }
  using safety_pkg::filter_velocity;
  const float inf = std::numeric_limits<float>::infinity();
  const float nan = std::numeric_limits<float>::quiet_NaN();
  safety_pkg::SafetyConfig cfg;
  cfg.strict_scan_coverage = true;
  std::vector<float> scan(360, inf);
  auto result = filter_velocity(0.26, 0.0, scan, cfg);
  assert(!result.intervened && result.v == 0.26);
  scan[180] = nan;
  result = filter_velocity(0.26, 0.0, scan, cfg);
  assert(result.v == 0.0 && result.reason == "scan_unknown_front");
  scan.assign(360, inf);
  scan[0] = nan;
  result = filter_velocity(0.26, 0.0, scan, cfg);
  assert(!result.intervened && result.v == 0.26);
  result = filter_velocity(-0.26, 0.0, scan, cfg);
  assert(result.v == 0.0 && result.reason == "scan_unknown_rear");
  result = filter_velocity(0.0, 0.5, scan, cfg);
  assert(result.w == 0.0 && result.reason == "scan_unknown_rotation");
  scan.assign(360, inf);
  scan[180] = 10.1F;
  result = filter_velocity(0.26, 0.0, scan, cfg);
  assert(result.v == 0.0 && result.reason == "scan_unknown_front");
  scan.assign(360, nan);
  result = filter_velocity(0.26, 0.5, scan, cfg);
  assert(result.v == 0.0 && result.w == 0.0 && result.reason == "no_valid_scan");
  scan.assign(360, inf);
  scan[180] = 0.25F;
  result = filter_velocity(0.1, 0.0, scan, cfg);
  assert(result.v == 0.0 && result.reason == "footprint_overlap");
  result = filter_velocity(nan, 0.0, scan, cfg);
  assert(result.v == 0.0 && result.reason == "command_invalid");
  // A 180-degree scanner leaves the back half unknown rather than inventing rays.
  const auto canonical = safety_pkg::canonicalize_scan(
    std::vector<float>(181, inf), -safety_pkg::kPi / 2, safety_pkg::kPi / 180, 360);
  assert(std::isnan(canonical[0]) && std::isinf(canonical[180]));
  result = filter_velocity(0.26, 0.0, canonical, cfg);
  assert(result.v == 0.26);
  result = filter_velocity(-0.26, 0.0, canonical, cfg);
  assert(result.v == 0.0);
  // A legacy policy keeps the previous permissive unknown-ray behavior.
  cfg.strict_scan_coverage = false;
  scan.assign(360, inf); scan[180] = nan;
  result = filter_velocity(0.26, 0.0, scan, cfg);
  assert(result.v == 0.26);
  std::cout << "PASS: safety core v1/v2, directional coverage, metadata angles and nonfinite commands\n";
  return 0;
}

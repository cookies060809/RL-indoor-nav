#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "bev_pkg/bev_core.hpp"

namespace fs = std::filesystem;

namespace
{
template<typename T>
bool read_value(std::ifstream & in, T & value)
{
  in.read(reinterpret_cast<char *>(&value), sizeof(T));
  return static_cast<bool>(in);
}

float max_abs_diff(
  const std::vector<float> & a, const std::vector<float> & b,
  size_t a_begin, size_t b_begin, size_t count)
{
  float worst = 0.0F;
  for (size_t i = 0; i < count; ++i) {
    const float d = std::fabs(a[a_begin + i] - b[b_begin + i]);
    if (d > worst) {
      worst = d;
    }
  }
  return worst;
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::cerr << "usage: bev_fixture_check <fixture-dir>\n";
    return 2;
  }
  int cases = 0;
  int failures = 0;
  for (const auto & entry : fs::directory_iterator(argv[1])) {
    if (entry.path().extension() != ".bin") {
      continue;
    }
    std::ifstream in(entry.path(), std::ios::binary);
    if (!in) {
      std::cerr << "cannot open " << entry.path() << "\n";
      return 2;
    }
    double header[11] = {0};
    in.read(reinterpret_cast<char *>(header), sizeof(header));
    int32_t count = 0;
    int32_t expected_count = 0;
    if (!read_value(in, count) || count <= 0) {
      std::cerr << "bad range count in " << entry.path() << "\n";
      return 2;
    }
    std::vector<float> ranges(static_cast<size_t>(count));
    in.read(reinterpret_cast<char *>(ranges.data()), static_cast<std::streamsize>(count * sizeof(float)));
    if (!read_value(in, expected_count) || expected_count <= 0) {
      std::cerr << "bad expected count in " << entry.path() << "\n";
      return 2;
    }
    std::vector<float> expected(static_cast<size_t>(expected_count));
    in.read(
      reinterpret_cast<char *>(expected.data()),
      static_cast<std::streamsize>(expected_count * sizeof(float)));
    if (!in) {
      std::cerr << "truncated fixture " << entry.path() << "\n";
      return 2;
    }

    bev_pkg::BevConfig cfg;
    cfg.n = static_cast<int>(header[0]);
    cfg.extent = header[1];
    cfg.range_min = header[2];
    cfg.range_max = header[3];
    cfg.sensor_x = header[4];
    cfg.sensor_y = header[5];
    cfg.sensor_yaw = header[6];
    cfg.robot_radius = header[7];
    cfg.distance_clip = header[8];
    const float goal_x = static_cast<float>(header[9]);
    const float goal_y = static_cast<float>(header[10]);

    std::vector<float> occupancy;
    std::vector<float> distance;
    bev_pkg::build_bev(ranges, cfg, occupancy, distance);
    const auto goal = bev_pkg::goal_features(goal_x, goal_y, cfg);

    const size_t cells = static_cast<size_t>(cfg.n) * cfg.n;
    if (expected.size() != 2 * cells + 8) {
      std::cerr << "unexpected observation size in " << entry.path() << "\n";
      ++failures;
      continue;
    }
    // Expected layout: occupancy, distance, goal, then motion scalars.
    const float occ_diff = max_abs_diff(occupancy, expected, 0, 0, cells);
    const float dist_diff = max_abs_diff(distance, expected, 0, cells, cells);
    float goal_diff = 0.0F;
    for (size_t i = 0; i < 4; ++i) {
      goal_diff = std::max(goal_diff, std::fabs(goal[i] - expected[2 * cells + i]));
    }
    const bool ok = occ_diff <= 1e-6F && dist_diff <= 1e-5F && goal_diff <= 1e-5F;
    std::cout << (ok ? "PASS " : "FAIL ") << entry.path().stem().string()
              << " occ=" << occ_diff << " dist=" << dist_diff << " goal=" << goal_diff << "\n";
    if (!ok) {
      size_t worst_idx = 0;
      float worst = -1.0F;
      for (size_t i = 0; i < cells; ++i) {
        const float d = std::fabs(distance[i] - expected[cells + i]);
        if (d > worst) {
          worst = d;
          worst_idx = i;
        }
      }
      std::cout << "  dist worst cell " << worst_idx
                << " ours=" << distance[worst_idx]
                << " expected=" << expected[cells + worst_idx] << "\n";
      for (size_t i = 0; i < cells; ++i) {
        if (std::fabs(occupancy[i] - expected[i]) > 1e-6F) {
          std::cout << "  occ mismatch cell " << i << " ours=" << occupancy[i]
                    << " expected=" << expected[i] << "\n";
          break;
        }
      }
    }
    ++cases;
    if (!ok) {
      ++failures;
    }
  }
  std::cout << cases << " fixtures, " << failures << " failures\n";
  return failures == 0 && cases > 0 ? 0 : 1;
}

#include "bev_pkg/bev_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace bev_pkg
{
namespace
{
constexpr double kInf = 1e9;

// Exact 1D squared Euclidean distance transform (Felzenszwalb & Huttenlocher).
void edt_1d(const std::vector<double> & f, std::vector<double> & d)
{
  const int n = static_cast<int>(f.size());
  std::vector<int> v(static_cast<size_t>(n));
  std::vector<double> z(static_cast<size_t>(n) + 1);
  int k = 0;
  v[0] = 0;
  z[0] = -1e30;
  z[1] = 1e30;
  for (int q = 1; q < n; ++q) {
    double s = ((f[q] + static_cast<double>(q) * q) -
      (f[v[k]] + static_cast<double>(v[k]) * v[k])) /
      (2.0 * q - 2.0 * v[k]);
    while (s <= z[k]) {
      --k;
      s = ((f[q] + static_cast<double>(q) * q) -
        (f[v[k]] + static_cast<double>(v[k]) * v[k])) /
        (2.0 * q - 2.0 * v[k]);
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = 1e30;
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < q) {
      ++k;
    }
    const double dq = q - v[k];
    d[q] = dq * dq + f[v[k]];
  }
}

void distance_transform_2d(const std::vector<uint8_t> & free, int n, std::vector<double> & out)
{
  std::vector<double> grid(static_cast<size_t>(n) * n);
  for (size_t i = 0; i < grid.size(); ++i) {
    grid[i] = free[i] ? kInf : 0.0;
  }
  std::vector<double> line(static_cast<size_t>(n));
  std::vector<double> result(static_cast<size_t>(n));
  for (int y = 0; y < n; ++y) {
    for (int x = 0; x < n; ++x) {
      line[x] = grid[static_cast<size_t>(y) * n + x];
    }
    edt_1d(line, result);
    for (int x = 0; x < n; ++x) {
      grid[static_cast<size_t>(y) * n + x] = result[x];
    }
  }
  for (int x = 0; x < n; ++x) {
    for (int y = 0; y < n; ++y) {
      line[y] = grid[static_cast<size_t>(y) * n + x];
    }
    edt_1d(line, result);
    for (int y = 0; y < n; ++y) {
      grid[static_cast<size_t>(y) * n + x] = result[y];
    }
  }
  out.resize(grid.size());
  for (size_t i = 0; i < grid.size(); ++i) {
    out[i] = std::sqrt(grid[i]);
  }
}
}  // namespace

std::vector<float> canonicalize_scan(
  const std::vector<float> & ranges, float angle_min, float angle_increment, int beams)
{
  if (ranges.size() < 2 || angle_increment <= 0.0F) {
    return {};
  }
  std::vector<float> output(static_cast<size_t>(beams),
    std::numeric_limits<float>::quiet_NaN());
  std::vector<double> best(static_cast<size_t>(beams), std::numeric_limits<double>::infinity());
  constexpr double kPi = 3.14159265358979323846;
  const int count = static_cast<int>(ranges.size());
  for (int turn_index = -1; turn_index <= 1; ++turn_index) {
    const double turn = static_cast<double>(turn_index) * 2.0 * kPi;
    for (int i = 0; i < beams; ++i) {
      const double target = -kPi + static_cast<double>(i) * 2.0 * kPi / beams;
      const double raw = std::rint((target + turn - angle_min) / angle_increment);
      const long long idx = static_cast<long long>(raw);
      if (idx < 0 || idx >= count) {
        continue;
      }
      const double error =
        std::fabs(static_cast<double>(angle_min) + static_cast<double>(idx) * angle_increment -
        (target + turn));
      if (error <= static_cast<double>(angle_increment) * 0.501 && error < best[i]) {
        output[i] = ranges[static_cast<size_t>(idx)];
        best[i] = error;
      }
    }
  }
  return output;
}

void build_bev(
  const std::vector<float> & ranges, const BevConfig & cfg,
  std::vector<float> & occupancy, std::vector<float> & distance)
{
  const int n = cfg.n;
  const double res = cfg.extent / static_cast<double>(n);
  const double half = cfg.extent / 2.0;
  const int count = static_cast<int>(ranges.size());
  std::vector<float> grid(static_cast<size_t>(n) * n, 0.5F);
  std::vector<uint8_t> hits(static_cast<size_t>(n) * n, 0);

  const double step = 2.0 * 3.14159265358979323846 / std::max(1, count);
  for (int i = 0; i < count; ++i) {
    const float value = ranges[static_cast<size_t>(i)];
    if (std::isnan(value) || static_cast<double>(value) < cfg.range_min ||
      value == -std::numeric_limits<float>::infinity())
    {
      continue;
    }
    const bool hit = std::isfinite(value) && value <= cfg.range_max;
    const double length = hit ? std::min(static_cast<double>(value), cfg.range_max) : cfg.range_max;
    const double angle = cfg.sensor_yaw - 3.14159265358979323846 +
      static_cast<double>(i) * step;
    const double dx = std::cos(angle);
    const double dy = std::sin(angle);
    double near = 0.0;
    double far = length;
    for (int k = 0; k < 2; ++k) {
      const double origin = k == 0 ? cfg.sensor_x : cfg.sensor_y;
      const double direction = k == 0 ? dx : dy;
      if (std::fabs(direction) < 1e-12) {
        if (origin < -half || origin >= half) {
          far = -1.0;
        }
      } else {
        const double a = (-half - origin) / direction;
        const double b = (half - origin) / direction;
        near = std::max(near, std::min(a, b));
        far = std::min(far, std::max(a, b));
      }
    }
    if (far < near) {
      continue;
    }
    int x0 = std::clamp(
      static_cast<int>(std::floor((cfg.sensor_x + near * dx + half) / res)), 0, n - 1);
    int y0 = std::clamp(
      static_cast<int>(std::floor((cfg.sensor_y + near * dy + half) / res)), 0, n - 1);
    int x1 = std::clamp(
      static_cast<int>(std::floor((cfg.sensor_x + far * dx + half) / res)), 0, n - 1);
    int y1 = std::clamp(
      static_cast<int>(std::floor((cfg.sensor_y + far * dy + half) / res)), 0, n - 1);
    int ddx = std::abs(x1 - x0);
    const int ddy = -std::abs(y1 - y0);
    const int stepx = x0 < x1 ? 1 : -1;
    const int stepy = y0 < y1 ? 1 : -1;
    int err = ddx + ddy;
    while (true) {
      grid[static_cast<size_t>(y0) * n + x0] = 0.0F;
      if (x0 == x1 && y0 == y1) {
        break;
      }
      const int twice = 2 * err;
      if (twice >= ddy) {
        err += ddy;
        x0 += stepx;
      }
      if (twice <= ddx) {
        err += ddx;
        y0 += stepy;
      }
    }
    const double ex = cfg.sensor_x + length * dx;
    const double ey = cfg.sensor_y + length * dy;
    if (hit && -half <= ex && ex < half && -half <= ey && ey < half) {
      const int col = static_cast<int>(std::floor((ex + half) / res));
      const int row = static_cast<int>(std::floor((ey + half) / res));
      hits[static_cast<size_t>(row) * n + col] = 1;
    }
  }

  for (int row = 0; row < n; ++row) {
    for (int col = 0; col < n; ++col) {
      const double x = (static_cast<double>(col) + 0.5) * res - half;
      const double y = (static_cast<double>(row) + 0.5) * res - half;
      const size_t idx = static_cast<size_t>(row) * n + col;
      if (x * x + y * y < cfg.robot_radius * cfg.robot_radius && hits[idx] == 0) {
        grid[idx] = 0.0F;
      }
      if (hits[idx] != 0) {
        grid[idx] = 1.0F;
      }
    }
  }

  occupancy = grid;

  std::vector<uint8_t> free(static_cast<size_t>(n) * n, 0);
  size_t free_count = 0;
  for (size_t i = 0; i < grid.size(); ++i) {
    if (grid[i] == 0.0F) {
      free[i] = 1;
      ++free_count;
    }
  }
  distance.assign(grid.size(), 1.0F);
  if (free_count == grid.size()) {
    return;
  }
  std::vector<double> edt;
  distance_transform_2d(free, n, edt);
  const double scale = res / cfg.distance_clip;
  for (size_t i = 0; i < edt.size(); ++i) {
    distance[i] = static_cast<float>(std::min(edt[i] * scale, 1.0));
  }
}

std::vector<float> goal_features(float dx, float dy, const BevConfig & cfg)
{
  std::vector<float> out(4, 0.0F);
  const double d = std::hypot(static_cast<double>(dx), static_cast<double>(dy));
  if (d < 1e-8) {
    return out;
  }
  out[0] = static_cast<float>(dx / d);
  out[1] = static_cast<float>(dy / d);
  out[2] = static_cast<float>(std::min(d / cfg.goal_distance_scale, 1.0));
  out[3] = static_cast<float>(std::atan2(dy, dx) / 3.14159265358979323846);
  return out;
}

}  // namespace bev_pkg

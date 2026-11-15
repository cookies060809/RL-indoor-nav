#ifndef BEV_PKG__BEV_CORE_HPP_
#define BEV_PKG__BEV_CORE_HPP_

#include <vector>

namespace bev_pkg
{

struct BevConfig
{
  int n = 32;
  double extent = 4.8;
  double range_min = 0.24;
  double range_max = 10.0;
  double sensor_x = 0.0;
  double sensor_y = 0.0;
  double sensor_yaw = 0.0;
  double robot_radius = 0.226;
  double distance_clip = 1.0;
  double goal_distance_scale = 5.0;
  // v1 accepted finite values above range_max as no-return rays. v2 must not.
  bool strict_scan = false;
};

std::vector<float> canonicalize_scan(
  const std::vector<float> & ranges, float angle_min, float angle_increment, int beams);

void build_bev(
  const std::vector<float> & ranges, const BevConfig & cfg,
  std::vector<float> & occupancy, std::vector<float> & distance);

std::vector<float> goal_features(float dx, float dy, const BevConfig & cfg);

}  // namespace bev_pkg

#endif  // BEV_PKG__BEV_CORE_HPP_

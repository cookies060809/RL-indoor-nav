#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <stdexcept>
#include <vector>

#include "bev_pkg/bev_core.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "project_msgs/msg/policy_observation.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;
}

class BevNode : public rclcpp::Node
{
public:
  BevNode()
  : Node("bev_node"), buffer_(this->get_clock()), listener_(buffer_)
  {
    cfg_.n = declare_parameter<int>("n", 32);
    cfg_.extent = declare_parameter<double>("extent", 4.8);
    cfg_.robot_radius = declare_parameter<double>("robot_radius", 0.226);
    cfg_.distance_clip = declare_parameter<double>("distance_clip", 1.0);
    cfg_.goal_distance_scale = declare_parameter<double>("goal_distance_scale", 5.0);
    spec_version_ = declare_parameter<std::string>("spec_version", "bev_nav_v1");
    if (spec_version_ != "bev_nav_v1" && spec_version_ != "bev_nav_v2") {
      throw std::invalid_argument("Unsupported BEV spec_version: " + spec_version_);
    }
    cfg_.strict_scan = spec_version_ == "bev_nav_v2";
    beams_ = declare_parameter<int>("beams", 360);
    goal_frame_ = declare_parameter<std::string>("goal_frame", "odom");
    goal_x_ = declare_parameter<double>("goal_x", 0.0);
    goal_y_ = declare_parameter<double>("goal_y", 0.0);
    v_max_ = declare_parameter<double>("v_max", 0.26);
    w_max_ = declare_parameter<double>("w_max", 1.0);
    const double range_min = declare_parameter<double>("range_min", 0.24);
    const double range_max = declare_parameter<double>("range_max", 10.0);
    publish_grid_ = declare_parameter<bool>("publish_grid", true);
    require_local_goal_ = declare_parameter<bool>("require_local_goal", false);
    local_goal_timeout_ = declare_parameter<double>("local_goal_timeout", 1.0);
    require_fresh_feedback_ = declare_parameter<bool>("require_fresh_feedback", cfg_.strict_scan);
    feedback_timeout_ = declare_parameter<double>("feedback_timeout", 0.5);

    rclcpp::QoS reliable(10);
    pub_observation_ = create_publisher<project_msgs::msg::PolicyObservation>(
      "/policy_observation", reliable);
    pub_grid_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/bev_grid", reliable);

    sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {onScan(msg);});
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", reliable,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_v_ = msg->twist.twist.linear.x;
        odom_w_ = msg->twist.twist.angular.z;
        odom_stamp_ = rclcpp::Time(msg->header.stamp);
        odom_received_ = std::chrono::steady_clock::now();
        have_odom_ = true;
      });
    sub_cmd_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", reliable,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        prev_v_ = msg->linear.x;
        prev_w_ = msg->angular.z;
        command_received_ = std::chrono::steady_clock::now();
        have_command_ = true;
      });
    sub_goal_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/local_goal", reliable,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        local_goal_ = *msg;
        have_local_goal_ = true;
        goal_received_ = std::chrono::steady_clock::now();
      });

    timer_log_ = create_wall_timer(
      std::chrono::seconds(2), [this]() {
        if (frames_ > 0) {
          RCLCPP_INFO(
            get_logger(), "BEV %dx%d, %zu frames, avg %.1f ms, goal (%.2f, %.2f) %s",
            cfg_.n, cfg_.n, frames_, total_ms_ / static_cast<double>(frames_),
            goal_dx_, goal_dy_, active_goal_frame_.c_str());
          frames_ = 0;
          total_ms_ = 0.0;
        }
      });

    if (range_min > 0.0) {
      cfg_.range_min = range_min;
    }
    if (range_max > 0.0) {
      cfg_.range_max = range_max;
    }
    RCLCPP_INFO(
      get_logger(), "bev_node ready: n=%d extent=%.2f beams=%d spec=%s",
      cfg_.n, cfg_.extent, beams_, spec_version_.c_str());
  }

private:
  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    const auto t0 = std::chrono::steady_clock::now();
    const auto age = [t0](const std::chrono::steady_clock::time_point & received) {
        return std::chrono::duration<double>(t0 - received).count();
      };
    if (require_local_goal_ &&
      (!have_local_goal_ || age(goal_received_) > local_goal_timeout_))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "no fresh local goal; withholding observation");
      return;
    }
    if (require_fresh_feedback_ &&
      (!have_odom_ || !have_command_ || age(odom_received_) > feedback_timeout_ ||
      age(command_received_) > feedback_timeout_))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "motion feedback stale; withholding observation");
      return;
    }
    std::vector<float> canonical = bev_pkg::canonicalize_scan(
      msg->ranges, msg->angle_min, msg->angle_increment, beams_);
    if (canonical.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "scan too short or invalid");
      return;
    }

    bev_pkg::BevConfig cfg = cfg_;
    if (cfg.strict_scan) {
      if (!std::isfinite(msg->range_min) || !std::isfinite(msg->range_max) ||
        msg->range_min <= 0 || msg->range_max <= msg->range_min)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "invalid scan range metadata");
        return;
      }
      cfg.range_min = std::max(cfg.range_min, static_cast<double>(msg->range_min));
      cfg.range_max = std::min(cfg.range_max, static_cast<double>(msg->range_max));
      if (cfg.range_max <= cfg.range_min) {
        return;
      }
    }

    const rclcpp::Time stamp(msg->header.stamp);
    geometry_msgs::msg::TransformStamped sensor_tf;
    try {
      sensor_tf = buffer_.lookupTransform(
        "base_footprint", msg->header.frame_id, stamp, rclcpp::Duration::from_seconds(0.05));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "sensor TF unavailable: %s", ex.what());
      return;
    }
    cfg.sensor_x = sensor_tf.transform.translation.x;
    cfg.sensor_y = sensor_tf.transform.translation.y;
    cfg.sensor_yaw = 2.0 * std::atan2(
      sensor_tf.transform.rotation.z, sensor_tf.transform.rotation.w);

    double gx = goal_x_;
    double gy = goal_y_;
    std::string frame = goal_frame_;
    if (have_local_goal_ && (!require_local_goal_ || age(goal_received_) <= local_goal_timeout_)) {
      gx = local_goal_.pose.position.x;
      gy = local_goal_.pose.position.y;
      frame = local_goal_.header.frame_id;
    }
    if (frame.empty()) {
      frame = "base_footprint";
    }
    double dx = gx;
    double dy = gy;
    if (frame != "base_footprint") {
      try {
        const auto goal_tf = buffer_.lookupTransform(
          "base_footprint", frame, stamp, rclcpp::Duration::from_seconds(0.05));
        geometry_msgs::msg::PointStamped in;
        geometry_msgs::msg::PointStamped out;
        in.header.frame_id = frame;
        in.header.stamp = msg->header.stamp;
        in.point.x = gx;
        in.point.y = gy;
        tf2::doTransform(in, out, goal_tf);
        dx = out.point.x;
        dy = out.point.y;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "goal TF %s failed: %s", frame.c_str(), ex.what());
        return;
      }
    }

    double v = 0.0;
    double w = 0.0;
    if (have_odom_) {
      const double age = (stamp - odom_stamp_).seconds();
      if (std::fabs(age) <= 0.2) {
        v = odom_v_;
        w = odom_w_;
      } else if (require_fresh_feedback_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "odometry stamp not aligned to scan");
        return;
      }
    }

    std::vector<float> occupancy;
    std::vector<float> distance;
    bev_pkg::build_bev(canonical, cfg, occupancy, distance);

    project_msgs::msg::PolicyObservation obs;
    obs.header.stamp = msg->header.stamp;
    obs.header.frame_id = "base_footprint";
    obs.spec_version = spec_version_;
    obs.height = static_cast<uint16_t>(cfg.n);
    obs.width = static_cast<uint16_t>(cfg.n);
    obs.resolution = static_cast<float>(cfg.extent / static_cast<double>(cfg.n));
    obs.bev.reserve(occupancy.size() + distance.size());
    obs.bev.insert(obs.bev.end(), occupancy.begin(), occupancy.end());
    obs.bev.insert(obs.bev.end(), distance.begin(), distance.end());
    const auto goal = bev_pkg::goal_features(static_cast<float>(dx), static_cast<float>(dy), cfg);
    for (size_t i = 0; i < 4; ++i) {
      obs.goal[i] = goal[i];
    }
    obs.motion[0] = static_cast<float>(v / v_max_);
    obs.motion[1] = static_cast<float>(w / w_max_);
    obs.motion[2] = static_cast<float>(prev_v_ / v_max_);
    obs.motion[3] = static_cast<float>(prev_w_ / w_max_);
    pub_observation_->publish(obs);

    if (publish_grid_) {
      nav_msgs::msg::OccupancyGrid grid;
      grid.header = obs.header;
      grid.info.resolution = obs.resolution;
      grid.info.width = static_cast<uint32_t>(cfg.n);
      grid.info.height = static_cast<uint32_t>(cfg.n);
      grid.info.origin.position.x = -cfg.extent / 2.0;
      grid.info.origin.position.y = -cfg.extent / 2.0;
      grid.info.origin.orientation.w = 1.0;
      grid.data.resize(occupancy.size());
      for (size_t i = 0; i < occupancy.size(); ++i) {
        const float value = occupancy[i];
        grid.data[i] = value > 0.75F ? 100 : (value < 0.25F ? 0 : -1);
      }
      pub_grid_->publish(grid);
    }

    goal_dx_ = dx;
    goal_dy_ = dy;
    active_goal_frame_ = frame;
    ++frames_;
    total_ms_ += std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();
  }

  bev_pkg::BevConfig cfg_;
  std::string spec_version_;
  std::string goal_frame_;
  std::string active_goal_frame_;
  int beams_ = 360;
  double goal_x_ = 0.0;
  double goal_y_ = 0.0;
  double v_max_ = 0.26;
  double w_max_ = 1.0;
  bool publish_grid_ = true;
  double goal_dx_ = 0.0;
  double goal_dy_ = 0.0;

  bool have_odom_ = false;
  double odom_v_ = 0.0;
  double odom_w_ = 0.0;
  rclcpp::Time odom_stamp_;
  double prev_v_ = 0.0;
  double prev_w_ = 0.0;

  bool have_local_goal_ = false;
  bool require_local_goal_ = false;
  bool require_fresh_feedback_ = false;
  bool have_command_ = false;
  double local_goal_timeout_ = 1.0;
  double feedback_timeout_ = 0.5;
  std::chrono::steady_clock::time_point goal_received_;
  std::chrono::steady_clock::time_point odom_received_;
  std::chrono::steady_clock::time_point command_received_;
  geometry_msgs::msg::PoseStamped local_goal_;

  size_t frames_ = 0;
  double total_ms_ = 0.0;

  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  rclcpp::Publisher<project_msgs::msg::PolicyObservation>::SharedPtr pub_observation_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_grid_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
  rclcpp::TimerBase::SharedPtr timer_log_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BevNode>());
  rclcpp::shutdown();
  return 0;
}

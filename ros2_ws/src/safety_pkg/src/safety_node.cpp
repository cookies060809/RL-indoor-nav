#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <sstream>
#include <string>
#include <stdexcept>
#include <vector>
#include "safety_pkg/safety_core.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class SafetyNode : public rclcpp::Node
{
public:
  SafetyNode()
  : Node("safety_node"), buffer_(this->get_clock()), listener_(buffer_)
  {
    candidate_topic_ = declare_parameter<std::string>("candidate_topic", "/cmd_vel_rl");
    v_max_ = declare_parameter<double>("v_max", 0.26);
    w_max_ = declare_parameter<double>("w_max", 1.0);
    accel_ = declare_parameter<double>("acceleration", 0.5);
    alpha_ = declare_parameter<double>("angular_acceleration", 2.0);
    control_dt_ = declare_parameter<double>("control_dt", 0.05);
    stale_timeout_ = declare_parameter<double>("stale_timeout", 0.5);
    sensor_timeout_ = declare_parameter<double>("sensor_timeout", 0.5);
    radius_ = declare_parameter<double>("robot_radius", 0.226);
    margin_ = declare_parameter<double>("safety_margin", 0.04);
    reaction_time_ = declare_parameter<double>("reaction_time", 0.2);
    brake_deceleration_ = declare_parameter<double>("brake_deceleration", 0.8);
    strict_scan_coverage_ = declare_parameter<bool>("strict_scan_coverage", false);
    canonical_beams_ = declare_parameter<int>("canonical_beams", 360);
    sensor_future_tolerance_ = declare_parameter<double>("sensor_future_tolerance", 0.15);
    configured_range_min_ = declare_parameter<double>("range_min", 0.24);
    configured_range_max_ = declare_parameter<double>("range_max", 10.0);
    if (canonical_beams_ < 2 || canonical_beams_ > 4096 || v_max_ <= 0 ||
      w_max_ <= 0 || accel_ <= 0 || alpha_ <= 0 || control_dt_ <= 0 ||
      brake_deceleration_ <= 0 || stale_timeout_ <= 0 || sensor_timeout_ <= 0)
    {
      throw std::invalid_argument("Invalid safety dimensions, limits or timing parameters");
    }

    pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    pub_estop_ = create_publisher<std_msgs::msg::Bool>("/estop_state", 10);
    pub_status_ = create_publisher<std_msgs::msg::String>("/safety/status", 10);

    sub_candidate_ = create_subscription<geometry_msgs::msg::Twist>(
      candidate_topic_, 10,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        cand_v_ = msg->linear.x;
        cand_w_ = msg->angular.z;
        last_candidate_ = std::chrono::steady_clock::now();
        candidate_valid_ = std::isfinite(cand_v_) && std::isfinite(cand_w_);
        if (!candidate_valid_) {
          cand_v_ = 0.0;
          cand_w_ = 0.0;
        }
        have_candidate_ = true;
      });
    sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {onScan(msg);});
    sub_estop_ = create_subscription<std_msgs::msg::Bool>(
      "/estop_request", 10,
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
          estop_ = true;
          estop_request_ = true;
        } else {
          estop_request_ = false;
        }
      });
    srv_reset_ = create_service<std_srvs::srv::Trigger>(
      "/safety/reset",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        if (!estop_) {
          response->success = false;
          response->message = "estop is not latched";
          return;
        }
        const double scan_age = have_scan_ ?
          std::chrono::duration<double>(std::chrono::steady_clock::now() - scan_received_).count() : 1e9;
        const double stamp_age = have_scan_ ? (this->now() - scan_stamp_).seconds() : 1e9;
        if (estop_request_ || scan_age > sensor_timeout_ || !points_valid_ ||
          stamp_age > sensor_timeout_ || stamp_age < -sensor_future_tolerance_)
        {
          response->success = false;
          response->message = "conditions not safe, keep stopped";
          return;
        }
        estop_ = false;
        response->success = true;
        response->message = "estop cleared";
        RCLCPP_WARN(get_logger(), "estop cleared by /safety/reset");
      });

    timer_ = create_wall_timer(
      std::chrono::milliseconds(50), [this]() {tick();});

    RCLCPP_INFO(
      get_logger(), "safety_node ready: candidate=%s v_max=%.2f w_max=%.2f",
      candidate_topic_.c_str(), v_max_, w_max_);
  }

private:
  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    const rclcpp::Time stamp(msg->header.stamp);
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = buffer_.lookupTransform(
        "base_footprint", msg->header.frame_id, stamp, rclcpp::Duration::from_seconds(0.05));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "sensor TF unavailable: %s", ex.what());
      points_valid_ = false;
      have_scan_ = false;
      return;
    }
    scan_config_.sensor_x = tf.transform.translation.x;
    scan_config_.sensor_y = tf.transform.translation.y;
    scan_config_.sensor_yaw = 2.0 * std::atan2(tf.transform.rotation.z, tf.transform.rotation.w);
    // Respect both the sensor's advertised limits and the trained model's
    // limits. A shorter-range sensor must never clear space to the model max.
    scan_config_.range_min = std::max(configured_range_min_, static_cast<double>(msg->range_min));
    scan_config_.range_max = std::min(configured_range_max_, static_cast<double>(msg->range_max));
    scan_config_.radius = radius_;
    scan_config_.margin = margin_;
    scan_config_.reaction_time = reaction_time_;
    scan_config_.brake_deceleration = brake_deceleration_;
    scan_config_.strict_scan_coverage = strict_scan_coverage_;
    if (!std::isfinite(msg->range_min) || !std::isfinite(msg->range_max) ||
      msg->range_min <= 0 || msg->range_max <= msg->range_min ||
      scan_config_.range_max <= scan_config_.range_min)
    {
      have_scan_ = false;
      points_valid_ = false;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "invalid or nonoverlapping scan limits");
      return;
    }
    ranges_ = safety_pkg::canonicalize_scan(
      msg->ranges, msg->angle_min, msg->angle_increment, canonical_beams_);
    points_valid_ = safety_pkg::has_valid_return(ranges_, scan_config_);
    scan_stamp_ = stamp;
    scan_received_ = std::chrono::steady_clock::now();
    have_scan_ = true;
  }

  void tick()
  {
    const rclcpp::Time now = this->now();
    const auto steady_now = std::chrono::steady_clock::now();
    if (have_ros_time_ && now.nanoseconds() < last_ros_time_) {
      have_candidate_ = false;
      have_scan_ = false;
      points_valid_ = false;
      prev_v_ = 0.0;
      prev_w_ = 0.0;
    }
    have_ros_time_ = true;
    last_ros_time_ = now.nanoseconds();
    const double cmd_age = have_candidate_ ? std::chrono::duration<double>(steady_now - last_candidate_).count() : 1e9;
    const double scan_age = have_scan_ ? std::chrono::duration<double>(steady_now - scan_received_).count() : 1e9;
    const double scan_stamp_age = have_scan_ ? (now - scan_stamp_).seconds() : 1e9;

    double v = 0.0;
    double w = 0.0;
    std::string reason = "idle";

    if (estop_) {
      reason = "estop";
    } else if (have_candidate_ && !candidate_valid_) {
      reason = "command_invalid";
    } else if (!have_candidate_ || cmd_age > stale_timeout_) {
      reason = "command_stale";
    } else if (scan_age > sensor_timeout_ || !points_valid_ ||
      scan_stamp_age > sensor_timeout_ || scan_stamp_age < -sensor_future_tolerance_)
    {
      reason = "sensor_timeout";
    } else {
      const double target_v = std::clamp(cand_v_, 0.0, v_max_);
      const double target_w = std::clamp(cand_w_, -w_max_, w_max_);
      const double dv = accel_ * control_dt_;
      const double dw = alpha_ * control_dt_;
      v = prev_v_ + std::clamp(target_v - prev_v_, -dv, dv);
      w = prev_w_ + std::clamp(target_w - prev_w_, -dw, dw);
      reason = "ok";

      const auto filtered = safety_pkg::filter_velocity(v, w, ranges_, scan_config_);
      v = filtered.v;
      w = filtered.w;
      reason = filtered.intervened ? filtered.reason : "ok";
    }

    const bool intervening = reason == "clearance" || reason == "braking" ||
      reason == "sensor_timeout" || reason == "footprint_overlap" ||
      reason == "braking_sweep" || reason == "no_valid_scan" ||
      reason.find("scan_unknown_") == 0;
    if (intervening) {
      if (!was_intervening_) {
        ++interventions_;
        RCLCPP_WARN(
          get_logger(), "intervention %lu: %s (v=%.2f w=%.2f)",
          static_cast<unsigned long>(interventions_), reason.c_str(), v, w);
      }
    }
    was_intervening_ = intervening;
    last_reason_ = reason;

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = v;
    cmd.angular.z = w;
    pub_cmd_->publish(cmd);
    prev_v_ = v;
    prev_w_ = w;

    if (std::chrono::duration<double>(steady_now - last_state_pub_).count() > 0.5) {
      std_msgs::msg::Bool state;
      state.data = estop_;
      pub_estop_->publish(state);
      last_state_pub_ = steady_now;
    }

    std_msgs::msg::String status;
    std::ostringstream out;
    out << "{\"estop\":" << (estop_ ? "true" : "false")
        << ",\"source\":\"" << candidate_topic_ << "\""
        << ",\"reason\":\"" << reason << "\""
        << ",\"v_target\":" << cand_v_ << ",\"w_target\":" << cand_w_
        << ",\"v_exec\":" << v << ",\"w_exec\":" << w
        << ",\"cmd_age\":" << cmd_age << ",\"scan_age\":" << scan_age
        << ",\"scan_stamp_age\":" << scan_stamp_age
        << ",\"interventions\":" << interventions_ << "}";
    status.data = out.str();
    pub_status_->publish(status);
  }

  std::string candidate_topic_;
  double v_max_ = 0.26;
  double w_max_ = 1.0;
  double accel_ = 0.5;
  double alpha_ = 2.0;
  double control_dt_ = 0.05;
  double stale_timeout_ = 0.5;
  double sensor_timeout_ = 0.5;
  double radius_ = 0.226;
  double margin_ = 0.04;
  double reaction_time_ = 0.2;
  double brake_deceleration_ = 0.8;
  bool strict_scan_coverage_ = false;
  int canonical_beams_ = 360;
  double sensor_future_tolerance_ = 0.15;
  double configured_range_min_ = 0.24;
  double configured_range_max_ = 10.0;

  double cand_v_ = 0.0;
  double cand_w_ = 0.0;
  bool have_candidate_ = false;
  bool candidate_valid_ = false;
  bool have_ros_time_ = false;
  int64_t last_ros_time_ = 0;
  std::chrono::steady_clock::time_point last_candidate_;
  double prev_v_ = 0.0;
  double prev_w_ = 0.0;

  std::vector<float> ranges_;
  safety_pkg::SafetyConfig scan_config_;
  bool points_valid_ = false;
  bool have_scan_ = false;
  rclcpp::Time scan_stamp_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point scan_received_;

  bool estop_ = false;
  bool estop_request_ = false;
  std::string last_reason_ = "init";
  size_t interventions_ = 0;
  bool was_intervening_ = false;
  std::chrono::steady_clock::time_point last_state_pub_;

  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_estop_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_candidate_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_estop_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyNode>());
  rclcpp::shutdown();
  return 0;
}

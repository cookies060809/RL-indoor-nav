#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{
constexpr double kPi = 3.14159265358979323846;

struct Pose2D
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

Pose2D integrate(double v, double w, double dt)
{
  Pose2D out;
  if (std::fabs(w) < 1e-8) {
    out.x = v * dt;
  } else {
    out.x = v / w * std::sin(w * dt);
    out.y = v / w * (1.0 - std::cos(w * dt));
  }
  return out;
}
}  // namespace

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

    pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    pub_estop_ = create_publisher<std_msgs::msg::Bool>("/estop_state", 10);
    pub_status_ = create_publisher<std_msgs::msg::String>("/safety/status", 10);

    sub_candidate_ = create_subscription<geometry_msgs::msg::Twist>(
      candidate_topic_, 10,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        cand_v_ = msg->linear.x;
        cand_w_ = msg->angular.z;
        last_candidate_ = this->now();
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
        const double scan_age = have_scan_ ? (this->now() - scan_stamp_).seconds() : 1e9;
        if (estop_request_ || scan_age > sensor_timeout_) {
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
    const double sx = tf.transform.translation.x;
    const double sy = tf.transform.translation.y;
    const double syaw = 2.0 * std::atan2(tf.transform.rotation.z, tf.transform.rotation.w);
    const size_t count = msg->ranges.size();
    points_.clear();
    points_.reserve(count);
    bool has_inf = false;
    for (size_t i = 0; i < count; ++i) {
      const float r = msg->ranges[i];
      if (std::isinf(r) && r > 0.0F) {
        has_inf = true;
        continue;
      }
      if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) {
        continue;
      }
      const double angle = -kPi + static_cast<double>(i) * 2.0 * kPi / count + syaw;
      points_.push_back(static_cast<float>(r * std::cos(angle) + sx));
      points_.push_back(static_cast<float>(r * std::sin(angle) + sy));
    }
    points_valid_ = !points_.empty() || has_inf;
    scan_stamp_ = stamp;
    have_scan_ = true;
  }

  bool braking(double v, double w) const
  {
    if (v <= 1e-6) {
      return false;
    }
    const double stopping = v * reaction_time_ + v * v / (2.0 * brake_deceleration_);
    const double duration = stopping / std::max(v, 1e-8);
    const double limit = radius_ + margin_;
    for (int i = 0; i < 7; ++i) {
      const double fraction = 0.15 + static_cast<double>(i) * (0.85 / 6.0);
      const Pose2D future = integrate(v, w, duration * fraction);
      for (size_t k = 0; k + 1 < points_.size(); k += 2) {
        const double dx = points_[k] - future.x;
        const double dy = points_[k + 1] - future.y;
        if (std::hypot(dx, dy) < limit) {
          return true;
        }
      }
    }
    return false;
  }

  void tick()
  {
    const rclcpp::Time now = this->now();
    const double cmd_age = have_candidate_ ? (now - last_candidate_).seconds() : 1e9;
    const double scan_age = have_scan_ ? (now - scan_stamp_).seconds() : 1e9;

    double v = 0.0;
    double w = 0.0;
    std::string reason = "idle";

    if (estop_) {
      reason = "estop";
    } else if (!have_candidate_ || cmd_age > stale_timeout_) {
      reason = "command_stale";
    } else if (scan_age > sensor_timeout_ || !points_valid_) {
      reason = "sensor_timeout";
    } else {
      const double target_v = std::clamp(cand_v_, 0.0, v_max_);
      const double target_w = std::clamp(cand_w_, -w_max_, w_max_);
      const double dv = accel_ * control_dt_;
      const double dw = alpha_ * control_dt_;
      v = prev_v_ + std::clamp(target_v - prev_v_, -dv, dv);
      w = prev_w_ + std::clamp(target_w - prev_w_, -dw, dw);
      reason = "ok";

      double min_distance = 1e9;
      for (size_t k = 0; k + 1 < points_.size(); k += 2) {
        const double d = std::hypot(points_[k], points_[k + 1]);
        if (d < min_distance) {
          min_distance = d;
        }
      }
      if (min_distance < radius_ + margin_) {
        v = 0.0;
        w = 0.0;
        reason = "clearance";
      } else if (braking(v, w)) {
        v = 0.0;
        reason = "braking";
      }
    }

    if (reason == "clearance" || reason == "braking" || reason == "sensor_timeout") {
      if (reason != last_reason_) {
        ++interventions_;
        RCLCPP_WARN(
          get_logger(), "intervention %lu: %s (v=%.2f w=%.2f)",
          static_cast<unsigned long>(interventions_), reason.c_str(), v, w);
      }
    }
    last_reason_ = reason;

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = v;
    cmd.angular.z = w;
    pub_cmd_->publish(cmd);
    prev_v_ = v;
    prev_w_ = w;

    if (now.nanoseconds() - last_state_pub_.nanoseconds() > 500000000LL) {
      std_msgs::msg::Bool state;
      state.data = estop_;
      pub_estop_->publish(state);
      last_state_pub_ = now;
    }

    std_msgs::msg::String status;
    std::ostringstream out;
    out << "{\"estop\":" << (estop_ ? "true" : "false")
        << ",\"source\":\"" << candidate_topic_ << "\""
        << ",\"reason\":\"" << reason << "\""
        << ",\"v_target\":" << cand_v_ << ",\"w_target\":" << cand_w_
        << ",\"v_exec\":" << v << ",\"w_exec\":" << w
        << ",\"cmd_age\":" << cmd_age << ",\"scan_age\":" << scan_age
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

  double cand_v_ = 0.0;
  double cand_w_ = 0.0;
  bool have_candidate_ = false;
  rclcpp::Time last_candidate_{0, 0, RCL_ROS_TIME};
  double prev_v_ = 0.0;
  double prev_w_ = 0.0;

  std::vector<float> points_;
  bool points_valid_ = false;
  bool have_scan_ = false;
  rclcpp::Time scan_stamp_{0, 0, RCL_ROS_TIME};

  bool estop_ = false;
  bool estop_request_ = false;
  std::string last_reason_ = "init";
  size_t interventions_ = 0;
  rclcpp::Time last_state_pub_{0, 0, RCL_ROS_TIME};

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

#include "osracer_aggressive_backup/aggressive_back_up.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace osracer_aggressive_backup
{

AggressiveBackUp::AggressiveBackUp()
: feedback_(std::make_shared<BackUpAction::Feedback>()),
  odom_topic_("odometry/filtered"),
  fallback_recovery_direction_("backward"),
  default_distance_(0.6),
  default_speed_(0.35),
  stopped_velocity_threshold_(0.03),
  last_odom_linear_x_(0.0),
  has_odom_(false),
  command_distance_(0.0),
  command_speed_(0.0),
  run_duration_(0.0),
  time_allowance_(0, 0)
{
}

void AggressiveBackUp::onConfigure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_ + ".default_distance", rclcpp::ParameterValue(default_distance_));
  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_ + ".default_speed", rclcpp::ParameterValue(default_speed_));
  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_ + ".odom_topic", rclcpp::ParameterValue(odom_topic_));
  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_ + ".stopped_velocity_threshold",
    rclcpp::ParameterValue(stopped_velocity_threshold_));
  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_ + ".fallback_recovery_direction",
    rclcpp::ParameterValue(fallback_recovery_direction_));

  node->get_parameter(behavior_name_ + ".default_distance", default_distance_);
  node->get_parameter(behavior_name_ + ".default_speed", default_speed_);
  node->get_parameter(behavior_name_ + ".odom_topic", odom_topic_);
  node->get_parameter(
    behavior_name_ + ".stopped_velocity_threshold", stopped_velocity_threshold_);
  node->get_parameter(
    behavior_name_ + ".fallback_recovery_direction", fallback_recovery_direction_);

  default_distance_ = std::fabs(default_distance_);
  default_speed_ = std::fabs(default_speed_);
  stopped_velocity_threshold_ = std::fabs(stopped_velocity_threshold_);

  if (default_speed_ <= 0.0) {
    RCLCPP_WARN(logger_, "default_speed must be positive; using 0.35 m/s");
    default_speed_ = 0.35;
  }

  if (fallback_recovery_direction_ != "forward" && fallback_recovery_direction_ != "backward") {
    RCLCPP_WARN(
      logger_,
      "fallback_recovery_direction must be 'forward' or 'backward'; using 'backward'");
    fallback_recovery_direction_ = "backward";
  }

  odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(&AggressiveBackUp::odomCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    logger_,
    "AggressiveBackUp configured: default_distance=%.3f m, default_speed=%.3f m/s, "
    "odom_topic=%s, stopped_velocity_threshold=%.3f m/s, fallback_recovery_direction=%s",
    default_distance_, default_speed_, odom_topic_.c_str(), stopped_velocity_threshold_,
    fallback_recovery_direction_.c_str());
}

void AggressiveBackUp::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  last_odom_linear_x_ = msg->twist.twist.linear.x;
  has_odom_ = true;
}

double AggressiveBackUp::getRecoveryDirection() const
{
  if (has_odom_ && std::fabs(last_odom_linear_x_) > stopped_velocity_threshold_) {
    return last_odom_linear_x_ > 0.0 ? -1.0 : 1.0;
  }

  return fallback_recovery_direction_ == "forward" ? 1.0 : -1.0;
}

nav2_behaviors::Status AggressiveBackUp::onRun(
  const std::shared_ptr<const BackUpAction::Goal> command)
{
  if (command->target.y != 0.0 || command->target.z != 0.0) {
    RCLCPP_ERROR(logger_, "AggressiveBackUp only supports x-axis recovery goals");
    return nav2_behaviors::Status::FAILED;
  }

  const double requested_distance = std::fabs(command->target.x);
  const double requested_speed = std::fabs(command->speed);
  const double recovery_direction = getRecoveryDirection();

  command_distance_ = requested_distance > 0.0 ? requested_distance : default_distance_;
  command_speed_ = recovery_direction * (requested_speed > 0.0 ? requested_speed : default_speed_);
  run_duration_ = command_distance_ / std::fabs(command_speed_);
  time_allowance_ = command->time_allowance;

  RCLCPP_WARN(
    logger_,
    "AggressiveBackUp TRIGGERED: goal_target_x=%.3f, goal_speed=%.3f, "
    "last_odom_linear_x=%.3f, has_odom=%s, using_distance=%.3f m, "
    "using_speed=%.3f m/s, duration=%.3f s, time_allowance=%.3f s, "
    "collision_projection=disabled",
    command->target.x, command->speed, last_odom_linear_x_, has_odom_ ? "true" : "false",
    command_distance_, command_speed_, run_duration_, time_allowance_.seconds());

  return nav2_behaviors::Status::SUCCEEDED;
}

nav2_behaviors::Status AggressiveBackUp::onCycleUpdate()
{
  const double elapsed = elasped_time_.seconds();

  if (time_allowance_.seconds() > 0.0 && elapsed > time_allowance_.seconds()) {
    stopRobot();
    RCLCPP_WARN(logger_, "AggressiveBackUp exceeded time allowance");
    return nav2_behaviors::Status::FAILED;
  }

  feedback_->distance_traveled = std::min(command_distance_, std::fabs(command_speed_) * elapsed);
  action_server_->publish_feedback(feedback_);

  if (elapsed >= run_duration_) {
    stopRobot();
    RCLCPP_WARN(
      logger_,
      "AggressiveBackUp completed: elapsed=%.3f s, estimated_distance=%.3f m",
      elapsed, feedback_->distance_traveled);
    return nav2_behaviors::Status::SUCCEEDED;
  }

  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
  cmd_vel->linear.x = command_speed_;
  vel_pub_->publish(std::move(cmd_vel));

  return nav2_behaviors::Status::RUNNING;
}

}  // namespace osracer_aggressive_backup

PLUGINLIB_EXPORT_CLASS(osracer_aggressive_backup::AggressiveBackUp, nav2_core::Behavior)

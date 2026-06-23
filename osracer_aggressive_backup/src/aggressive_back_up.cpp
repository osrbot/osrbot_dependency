#include "osracer_aggressive_backup/aggressive_back_up.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>

#include "geometry_msgs/msg/twist.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace osracer_aggressive_backup
{

AggressiveBackUp::AggressiveBackUp()
: feedback_(std::make_shared<BackUpAction::Feedback>()),
  default_distance_(0.6),
  default_speed_(0.35),
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

  node->get_parameter(behavior_name_ + ".default_distance", default_distance_);
  node->get_parameter(behavior_name_ + ".default_speed", default_speed_);

  default_distance_ = std::fabs(default_distance_);
  default_speed_ = std::fabs(default_speed_);

  if (default_speed_ <= 0.0) {
    RCLCPP_WARN(logger_, "default_speed must be positive; using 0.35 m/s");
    default_speed_ = 0.35;
  }
}

nav2_behaviors::Status AggressiveBackUp::onRun(
  const std::shared_ptr<const BackUpAction::Goal> command)
{
  if (command->target.y != 0.0 || command->target.z != 0.0) {
    RCLCPP_ERROR(logger_, "AggressiveBackUp only supports x-axis backup goals");
    return nav2_behaviors::Status::FAILED;
  }

  const double requested_distance = std::fabs(command->target.x);
  const double requested_speed = std::fabs(command->speed);

  command_distance_ = requested_distance > 0.0 ? requested_distance : default_distance_;
  command_speed_ = -(requested_speed > 0.0 ? requested_speed : default_speed_);
  run_duration_ = command_distance_ / std::fabs(command_speed_);
  time_allowance_ = command->time_allowance;

  RCLCPP_WARN(
    logger_,
    "AggressiveBackUp backing up %.3f m at %.3f m/s without collision projection",
    command_distance_, command_speed_);

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
    return nav2_behaviors::Status::SUCCEEDED;
  }

  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
  cmd_vel->linear.x = command_speed_;
  vel_pub_->publish(std::move(cmd_vel));

  return nav2_behaviors::Status::RUNNING;
}

}  // namespace osracer_aggressive_backup

PLUGINLIB_EXPORT_CLASS(osracer_aggressive_backup::AggressiveBackUp, nav2_core::Behavior)

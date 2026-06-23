#include "osracer_aggressive_backup/aggressive_back_up.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace osracer_aggressive_backup
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

double degreesToRadians(const double degrees)
{
  return degrees * kPi / 180.0;
}
}  // namespace

AggressiveBackUp::AggressiveBackUp()
: feedback_(std::make_shared<BackUpAction::Feedback>()),
  odom_topic_("odometry/filtered"),
  scan_topic_("scan"),
  fallback_recovery_direction_("backward"),
  local_clear_service_("local_costmap/clear_entirely_local_costmap"),
  global_clear_service_("global_costmap/clear_entirely_global_costmap"),
  scan_base_frame_("base_link"),
  default_distance_(0.6),
  default_speed_(0.35),
  stopped_velocity_threshold_(0.03),
  first_phase_distance_ratio_(0.45),
  second_phase_distance_ratio_(0.55),
  min_exit_clearance_(0.45),
  front_sector_deg_(35.0),
  rear_sector_deg_(35.0),
  costmap_clear_wait_ms_(150.0),
  scan_timeout_(0.35),
  last_odom_linear_x_(0.0),
  has_odom_(false),
  enable_second_phase_(true),
  clear_local_costmap_(true),
  clear_global_costmap_(false),
  phase_(Phase::FIRST_ODOM_ESCAPE),
  odom_recovery_direction_(-1.0),
  command_distance_(0.0),
  command_speed_(0.0),
  run_duration_(0.0),
  phase_distance_(0.0),
  completed_distance_(0.0),
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
    node, behavior_name_ + ".scan_topic", rclcpp::ParameterValue(scan_topic_));
  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_ + ".scan_base_frame", rclcpp::ParameterValue(scan_base_frame_));
  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_ + ".stopped_velocity_threshold",
    rclcpp::ParameterValue(stopped_velocity_threshold_));
  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_ + ".fallback_recovery_direction",
    rclcpp::ParameterValue(fallback_recovery_direction_));
  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_ + ".first_phase_distance_ratio",
    rclcpp::ParameterValue(first_phase_distance_ratio_));
  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_ + ".second_phase_distance_ratio",
    rclcpp::ParameterValue(second_phase_distance_ratio_));
  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_ + ".min_exit_clearance", rclcpp::ParameterValue(min_exit_clearance_));
  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_ + ".front_sector_deg", rclcpp::ParameterValue(front_sector_deg_));
  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_ + ".rear_sector_deg", rclcpp::ParameterValue(rear_sector_deg_));
  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_ + ".scan_timeout", rclcpp::ParameterValue(scan_timeout_));
  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_ + ".enable_second_phase", rclcpp::ParameterValue(enable_second_phase_));
  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_ + ".clear_local_costmap", rclcpp::ParameterValue(clear_local_costmap_));
  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_ + ".clear_global_costmap", rclcpp::ParameterValue(clear_global_costmap_));
  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_ + ".local_clear_service", rclcpp::ParameterValue(local_clear_service_));
  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_ + ".global_clear_service", rclcpp::ParameterValue(global_clear_service_));
  nav2_util::declare_parameter_if_not_declared(
    node, behavior_name_ + ".costmap_clear_wait_ms",
    rclcpp::ParameterValue(costmap_clear_wait_ms_));

  node->get_parameter(behavior_name_ + ".default_distance", default_distance_);
  node->get_parameter(behavior_name_ + ".default_speed", default_speed_);
  node->get_parameter(behavior_name_ + ".odom_topic", odom_topic_);
  node->get_parameter(behavior_name_ + ".scan_topic", scan_topic_);
  node->get_parameter(behavior_name_ + ".scan_base_frame", scan_base_frame_);
  node->get_parameter(
    behavior_name_ + ".stopped_velocity_threshold", stopped_velocity_threshold_);
  node->get_parameter(
    behavior_name_ + ".fallback_recovery_direction", fallback_recovery_direction_);
  node->get_parameter(
    behavior_name_ + ".first_phase_distance_ratio", first_phase_distance_ratio_);
  node->get_parameter(
    behavior_name_ + ".second_phase_distance_ratio", second_phase_distance_ratio_);
  node->get_parameter(behavior_name_ + ".min_exit_clearance", min_exit_clearance_);
  node->get_parameter(behavior_name_ + ".front_sector_deg", front_sector_deg_);
  node->get_parameter(behavior_name_ + ".rear_sector_deg", rear_sector_deg_);
  node->get_parameter(behavior_name_ + ".scan_timeout", scan_timeout_);
  node->get_parameter(behavior_name_ + ".enable_second_phase", enable_second_phase_);
  node->get_parameter(behavior_name_ + ".clear_local_costmap", clear_local_costmap_);
  node->get_parameter(behavior_name_ + ".clear_global_costmap", clear_global_costmap_);
  node->get_parameter(behavior_name_ + ".local_clear_service", local_clear_service_);
  node->get_parameter(behavior_name_ + ".global_clear_service", global_clear_service_);
  node->get_parameter(behavior_name_ + ".costmap_clear_wait_ms", costmap_clear_wait_ms_);

  default_distance_ = std::fabs(default_distance_);
  default_speed_ = std::fabs(default_speed_);
  stopped_velocity_threshold_ = std::fabs(stopped_velocity_threshold_);
  min_exit_clearance_ = std::fabs(min_exit_clearance_);
  first_phase_distance_ratio_ = std::clamp(first_phase_distance_ratio_, 0.0, 1.0);
  second_phase_distance_ratio_ = std::clamp(second_phase_distance_ratio_, 0.0, 1.0);

  if (default_speed_ <= 0.0) {
    RCLCPP_WARN(logger_, "default_speed must be positive; using 0.35 m/s");
    default_speed_ = 0.35;
  }

  if (first_phase_distance_ratio_ + second_phase_distance_ratio_ <= 0.0) {
    RCLCPP_WARN(logger_, "phase distance ratios are zero; using 0.45/0.55");
    first_phase_distance_ratio_ = 0.45;
    second_phase_distance_ratio_ = 0.55;
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
  scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, rclcpp::SensorDataQoS(),
    std::bind(&AggressiveBackUp::scanCallback, this, std::placeholders::_1));
  local_clear_client_ = node->create_client<ClearCostmap>(local_clear_service_);
  global_clear_client_ = node->create_client<ClearCostmap>(global_clear_service_);

  RCLCPP_INFO(
    logger_,
    "AggressiveBackUp configured: default_distance=%.3f m, default_speed=%.3f m/s, "
    "odom_topic=%s, scan_topic=%s, scan_base_frame=%s, stopped_velocity_threshold=%.3f m/s, "
    "fallback_recovery_direction=%s, enable_second_phase=%s, phase_ratios=%.2f/%.2f, "
    "clear_local=%s, clear_global=%s",
    default_distance_, default_speed_, odom_topic_.c_str(), scan_topic_.c_str(),
    scan_base_frame_.c_str(), stopped_velocity_threshold_, fallback_recovery_direction_.c_str(),
    enable_second_phase_ ? "true" : "false", first_phase_distance_ratio_,
    second_phase_distance_ratio_, clear_local_costmap_ ? "true" : "false",
    clear_global_costmap_ ? "true" : "false");
}

void AggressiveBackUp::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  last_odom_linear_x_ = msg->twist.twist.linear.x;
  has_odom_ = true;
}

void AggressiveBackUp::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  latest_scan_ = msg;
}

double AggressiveBackUp::getFallbackRecoveryDirection() const
{
  return fallback_recovery_direction_ == "forward" ? 1.0 : -1.0;
}

double AggressiveBackUp::getOdomRecoveryDirection() const
{
  if (has_odom_ && std::fabs(last_odom_linear_x_) > stopped_velocity_threshold_) {
    return last_odom_linear_x_ > 0.0 ? -1.0 : 1.0;
  }

  return getFallbackRecoveryDirection();
}

void AggressiveBackUp::requestClear(
  const rclcpp::Client<ClearCostmap>::SharedPtr & client,
  const std::string & service_name)
{
  if (!client->service_is_ready()) {
    RCLCPP_WARN(logger_, "Costmap clear service is not ready: %s", service_name.c_str());
    return;
  }

  client->async_send_request(std::make_shared<ClearCostmap::Request>());
  RCLCPP_WARN(logger_, "Requested costmap clear: %s", service_name.c_str());
}

void AggressiveBackUp::clearCostmaps()
{
  if (clear_local_costmap_) {
    requestClear(local_clear_client_, local_clear_service_);
  }
  if (clear_global_costmap_) {
    requestClear(global_clear_client_, global_clear_service_);
  }
  if (costmap_clear_wait_ms_ > 0.0) {
    rclcpp::sleep_for(
      std::chrono::milliseconds(static_cast<int>(std::round(costmap_clear_wait_ms_))));
  }
}

AggressiveBackUp::ScanClearance AggressiveBackUp::evaluateScanClearance() const
{
  ScanClearance clearance;
  if (!latest_scan_) {
    return clearance;
  }

  const auto scan_age = (clock_->now() - latest_scan_->header.stamp).seconds();
  if (scan_age > scan_timeout_) {
    RCLCPP_WARN(
      logger_, "Latest scan is stale: age=%.3f s, timeout=%.3f s", scan_age, scan_timeout_);
    return clearance;
  }

  const double front_limit = degreesToRadians(front_sector_deg_);
  const double rear_limit = degreesToRadians(rear_sector_deg_);
  double front_min = std::numeric_limits<double>::infinity();
  double rear_min = std::numeric_limits<double>::infinity();

  for (size_t i = 0; i < latest_scan_->ranges.size(); ++i) {
    const double range = latest_scan_->ranges[i];
    if (!std::isfinite(range) || range < latest_scan_->range_min || range > latest_scan_->range_max) {
      continue;
    }

    const double angle = latest_scan_->angle_min + static_cast<double>(i) * latest_scan_->angle_increment;
    geometry_msgs::msg::PointStamped point_in;
    point_in.header = latest_scan_->header;
    point_in.point.x = range * std::cos(angle);
    point_in.point.y = range * std::sin(angle);
    point_in.point.z = 0.0;

    geometry_msgs::msg::PointStamped point_base;
    try {
      tf_->transform(point_in, point_base, scan_base_frame_, tf2::durationFromSec(0.05));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(logger_, "Failed to transform scan point to %s: %s", scan_base_frame_.c_str(), ex.what());
      return clearance;
    }

    const double base_angle = std::atan2(point_base.point.y, point_base.point.x);
    const double base_range = std::hypot(point_base.point.x, point_base.point.y);
    if (std::fabs(base_angle) <= front_limit) {
      front_min = std::min(front_min, base_range);
    }
    if (std::fabs(std::fabs(base_angle) - kPi) <= rear_limit) {
      rear_min = std::min(rear_min, base_range);
    }
  }

  clearance.valid = std::isfinite(front_min) || std::isfinite(rear_min);
  clearance.front = std::isfinite(front_min) ? front_min : latest_scan_->range_max;
  clearance.rear = std::isfinite(rear_min) ? rear_min : latest_scan_->range_max;
  return clearance;
}

bool AggressiveBackUp::isObstacleCleared(const ScanClearance & clearance) const
{
  return clearance.valid && clearance.front >= min_exit_clearance_ &&
         clearance.rear >= min_exit_clearance_;
}

double AggressiveBackUp::chooseScanRecoveryDirection(
  const double odom_direction, const ScanClearance & clearance) const
{
  if (!clearance.valid) {
    RCLCPP_WARN(logger_, "Scan clearance unavailable; keeping odom recovery direction %.1f", odom_direction);
    return odom_direction;
  }

  const bool front_safe = clearance.front >= min_exit_clearance_;
  const bool rear_safe = clearance.rear >= min_exit_clearance_;
  double direction = odom_direction;

  if (odom_direction > 0.0) {
    direction = front_safe || !rear_safe ? 1.0 : -1.0;
  } else {
    direction = rear_safe || !front_safe ? -1.0 : 1.0;
  }

  if (front_safe && rear_safe) {
    direction = odom_direction;
  } else if (!front_safe && !rear_safe) {
    direction = clearance.front > clearance.rear ? 1.0 : -1.0;
  }

  RCLCPP_WARN(
    logger_,
    "AggressiveBackUp scan decision: front_clearance=%.3f m, rear_clearance=%.3f m, "
    "min_exit_clearance=%.3f m, odom_direction=%.1f, chosen_direction=%.1f",
    clearance.front, clearance.rear, min_exit_clearance_, odom_direction, direction);
  return direction;
}

void AggressiveBackUp::startPhase(
  const Phase phase, const double direction, const double distance,
  const double speed, const rclcpp::Time & now)
{
  phase_ = phase;
  phase_distance_ = distance;
  command_speed_ = direction * speed;
  run_duration_ = phase_distance_ / std::fabs(command_speed_);
  phase_start_time_ = now;

  RCLCPP_WARN(
    logger_, "AggressiveBackUp phase %s: distance=%.3f m, speed=%.3f m/s, duration=%.3f s",
    phase_ == Phase::FIRST_ODOM_ESCAPE ? "first_odom_escape" : "second_scan_escape",
    phase_distance_, command_speed_, run_duration_);
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
  command_distance_ = requested_distance > 0.0 ? requested_distance : default_distance_;
  const double speed = requested_speed > 0.0 ? requested_speed : default_speed_;
  time_allowance_ = command->time_allowance;
  completed_distance_ = 0.0;
  odom_recovery_direction_ = getOdomRecoveryDirection();

  const double ratio_sum = first_phase_distance_ratio_ + second_phase_distance_ratio_;
  const double first_distance = enable_second_phase_ ?
    command_distance_ * first_phase_distance_ratio_ / ratio_sum : command_distance_;

  RCLCPP_WARN(
    logger_,
    "AggressiveBackUp TRIGGERED: goal_target_x=%.3f, goal_speed=%.3f, "
    "last_odom_linear_x=%.3f, has_odom=%s, total_distance=%.3f m, speed=%.3f m/s, "
    "odom_direction=%.1f, time_allowance=%.3f s, collision_projection=disabled",
    command->target.x, command->speed, last_odom_linear_x_, has_odom_ ? "true" : "false",
    command_distance_, speed, odom_recovery_direction_, time_allowance_.seconds());

  startPhase(Phase::FIRST_ODOM_ESCAPE, odom_recovery_direction_, first_distance, speed, clock_->now());
  return nav2_behaviors::Status::SUCCEEDED;
}

nav2_behaviors::Status AggressiveBackUp::onCycleUpdate()
{
  if (time_allowance_.seconds() > 0.0 && elasped_time_.seconds() > time_allowance_.seconds()) {
    stopRobot();
    RCLCPP_WARN(logger_, "AggressiveBackUp exceeded time allowance");
    return nav2_behaviors::Status::FAILED;
  }

  const double phase_elapsed = (clock_->now() - phase_start_time_).seconds();
  const double phase_traveled = std::min(phase_distance_, std::fabs(command_speed_) * phase_elapsed);
  feedback_->distance_traveled = std::min(command_distance_, completed_distance_ + phase_traveled);
  action_server_->publish_feedback(feedback_);

  if (phase_elapsed >= run_duration_) {
    completed_distance_ += phase_distance_;
    if (phase_ == Phase::FIRST_ODOM_ESCAPE) {
      stopRobot();
      clearCostmaps();
      const double remaining_distance = std::max(0.0, command_distance_ - completed_distance_);
      if (remaining_distance <= 0.0 || !enable_second_phase_) {
        RCLCPP_WARN(
          logger_,
          "AggressiveBackUp completed after first phase: remaining_distance=%.3f m, "
          "enable_second_phase=%s",
          remaining_distance, enable_second_phase_ ? "true" : "false");
        return nav2_behaviors::Status::SUCCEEDED;
      }
      const auto clearance = evaluateScanClearance();
      if (isObstacleCleared(clearance)) {
        RCLCPP_WARN(
          logger_,
          "AggressiveBackUp completed after first phase: front_clearance=%.3f m, "
          "rear_clearance=%.3f m, min_exit_clearance=%.3f m",
          clearance.front, clearance.rear, min_exit_clearance_);
        return nav2_behaviors::Status::SUCCEEDED;
      }
      const double scan_direction = chooseScanRecoveryDirection(odom_recovery_direction_, clearance);
      startPhase(
        Phase::SECOND_SCAN_ESCAPE, scan_direction, remaining_distance,
        std::fabs(command_speed_), clock_->now());
      return nav2_behaviors::Status::RUNNING;
    }

    stopRobot();
    RCLCPP_WARN(
      logger_,
      "AggressiveBackUp completed: elapsed=%.3f s, estimated_distance=%.3f m",
      elasped_time_.seconds(), feedback_->distance_traveled);
    return nav2_behaviors::Status::SUCCEEDED;
  }

  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
  cmd_vel->linear.x = command_speed_;
  vel_pub_->publish(std::move(cmd_vel));

  return nav2_behaviors::Status::RUNNING;
}

}  // namespace osracer_aggressive_backup

PLUGINLIB_EXPORT_CLASS(osracer_aggressive_backup::AggressiveBackUp, nav2_core::Behavior)

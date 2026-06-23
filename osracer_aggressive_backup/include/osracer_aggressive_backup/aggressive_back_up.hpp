#ifndef OSRACER_AGGRESSIVE_BACKUP__AGGRESSIVE_BACK_UP_HPP_
#define OSRACER_AGGRESSIVE_BACKUP__AGGRESSIVE_BACK_UP_HPP_

#include <memory>
#include <string>

#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace osracer_aggressive_backup
{

class AggressiveBackUp : public nav2_behaviors::TimedBehavior<nav2_msgs::action::BackUp>
{
public:
  using BackUpAction = nav2_msgs::action::BackUp;

  AggressiveBackUp();

  nav2_behaviors::Status onRun(
    const std::shared_ptr<const BackUpAction::Goal> command) override;
  nav2_behaviors::Status onCycleUpdate() override;

protected:
  void onConfigure() override;

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  double getRecoveryDirection() const;

  BackUpAction::Feedback::SharedPtr feedback_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::string odom_topic_;
  std::string fallback_recovery_direction_;
  double default_distance_;
  double default_speed_;
  double stopped_velocity_threshold_;
  double last_odom_linear_x_;
  bool has_odom_;
  double command_distance_;
  double command_speed_;
  double run_duration_;
  rclcpp::Duration time_allowance_;
};

}  // namespace osracer_aggressive_backup

#endif  // OSRACER_AGGRESSIVE_BACKUP__AGGRESSIVE_BACK_UP_HPP_

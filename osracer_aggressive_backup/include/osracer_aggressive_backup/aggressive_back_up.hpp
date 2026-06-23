#ifndef OSRACER_AGGRESSIVE_BACKUP__AGGRESSIVE_BACK_UP_HPP_
#define OSRACER_AGGRESSIVE_BACKUP__AGGRESSIVE_BACK_UP_HPP_

#include <memory>

#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/back_up.hpp"

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
  BackUpAction::Feedback::SharedPtr feedback_;
  double default_distance_;
  double default_speed_;
  double command_distance_;
  double command_speed_;
  double run_duration_;
  rclcpp::Duration time_allowance_;
};

}  // namespace osracer_aggressive_backup

#endif  // OSRACER_AGGRESSIVE_BACKUP__AGGRESSIVE_BACK_UP_HPP_

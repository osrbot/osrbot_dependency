#ifndef OSRACER_AGGRESSIVE_BACKUP__AGGRESSIVE_BACK_UP_HPP_
#define OSRACER_AGGRESSIVE_BACKUP__AGGRESSIVE_BACK_UP_HPP_

#include <memory>
#include <string>

#include "nav2_behaviors/timed_behavior.hpp"
#include "nav2_msgs/action/back_up.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace osracer_aggressive_backup
{

class AggressiveBackUp : public nav2_behaviors::TimedBehavior<nav2_msgs::action::BackUp>
{
public:
  using BackUpAction = nav2_msgs::action::BackUp;
  using ClearCostmap = nav2_msgs::srv::ClearEntireCostmap;

  AggressiveBackUp();

  nav2_behaviors::Status onRun(
    const std::shared_ptr<const BackUpAction::Goal> command) override;
  nav2_behaviors::Status onCycleUpdate() override;

protected:
  void onConfigure() override;

private:
  enum class Phase
  {
    FIRST_ODOM_ESCAPE,
    SECOND_SCAN_ESCAPE
  };

  struct ScanClearance
  {
    bool valid{false};
    double front{0.0};
    double rear{0.0};
  };

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void clearCostmaps();
  void requestClear(
    const rclcpp::Client<ClearCostmap>::SharedPtr & client,
    const std::string & service_name);
  double getOdomRecoveryDirection() const;
  double getFallbackRecoveryDirection() const;
  double chooseScanRecoveryDirection(const double odom_direction);
  ScanClearance evaluateScanClearance() const;
  void startPhase(
    const Phase phase, const double direction, const double distance,
    const double speed, const rclcpp::Time & now);

  BackUpAction::Feedback::SharedPtr feedback_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Client<ClearCostmap>::SharedPtr local_clear_client_;
  rclcpp::Client<ClearCostmap>::SharedPtr global_clear_client_;

  std::string odom_topic_;
  std::string scan_topic_;
  std::string fallback_recovery_direction_;
  std::string local_clear_service_;
  std::string global_clear_service_;
  std::string scan_base_frame_;

  double default_distance_;
  double default_speed_;
  double stopped_velocity_threshold_;
  double first_phase_distance_ratio_;
  double second_phase_distance_ratio_;
  double min_exit_clearance_;
  double front_sector_deg_;
  double rear_sector_deg_;
  double costmap_clear_wait_ms_;
  double scan_timeout_;
  double last_odom_linear_x_;
  bool has_odom_;
  bool clear_local_costmap_;
  bool clear_global_costmap_;

  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  Phase phase_;
  double odom_recovery_direction_;
  double command_distance_;
  double command_speed_;
  double run_duration_;
  double phase_distance_;
  double completed_distance_;
  rclcpp::Time phase_start_time_;
  rclcpp::Duration time_allowance_;
};

}  // namespace osracer_aggressive_backup

#endif  // OSRACER_AGGRESSIVE_BACKUP__AGGRESSIVE_BACK_UP_HPP_

# Changelog

All notable changes to `osracer_aggressive_backup` are documented in this file.

## 0.1.0

- Added `osracer_aggressive_backup/AggressiveBackUp`, a Nav2 behavior plugin
  implementing the standard `nav2_msgs/action/BackUp` action.
- Added aggressive recovery without Nav2 forward collision projection, intended
  for robots already inside an obstacle or inflated obstacle layer.
- Added odometry-aware escape direction:
  - forward motion before recovery commands reverse escape;
  - reverse motion before recovery commands forward escape;
  - stopped or missing odom falls back to a configurable direction.
- Added optional scan-aware second recovery phase:
  - transforms LaserScan points into the configured base frame;
  - evaluates front and rear clearance sectors;
  - skips second phase when the first phase already clears the robot;
  - chooses a safer second-phase exit direction when needed.
- Added optional local/global costmap clear requests between recovery phases.
- Added configurable recovery topics, thresholds, scan sectors, clear services,
  phase ratios, and fallback behavior.

# osracer_aggressive_backup

`osracer_aggressive_backup` is a Nav2 behavior plugin for getting an OSRacer-style
Ackermann robot out of a collision or near-collision state.

The plugin intentionally does not use Nav2's forward collision projection during
the escape motion. When the robot is already inside an inflated obstacle layer or
has slipped into a real obstacle, standard collision projection can prevent any
recovery command from being published. This plugin first commands a short,
aggressive escape based on recent odometry, then optionally uses the latest laser
scan to decide whether a second escape phase is needed.

## Behavior Summary

The plugin exports:

```text
osracer_aggressive_backup/AggressiveBackUp
```

It implements the standard Nav2 `nav2_msgs/action/BackUp` action interface, so it
can be used by the normal Nav2 `BackUp` behavior-tree node.

Recovery flow:

1. Read the latest odometry linear velocity.
2. Run the first escape phase opposite the recent odom direction.
   - If the robot was moving forward, command reverse motion.
   - If the robot was moving backward, command forward motion.
   - If odom is unavailable or nearly stopped, use the configured fallback direction.
3. Stop and optionally clear costmaps.
4. If `enable_second_phase` is false, finish after the first phase.
5. If `enable_second_phase` is true, transform the latest scan points into
   `scan_base_frame`, evaluate front and rear clearance, and:
   - finish immediately if both front and rear clearances are safe;
   - otherwise run the second phase in the safer exit direction.

## Nav2 Configuration

Configure the behavior server to load this plugin:

```yaml
behavior_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["backup", "wait"]
    backup:
      plugin: "osracer_aggressive_backup/AggressiveBackUp"
      default_distance: 0.6
      default_speed: 0.6
      odom_topic: odometry/filtered
      stopped_velocity_threshold: 0.03
      fallback_recovery_direction: backward
      enable_second_phase: true
      scan_topic: scan
      scan_base_frame: base_link
      first_phase_distance_ratio: 0.45
      second_phase_distance_ratio: 0.55
      min_exit_clearance: 0.45
      front_sector_deg: 35.0
      rear_sector_deg: 35.0
      scan_timeout: 0.35
      clear_local_costmap: true
      clear_global_costmap: false
      local_clear_service: local_costmap/clear_entirely_local_costmap
      global_clear_service: global_costmap/clear_entirely_global_costmap
      costmap_clear_wait_ms: 150.0
    wait:
      plugin: "nav2_behaviors/Wait"
```

If `spin` is not loaded in `behavior_plugins`, do not use Nav2's default recovery
trees because they contain a `Spin` recovery node. Use a behavior tree that only
references available actions, for example `BackUp` and `Wait`.

Example recovery subtree:

```xml
<RoundRobin name="RecoveryActions">
  <Sequence name="ClearingActions">
    <ClearEntireCostmap name="ClearLocalCostmap-Subtree"
      service_name="local_costmap/clear_entirely_local_costmap"/>
    <ClearEntireCostmap name="ClearGlobalCostmap-Subtree"
      service_name="global_costmap/clear_entirely_global_costmap"/>
  </Sequence>
  <BackUp backup_dist="0.60" backup_speed="0.60"/>
  <Wait wait_duration="0"/>
</RoundRobin>
```

## Parameters

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `default_distance` | double | `0.6` | Escape distance in meters when the BT action goal does not provide a positive distance. |
| `default_speed` | double | `0.35` | Escape speed magnitude in meters per second when the BT action goal does not provide a positive speed. |
| `odom_topic` | string | `odometry/filtered` | Odometry topic used to infer the recent movement direction. |
| `stopped_velocity_threshold` | double | `0.03` | Absolute odom linear-x threshold below which the robot is treated as stopped. |
| `fallback_recovery_direction` | string | `backward` | Direction used when odom is unavailable or below threshold. Must be `forward` or `backward`. |
| `enable_second_phase` | bool | `true` | Enables scan-aware second-phase recovery. If false, the first odom-based phase runs the full distance. |
| `scan_topic` | string | `scan` | LaserScan topic used for second-phase clearance checks. |
| `scan_base_frame` | string | `base_link` | Frame used for front/rear clearance evaluation after TF-transforming scan points. |
| `first_phase_distance_ratio` | double | `0.45` | Fraction of total distance used by the first odom-based phase when second phase is enabled. |
| `second_phase_distance_ratio` | double | `0.55` | Fraction used to compute phase proportions with `first_phase_distance_ratio`. |
| `min_exit_clearance` | double | `0.45` | Minimum front and rear clearance required to skip the second phase. |
| `front_sector_deg` | double | `35.0` | Half-angle of the forward sector around the positive X axis in `scan_base_frame`. |
| `rear_sector_deg` | double | `35.0` | Half-angle of the rear sector around the negative X axis in `scan_base_frame`. |
| `scan_timeout` | double | `0.35` | Maximum accepted scan age in seconds. Stale scans are ignored. |
| `clear_local_costmap` | bool | `true` | Whether to call the local costmap clear service after the first phase. |
| `clear_global_costmap` | bool | `false` | Whether to call the global costmap clear service after the first phase. |
| `local_clear_service` | string | `local_costmap/clear_entirely_local_costmap` | Local costmap clear service name. |
| `global_clear_service` | string | `global_costmap/clear_entirely_global_costmap` | Global costmap clear service name. |
| `costmap_clear_wait_ms` | double | `150.0` | Delay after clear requests before scan evaluation and optional second phase. |

## Operational Notes

- The scan is transformed into `scan_base_frame` using TF before clearance is
  evaluated. This avoids assuming the laser frame is perfectly aligned with
  `base_link`.
- If scan data is missing, stale, or cannot be transformed, the second phase keeps
  the odom-based recovery direction.
- If both front and rear sectors are clear after the first phase, the behavior
  exits successfully without running the second phase.
- This behavior clears costmaps as a recovery aid. It does not correct global
  localization drift or map-to-odom errors caused by wheel slip.

## Expected Logs

On startup:

```text
AggressiveBackUp configured: default_distance=..., scan_topic=..., enable_second_phase=...
```

On recovery:

```text
AggressiveBackUp TRIGGERED: ...
AggressiveBackUp phase first_odom_escape: ...
Requested costmap clear: local_costmap/clear_entirely_local_costmap
AggressiveBackUp scan decision: front_clearance=..., rear_clearance=...
AggressiveBackUp phase second_scan_escape: ...
```

If the first phase is enough:

```text
AggressiveBackUp completed after first phase: ...
```

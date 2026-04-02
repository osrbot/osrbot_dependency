[中文](./README_ZH.md) | **English**

# osrbot_dependency

A collection of ROS2 dependency packages for the OSR robot navigation system. This repository consolidates the key ROS2 components required to run the OSR robot, including a LiDAR driver, SLAM mapping, and a local path planner.

---

## 📦 Repository Structure

```
osrbot_dependency/
├── Lakibeam_ROS2_Driver/   # ROS2 driver for the LakiBeam series LiDAR
├── ros2_gmapping/          # GMapping SLAM adapted for ROS2
└── teb_local_planner/      # TEB local path planner (ROS2 Humble)
    ├── teb_local_planner/  # TEB core planning algorithm package
    ├── costmap_converter/  # Costmap conversion plugin package
    └── teb_msgs/           # TEB custom message package
```

---

## 🧩 Module Overview

### 1. Lakibeam_ROS2_Driver — LiDAR Driver

> Source: [RichbeamTechnology/Lakibeam_ROS2_Driver](https://github.com/RichbeamTechnology/Lakibeam_ROS2_Driver)

The ROS2 driver for the LakiBeam LiDAR sensor, supporting the **LakiBeam1 / LakiBeam1S / LakiBeam1L** series. The driver parses point cloud data from incoming UDP packets and publishes it on the `/scan` (LaserScan) or `/pcd` (PointCloud2) topics.

**Supported Environments:**
- Ubuntu 20.04 + ROS2 Foxy
- Ubuntu 22.04 + ROS2 Humble

**Key Configurable Parameters:**

| Parameter | Description |
|-----------|-------------|
| `hostip` | Target PC IP address (`0.0.0.0` to listen on all interfaces) |
| `port` | Listening port number |
| `inverted` | Whether the sensor is mounted upside-down |
| `angle_offset` | Point cloud rotation offset around the Z-axis |
| `scanfreq` | Scan frequency (10/20/25/30 Hz) |
| `scan_range_start/stop` | Scan angle range (45°~315°) |

**Quick Start:**
```bash
colcon build
source ./install/setup.bash
ros2 launch lakibeam1 lakibeam1_scan_view.launch.py
```

---

### 2. ros2_gmapping — SLAM Mapping

> Reference: [Project-MANAS/slam_gmapping](https://github.com/Project-MANAS/slam_gmapping)

A ROS2 port of the classic OpenSLAM GMapping algorithm, with the following improvements over the original ROS2 adaptation:
- Dynamic parameter handling
- Bug fix for the map update frequency setting

**Required Topics:** `/scan` (LiDAR), `/odom` (Odometry)

**Quick Start:**
```bash
colcon build --packages-select openslam_gmapping slam_gmapping
source ./install/setup.bash
ros2 launch slam_gmapping slam_gmapping.launch.py
```

**Key Configuration Parameters** (`slam_gmapping/params/slam_gmapping.yaml`):

| Parameter | Description |
|-----------|-------------|
| `base_frame` | Robot base frame |
| `map_frame` | Map frame |
| `odom_frame` | Odometry frame |
| `use_sim_time` | Enable simulated clock (set `true` in simulation environments) |

---

### 3. teb_local_planner — TEB Local Planner

> Adapted for: ROS2 Humble

A ROS2 Humble adaptation of the Timed Elastic Band (TEB) local path planner, integrated with the Nav2 framework. Contains three sub-packages:

#### 3.1 `teb_local_planner`
Implements the core TEB planning algorithm as a Nav2 local planner plugin. Supports multiple kinematic models including differential-drive, omnidirectional, and Ackermann.

#### 3.2 `costmap_converter`
A collection of costmap conversion plugins that transform occupied cells from `costmap2d` into geometric primitives (points, lines, polygons) for use by the TEB planner.

- **License:** BSD (some components depend on `MultitargetTracker`, which is GPLv3)

#### 3.3 `teb_msgs`
Custom ROS2 message definitions for the TEB planner.

---

## 🚀 Building Everything

After cloning this repository into your ROS2 workspace, run:

```bash
cd ~/ros2_ws
# Place the contents of this repository under the src/ directory
colcon build
source ./install/setup.bash
```

---

## 🔧 System Dependencies

- **ROS2 Humble** (Ubuntu 22.04 recommended)
- `nav2` navigation framework (required by teb_local_planner)
- `pcl`, `pcl_conversions` (required by the LiDAR driver)

---

## 📄 License

Each sub-package follows its original license agreement. Please refer to the `LICENSE` file within each subdirectory for details.

# osrbot_dependency

OSR 机器人导航依赖包集合。本仓库汇集了 OSR 机器人系统运行所需的关键 ROS2 依赖组件，包括激光雷达驱动、SLAM 建图和局部规划器等核心模块。

---

## 📦 仓库结构

```
osrbot_dependency/
├── Lakibeam_ROS2_Driver/   # LakiBeam 系列激光雷达 ROS2 驱动
├── ros2_gmapping/          # 适配 ROS2 的 GMapping SLAM 建图
└── teb_local_planner/      # TEB 局部路径规划器（ROS2 Humble）
    ├── teb_local_planner/  # TEB 核心规划算法包
    ├── costmap_converter/  # 代价地图转换插件包
    └── teb_msgs/           # TEB 自定义消息包
```

---

## 🧩 模块说明

### 1. Lakibeam_ROS2_Driver — 激光雷达驱动

> 来源：[RichbeamTechnology/Lakibeam_ROS2_Driver](https://github.com/RichbeamTechnology/Lakibeam_ROS2_Driver)

ROS2 版 LakiBeam 激光雷达驱动，支持 **LakiBeam1 / LakiBeam1S / LakiBeam1L** 系列传感器。驱动通过监听 UDP 数据包解析点云，并以 `/scan`（LaserScan）或 `/pcd`（PointCloud2）话题发布。

**支持环境：**
- Ubuntu 20.04 + ROS2 Foxy
- Ubuntu 22.04 + ROS2 Humble

**主要可配置参数：**

| 参数 | 说明 |
|------|------|
| `hostip` | 目标 PC IP 地址（0.0.0.0 监听所有） |
| `port` | 监听端口号 |
| `inverted` | 是否倒置安装传感器 |
| `angle_offset` | 点云绕 Z 轴旋转偏移 |
| `scanfreq` | 扫描频率（10/20/25/30 Hz） |
| `scan_range_start/stop` | 扫描角度范围（45°~315°） |

**快速启动：**
```bash
colcon build
source ./install/setup.bash
ros2 launch lakibeam1 lakibeam1_scan_view.launch.py
```

---

### 2. ros2_gmapping — SLAM 建图

> 参考：[Project-MANAS/slam_gmapping](https://github.com/Project-MANAS/slam_gmapping)

基于经典 OpenSLAM GMapping 算法的 ROS2 移植版本，在原 ROS2 适配版本的基础上作了以下优化：
- 参数动态化处理
- 修复地图更新频率设置 BUG

**依赖话题：** `/scan`（激光雷达）、`/odom`（里程计）

**快速启动：**
```bash
colcon build --packages-select openslam_gmapping slam_gmapping
source ./install/setup.bash
ros2 launch slam_gmapping slam_gmapping.launch.py
```

**关键配置参数**（`slam_gmapping/params/slam_gmapping.yaml`）：

| 参数 | 说明 |
|------|------|
| `base_frame` | 机器人基坐标系 |
| `map_frame` | 地图坐标系 |
| `odom_frame` | 里程计坐标系 |
| `use_sim_time` | 仿真时钟开关（仿真环境设 `true`） |

---

### 3. teb_local_planner — TEB 局部规划器

> 适配版本：ROS2 Humble

Timed Elastic Band（TEB）局部路径规划器的 ROS2 Humble 适配版本，与 Nav2 框架集成使用。包含三个子包：

#### 3.1 `teb_local_planner`
TEB 核心规划算法实现，作为 Nav2 的局部规划器插件运行，支持差速、全向、阿克曼等多种运动学模型。

#### 3.2 `costmap_converter`
代价地图转换插件集，将 `costmap2d` 的占用栅格单元转换为几何基元（点、线、多边形），供 TEB 规划器使用。

- **许可证：** BSD（部分依赖 `MultitargetTracker` 为 GPLv3）

#### 3.3 `teb_msgs`
TEB 规划器的自定义 ROS2 消息定义包。

---

## 🚀 整体构建

在 ROS2 工作空间下克隆本仓库后执行：

```bash
cd ~/ros2_ws
# 将本仓库内容放置于 src/ 目录下
colcon build
source ./install/setup.bash
```

---

## 🔧 系统依赖

- **ROS2 Humble** (Ubuntu 22.04 推荐)
- `nav2` 导航框架（teb_local_planner 依赖）
- `pcl`、`pcl_conversions`（激光雷达驱动依赖）

---

## 📄 许可证

各子包遵循其原始许可证协议，详见各子目录内的 `LICENSE` 文件。

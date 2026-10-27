# navbot — 差速移动机器人自主导航仿真系统

![ROS 2](https://img.shields.io/badge/ROS_2-Humble-blue?logo=ros) ![Gazebo](https://img.shields.io/badge/Gazebo-Classic11-blue?logo=ros&logoColor=white) ![Nav2](https://img.shields.io/badge/Nav2-Humble-brightgreen?logo=ros) ![Python](https://img.shields.io/badge/Python-3.10-yellow?logo=python) ![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-orange?logo=ubuntu)

navbot 是一个基于 **ROS 2 Humble**、**Gazebo Classic 11** 与 **Nav2** 的室内差速机器人自主导航仿真项目：机器人模型完全由 primitive 几何搭建（不含任何外部网格文件），惯量与轮距由几何公式在 xacro 内推导；slam_toolbox 建图、AMCL 定位、Nav2 规划控制、多航点巡检四层串成一条完整链路，模型层 / 仿真层 / 任务层三包解耦，并配套 12 个可复验的诊断脚本。

## 项目概述

整个系统按「模型 → 仿真 → 建图 → 定位 → 导航 → 任务」流水线构建：`navbot_description` 提供底盘、双驱动轮、前万向球与三类传感器（2D 激光雷达、相机、IMU）的 xacro 与 Gazebo 插件配置；`navbot_gazebo` 提供 30.18 × 50.22 m 仓库场景（基于 nav2 官方 warehouse 地图反推）、Nav2 / EKF / slam_toolbox 参数与 `sim / slam / navigate / verify` 四条 launch 链；`navbot_patrol` 基于 `nav2_simple_commander.BasicNavigator` 执行多航点巡检，航点与超时全部来自 YAML。TF 采用「每条边单一发布者」设计：`odom → base_footprint` 由 EKF 或差速插件二选一，`map → odom` 只由定位节点发布。

**核心特性：**

- **零外部资产**：底盘 / 轮 / 雷达 / 相机 / IMU 全部用 box、cylinder、sphere 表达，无 `.dae`，无许可证牵连。
- **几何驱动参数**：惯量由 `m/12(a²+b²)`、`½mr²`、`2/5mr²` 在 xacro 内推导；轮心高度由 `wheel_radius - base_z` 反推，改轮径自动保持触地；Nav2 的 `robot_radius`、膨胀半径、加减速度上限全部由模型尺寸算出并写明换算式。
- **TF 单一所有权**：`publish_odom_tf` 做成 xacro 参数，launch 探测 `robot_localization` 是否安装自动切换发布方，工程内不存在任何静态 `map→odom`。
- **传感器真实建模**：540 线 ±157°、量程 0.15–8.0 m、σ=10 mm 高斯测距噪声的雷达 + 200 Hz IMU，使 EKF 有真实输入而非空转。
- **任务层配置化**：10 点 / 105.03 m 航线写在 `waypoints.yaml`，换现场不改代码；服务就绪上限、任务总超时取消、结果分类、非 0 退出码，便于脚本与 CI 判定。
- **可复验**：TF 唯一父帧、话题与发布者数量、轮距标定拟合、端到端联调各有专用脚本，README 每个数字都能现场复跑。
- **一条命令拉起**：`sim / slam / navigate / patrol` 四条 launch 复用同一个仿真底座，不重复维护。

**关键词：** ROS 2、Nav2、slam_toolbox、robot_localization、Gazebo、URDF/xacro、差速里程计、TF、自主导航、航点巡检、WSL2

## 系统架构

```
                  ┌────────────────────────────────────────────────────┐
  仓库场景 ──▶   │  Gazebo Classic 11（navbot_gazebo/worlds/warehouse.world）│
                  │  · 30.18×50.22 m，30 个实体，南墙开口 7.4 m            │
                  │  · 4 根大货架 + 21 个护栏/立柱，几何由地图反推         │
                  └──────────────────┬─────────────────────────────────┘
                                     │ /odom /scan /imu/data  ← 差速插件 + 传感器插件
                                     ▼
                  ┌────────────────────────────────────────────────────┐
                  │  navbot_description（模型层）                       │
                  │  · 底盘 0.320×0.240×0.060，轮距 0.196，轮径 0.072   │
                  │  · 惯量由几何公式推导；TF：base_footprint→base_link │
                  │    →{轮/万向球/scanner_link/camera_link/imu_link}   │
                  └──────────────────┬─────────────────────────────────┘
                                     │ odom(轮速) + IMU(姿态/角速度)
                                     ▼
                  ┌────────────────────────────────────────────────────┐
                  │  ekf_filter_node（robot_localization）              │
                  │  · 只融合 vx / yaw_rate + 姿态，two_d_mode          │
                  │  · 独占 odom → base_footprint 的 TF 发布            │
                  └──────────────────┬─────────────────────────────────┘
                          建图期      │       导航期
              ┌──────────────────────┴───────────────────────┐
              ▼                                              ▼
   ┌────────────────────────┐                  ┌────────────────────────────┐
   │ slam_toolbox（sync）   │                  │ map_server + AMCL          │
   │ 发布 map → odom        │                  │ 发布 map → odom            │
   │ → scripts/save_map.sh  │                  │ /map /amcl_pose /initialpose│
   └────────────────────────┘                  └─────────────┬──────────────┘
                                                             │ NavigateToPose / FollowWaypoints
                                                             ▼
                  ┌────────────────────────────────────────────────────┐
                  │  Nav2 栈：bt_navigator → planner(NavFN+A*)          │
                  │  → controller(DWB) → velocity_smoother              │
                  │  local/global costmap（robot_radius 0.21）          │
                  └──────────────────┬─────────────────────────────────┘
                                     │ /cmd_vel
                                     ▼
                  ┌────────────────────────────────────────────────────┐
                  │  navbot_patrol（任务层，Python）                    │
                  │  读 waypoints.yaml → setInitialPose → followWaypoints│
                  │  → 进度反馈 → 超时取消 → 结果分类与退出码           │
                  └────────────────────────────────────────────────────┘
```

## 目录结构

```
navbot/
├── config/
│   └── fastdds_wsl.xml                   # WSL2 专用 DDS 传输（关 SHM，不限接口）
├── scripts/
│   ├── env.sh                            # source ROS + 工作区 + 导出 DDS 配置
│   ├── build.sh                          # 修执行位 + colcon build
│   ├── kill_sim.sh                       # 清孤儿进程 / gzmaster / SHM 锁
│   ├── reset_dds.sh                      # 复位 FastDDS 共享内存状态
│   ├── check_system.sh                   # 话题/发布者数量、TF 归属、服务端体检
│   ├── tf_probe.py                       # 按仿真时钟查 TF，校验唯一父帧
│   ├── gen_map_from_world.py             # 按场景坐标生成栅格地图（跳过 SLAM）
│   ├── save_map.sh                       # map_saver_cli 绝对路径存图
│   ├── measure_sim_speed.sh              # 按传感器组合量仿真实时倍率
│   ├── verify_odometry.py                # 轮距标定拟合（已并入包内 tools/）
│   ├── selftest.sh                       # 健康检查 + 标定两阶段自测
│   └── nav_check.sh                      # 地图→AMCL→Nav2→巡检受理 端到端联调
└── src/
    ├── navbot_description/               # 模型层（ament_cmake）
    │   ├── urdf/
    │   │   ├── navbot.urdf.xacro         # 入口，4 个可传参数
    │   │   ├── navbot_constants.xacro    # 几何/质量常量 + 惯量推导宏
    │   │   ├── navbot_base.xacro         # 底盘、双驱动轮、前万向球
    │   │   ├── navbot_sensors.xacro      # 雷达、相机、IMU 及其 Gazebo 插件
    │   │   └── navbot_drivetrain.xacro   # 差速驱动插件与接触/摩擦参数
    │   ├── launch/
    │   │   ├── display.launch.py         # 仅 RViz 看模型
    │   │   └── gazebo.launch.py          # 空世界跑模型与传感器
    │   └── rviz/model.rviz
    ├── navbot_gazebo/                    # 仿真与配置层（ament_cmake）
    │   ├── worlds/warehouse.world        # 30.18×50.22 m 仓库（由地图反推）
    │   ├── worlds/lab.world              # 12×9 m 实验室（备用场景）
    │   ├── launch/
    │   │   ├── sim.launch.py             # 唯一「起仿真」入口
    │   │   ├── slam.launch.py            # = sim + slam_toolbox
    │   │   ├── navigate.launch.py        # = sim + AMCL + Nav2
    │   │   └── verify.launch.py          # = sim + 标定节点（同一会话）
    │   ├── params/nav2_params.yaml       # 由模型几何推导
    │   ├── config/
    │   │   ├── ekf.yaml                  # odom + IMU 融合
    │   │   ├── slam.yaml                 # slam_toolbox 建图参数
    │   │   └── sim.rviz / nav.rviz       # 无图 / 有图两套 RViz 配置
    │   ├── maps/nav2_warehouse/          # nav2 官方 warehouse 地图（默认）
    │   ├── maps/lab_map.pgm|.yaml        # 实验室地图（备用）
    │   └── tools/verify_odometry.py      # 轮距标定校验节点
    └── navbot_patrol/                    # 任务层（ament_python）
        ├── config/waypoints.yaml         # 航点、超时、初始位姿
        ├── launch/patrol.launch.py       # 注入 use_sim_time
        └── navbot_patrol/patrol_node.py  # 多航点巡航节点
```

## 命名体系

| 类别 | 命名规则 | 示例 |
|------|----------|------|
| 功能包 | `navbot_*` | `navbot_description` / `navbot_gazebo` / `navbot_patrol` |
| TF 帧树 | `map → odom → base_footprint → base_link → 部件` | 部件：`left/right_wheel_link`、`caster_link`、`scanner_link → scanner_frame`、`camera_link → camera_optical_frame`、`imu_link` |
| 关节 | `<部位>_joint`；驱动轮 `<side>_wheel_joint` | `base_joint` / `left_wheel_joint` / `scanner_beam_joint` / `camera_optical_joint` |
| 核心话题 | `/odom`（nav_msgs/Odometry）、`/scan`、`/imu/data`、`/cmd_vel` | 由差速插件与传感器插件发布，话题名固定不带命名空间 |
| 定位话题 | `/map`、`/amcl_pose`、`/initialpose`、`/plan`、`/local_plan` | map_server / AMCL / bt_navigator |
| 动作接口 | `/navigate_to_pose`、`/follow_waypoints` | `nav2_simple_commander.BasicNavigator` 封装 |
| 仿真实体 | `navbot` | `spawn_entity.py -entity navbot` |
| 场景与地图 | `worlds/warehouse.world` ↔ `maps/nav2_warehouse/warehouse.{pgm,yaml}` | 坐标一一对应，出生点在世界原点 |
| 巡检配置 | `config/waypoints.yaml` 的 `initial_pose / task / waypoints` | 航点字段 `name x y yaw_deg` |

## 环境依赖

- **操作系统**：Ubuntu 22.04（亦验证于 Windows WSL2 Ubuntu 22.04 + WSLg）。
- **ROS 2**：Humble Hawksbill。
- **Gazebo**：Classic 11（`libgazebo_ros_*` 插件体系）。
- **主要依赖**：

```bash
sudo apt install -y \
  ros-humble-nav2-bringup \
  ros-humble-nav2-simple-commander \
  ros-humble-slam-toolbox \
  ros-humble-robot-localization \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-rviz2 \
  ros-humble-teleop-twist-keyboard \
  python3-colcon-common-extensions
```

> 未安装 `robot_localization` 也能跑：launch 会自动探测并把 `odom→base_footprint` 的 TF 发布权交给差速插件。

## 安装步骤

1. **创建工作区并放入源码**：

```bash
mkdir -p ~/navbot_ws/src
cp -r <此仓库>/src/* ~/navbot_ws/src/
cp -r <此仓库>/scripts <此仓库>/config ~/navbot_ws/
```

> **WSL2 注意**：不要在 `/mnt/c`（NTFS 挂载）下编译，CMake 编译器自检会报错；请把工程拷入 WSL 原生文件系统（如 `~/navbot_ws`）后再编。另外从 Windows 侧写过的文件可能丢失可执行位，`scripts/build.sh` 会自动 `chmod +x`。

2. **安装依赖**：

```bash
cd ~/navbot_ws
rosdep install --from-paths src --ignore-src -r -y
```

未装 rosdep 时，按上面 apt 列表手动安装即可。

3. **编译**：

`scripts/build.sh` 等价于 `colcon build --symlink-install`，并额外修正 Windows 侧写入丢失的可执行位。

```bash
source /opt/ros/humble/setup.bash
cd ~/navbot_ws
./scripts/build.sh
source install/setup.bash
```

4. **WSL2 用户追加：禁用 FastDDS 共享内存**（原生 Ubuntu 可跳过）：

WSL2 下被 `kill -9` 打断的节点会在 `/dev/shm` 留下 `fastrtps_portXXXX` 锁，之后新进程出现「话题能列出、订阅收不到数据」。仓库自带 `config/fastdds_wsl.xml` 只禁用 SHM，导出工作由 `scripts/env.sh` 负责，无需手工设置：

```bash
source scripts/env.sh
echo "$FASTDDS_DEFAULT_PROFILES_FILE"
```

第二行输出应指向 `config/fastdds_wsl.xml`。

> **不要**给 UDP 传输加 `<interfaceWhiteList>127.0.0.1`：WSL2 的 `lo` 不支持组播，写死回环会破坏 SPDP 发现，反而彻底收不到数据。只关 SHM、接口保持默认。

## 运行

### 方式一：一键启动导航 + 巡检

在终端 1 起仿真 + AMCL + Nav2，等 Nav2 生命周期节点就绪后，再在终端 2 拉起巡检：

```bash
source ~/navbot_ws/install/setup.bash
ros2 launch navbot_gazebo navigate.launch.py
ros2 launch navbot_patrol patrol.launch.py
```

启动后应看到：Gazebo 中出现机器人与仓库场景，RViz 以 `map` 为参考系显示栅格地图、代价地图与全局路径，巡检终端打印 `航点 10 个，航线 105.03 m` 与逐点进度。

### 方式二：自己建图（不用仓库里的离线地图）

终端 1 起建图，终端 2 用键盘遥控走遍机器人的三个区域，终端 3 存图：

```bash
ros2 launch navbot_gazebo slam.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
~/navbot_ws/scripts/save_map.sh
```

存图后 `navigate.launch.py` 会自动改用新地图。不想跑建图，可直接按场景坐标出图：

```bash
./scripts/gen_map_from_world.py && ./scripts/build.sh
```

### 方式三：仅启动仿真预览（不起 Nav2）

前者起场景 + 机器人 + RViz，后者只用 RViz 看模型与 TF：

```bash
ros2 launch navbot_gazebo sim.launch.py
ros2 launch navbot_description display.launch.py
```

### 分步调试（推荐第一次运行时）

```bash
xacro src/navbot_description/urdf/navbot.urdf.xacro | grep -E "wheel_separation|wheel_diameter"
ros2 topic hz /odom
ros2 topic echo /scan --once --field header
./scripts/tf_probe.py 'map->odom' 'odom->base_footprint' 'base_footprint->base_link'
./scripts/check_system.sh
ros2 launch navbot_gazebo verify.launch.py
```

依次核对：xacro 展开后的轮距与轮径；`/odom`、`/scan` 是否在出数；TF 每条边的唯一父帧（`tf_probe.py` 按仿真时钟查询，`tf2_echo` 用系统时钟会误报 `frame does not exist`）；话题数量、EKF 输入与 Nav2 服务端体检；最后是轮距标定，驱动转圈后拟合 `yaw_odom / yaw_true`，斜率应接近 1.0。

## 调参速查

| 参数 | 位置 | 默认 | 说明 |
|------|------|------|------|
| 底盘尺寸 | `navbot_constants.xacro` | 0.320×0.240×0.060 m | 决定 `robot_radius` 与外接圆 |
| 轮径 / 轮距 | `navbot_constants.xacro` | r=0.036 / track=0.196 m | 插件值由常量推导，改一处即可 |
| 各部件质量 | `navbot_constants.xacro` | 整机 2.235 kg | 惯量自动重算 |
| `publish_odom_tf` | xacro 参数 | `false` | 谁发布 `odom→base_footprint`；`use_ekf:=false` 时自动置 true |
| `enable_camera` / `enable_lidar` | launch 参数 | true | 关掉渲染型传感器可显著加快 headless 步进 |
| `lidar_visualize` | launch 参数 | false | 在 Gazebo 中画出射线 |
| `headless` | launch 参数 | false | 不起 gzclient（无图形环境） |
| `spawn_timeout` | launch 参数 | 240 s | 等待 `/spawn_entity` 服务，慢机器需放宽 |
| `robot_radius` | `params/nav2_params.yaml` | 0.21 m | 外接圆 √(0.16²+0.12²)=0.20，相机前伸 0.205 |
| `inflation_radius` | `params/nav2_params.yaml` | 0.45 m | 场景最窄通道约 4.95 m（S3 与 S4 之间），余量充裕 |
| `max_vel_x / min_vel_x` | `params/nav2_params.yaml` | 0.30 / −0.12 m/s | 允许倒车脱困 |
| `acc_lim_x` | `params/nav2_params.yaml` | 0.5 m/s² | 上限来自插件 `max_wheel_acceleration 15 × 0.036 = 0.54` |
| `laser_max_range` | `params/nav2_params.yaml` | 7.5 m | 与雷达量程 8.0 m 留余量，避免边界光束干扰 |
| `z_hit / z_rand` | `params/nav2_params.yaml` | 0.9 / 0.03 | 原来 0.5 / 0.5 会让粒子在大场景里发散 |
| `sigma_hit` | `params/nav2_params.yaml` | 0.12 m | 地图 0.03 m 分辨率下提高打分明锐度 |
| `max_beams` | `params/nav2_params.yaml` | 90 | 540 线雷达，过稀会导致匹配不足 |
| `min/max_particles` | `params/nav2_params.yaml` | 800 / 4000 | 30×50 m 场景需更多粒子 |
| `xy_goal_tolerance` | `params/nav2_params.yaml` | 0.20 m | 到点判定容差 |
| EKF `odom0_config` | `config/ekf.yaml` | 仅 vx + yaw_rate | 差速车 vy 不可测，不要喂位姿 |
| EKF `imu0_relative` | `config/ekf.yaml` | true | 以开机第一帧为零点，避免把仿真绝对航向当真值 |
| 建图分辨率 | `config/slam.yaml` | 0.05 m | 与 `nav2_params` 的 costmap 分辨率保持一致 |
| 航点与超时 | `navbot_patrol/config/waypoints.yaml` | 10 点 / 105.03 m，`timeout_sec 480` | `--waypoints` > `NAVBOT_WAYPOINTS` > 包内默认 |
| `task.loop` | 同上 | 1 | 整条航线重复轮数 |

## 验证状态

**已在 WSL2 Ubuntu 22.04 + ROS 2 Humble + Gazebo Classic 11 实跑验证**：

| 项 | 结果 |
|---|---|
| 三包 `colcon build` | 全绿（5.53 s） |
| 场景加载 | `gzserver` 加载 `warehouse.world`，机器人成功 spawn |
| Nav2 | 11 个进程，`/amcl` 与 `/bt_navigator` 均 `active [3]` |
| 话题 / 动作 | 6 个关键话题 + 2 个动作服务端全部在位 |
| `/scan` | 9.96 Hz，540 线 ±157°，量程 0.15–8.0 m，有效 195 束 |
| AMCL 跟踪 | 误差 0.263 m，σx 0.250，**PASS** |
| **完整巡检** | **10/10 航点全到达，105.03 m，440.3 s，0 失败** |
| 终点回位精度 | 误差 0.135 m（容差 0.20 m） |

两次独立复跑结果一致（434.4 s / 0.121 m 与 440.3 s / 0.135 m）。

**未验证**：带窗口（gzclient）模式、`slam.launch.py` 建图链路、`verify.launch.py` 轮距标定、`selftest.sh` 阶段 2、多轮循环（`task.loop > 1`）、相机链路（本次以 `enable_camera:=false` 启动）。新场景的完整参数与校验数据见 `navbot_gazebo/WAREHOUSE_SCENE.md`。

## 常见问题

- **`Service /spawn_entity unavailable. Was Gazebo started with GazeboRosFactory?`**：不是插件没加载，而是 gzserver 冷启动慢于 `spawn_entity.py` 默认 30 s 等待。用 `-p spawn_timeout:=480` 放宽。
- **Gazebo 窗口一开就卡住、机器人不出现**：`/etc/profile.d` 里若设了 `LIBGL_ALWAYS_SOFTWARE=0`（走 Mesa D3D12/GPU-PV），gzclient 会饿死 gzserver 的 ROS 服务线程。本工程 launch 在带窗口时自动强制软件渲染，无需手工 export。
- **话题能 `ros2 topic list` 到，但订阅收不到数据（WSL2）**：FastDDS SHM 端口锁泄漏，执行 `./scripts/reset_dds.sh` 或 `./scripts/kill_sim.sh`；并确认发布端与订阅端 source 的是同一份 `scripts/env.sh`。
- **`tf2_echo` 报 `frame does not exist`**：`tf2_echo` 用系统时间查询，而仿真时间只走了几十秒。改用 `./scripts/tf_probe.py`（按仿真时钟查询，且采满时间窗，否则只会看到静态边）。
- **RViz 报 `Frame [map] does not exist`**：`sim.launch.py` 阶段没有定位节点，`map` 帧本就不存在。默认配置已改用 `odom`；若你在 `navigate.launch.py` 下看到该报错，说明地图文件缺失（跑 `./scripts/gen_map_from_world.py` 或先建图）。
- **Nav2 报 `planner lagging` / EKF `Frequency too low`**：期望频率按实时仿真设定，而当前步进倍率低于实时所致，不是配置错误。
- **`pkill -f "gzserver"` 把终端自己杀了**：`wsl.exe -e bash -lc '...'` 内联写这种 pattern 会匹配到调用者自身的命令行。清理逻辑请走 `./scripts/kill_sim.sh`（pattern 锚定安装路径并跳过自身与祖先进程）。
- **`ros2 launch ... use_composition:=true` 报 `NameError: name 'true' is not defined`**：nav2 的 launch 会 `eval()` 该参数值，需要 Python 字面量。本工程的 `_nav_bool()` 已在生成期归一，`true/True/1/false/False/0` 都可用。

## License

MIT，见 `LICENSE`。场景中所有几何、参数、脚本与文档均为本项目自定义，不含第三方模型或网格文件。

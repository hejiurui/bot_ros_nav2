# navbot —— 差速移动机器人自主导航仿真工程

ROS 2 Humble + Nav2 + slam_toolbox + robot_localization + Gazebo 11。
从零设计的差速底盘模型（全 primitive 几何，不含任何外部网格文件）、实验室场景、
建图/定位/导航三条 launch 链，以及多航点巡检任务层与一套可复验的诊断脚本。

| 项 | 值 |
|---|---|
| 许可 | MIT（`LICENSE`） |
| 平台 | Ubuntu 22.04（含 WSL2）、ROS 2 Humble、Gazebo classic 11 |
| 依赖 | `ros-humble-nav2-bringup`、`slam-toolbox`、`robot-localization`、`gazebo-ros-pkgs`、`teleop-twist-keyboard` |

---

## 1. 包结构

```
src/
├── navbot_description/     模型层
│   ├── urdf/
│   │   ├── navbot.urdf.xacro        入口，4 个可传参数
│   │   ├── navbot_constants.xacro   几何/质量常量 + 惯量推导宏
│   │   ├── navbot_base.xacro        底盘、双驱动轮、前万向球
│   │   ├── navbot_sensors.xacro     激光雷达、相机、IMU
│   │   └── navbot_drivetrain.xacro  Gazebo 差速驱动插件与接触参数
│   ├── launch/display.launch.py     仅 RViz 看模型
│   ├── launch/gazebo.launch.py      空世界跑模型与传感器
│   └── rviz/model.rviz
│
├── navbot_gazebo/          仿真与配置层
│   ├── worlds/lab.world             12 × 9 m 实验室场景（手写显式坐标）
│   ├── launch/sim.launch.py         唯一"起仿真"入口，其余 launch 复用它
│   ├── launch/slam.launch.py        = sim + slam_toolbox
│   ├── launch/navigate.launch.py    = sim + AMCL + Nav2
│   ├── launch/verify.launch.py      = sim + 标定节点（同一会话）
│   ├── params/nav2_params.yaml      由模型几何推导
│   ├── config/ekf.yaml              odom + IMU 融合
│   ├── config/slam.yaml             slam_toolbox 建图参数
│   ├── config/{sim,nav}.rviz
│   ├── maps/lab_map.{pgm,yaml}      脚本按场景坐标生成
│   └── tools/verify_odometry.py     轮距标定校验
│
└── navbot_patrol/          任务层
    ├── config/waypoints.yaml        航点、超时、初始位姿
    ├── navbot_patrol/patrol_node.py 多航点巡航节点
    └── launch/patrol.launch.py
```

依赖方向单向：`navbot_patrol → Nav2 接口`，`navbot_gazebo → navbot_description`。
任务层不知道模型长什么样，只发 `PoseStamped`。

---

## 2. 快速开始

```bash
mkdir -p ~/navbot_ws && git clone <此仓库> ~/navbot_ws/src   # 或直接把 src/ 放进工作区
cd ~/navbot_ws
./scripts/build.sh
source install/setup.bash

ros2 launch navbot_description display.launch.py          # ① 看模型
ros2 launch navbot_gazebo sim.launch.py                   # ② 进场景
ros2 launch navbot_gazebo slam.launch.py                  # ③ 建图
ros2 run teleop_twist_keyboard teleop_twist_keyboard      #    遥控走一圈
./scripts/save_map.sh                                     #    存图
ros2 launch navbot_gazebo navigate.launch.py              # ④ 定位 + 导航
ros2 launch navbot_patrol patrol.launch.py                # ⑤ 巡检
```

不想跑建图，直接按场景坐标出图：

```bash
./scripts/gen_map_from_world.py && ./scripts/build.sh
```

调试开关：

| 参数 | 作用 |
|---|---|
| `headless:=true` | 不起 gzclient（无图形环境时用） |
| `enable_camera:=false` | 关掉相机（软件渲染环境里的主要开销） |
| `enable_lidar:=false` | 关掉激光雷达，只留物理与 TF |
| `use_ekf:=false` | 由差速插件发布 odom TF，不用 EKF |
| `spawn_timeout:=480` | 等待 `/spawn_entity` 的秒数 |
| `map_file_path:=/绝对路径.yaml` | 换地图 |
| `use_composition:=True` | Nav2 节点合并进组件容器 |

---

## 3. 模型设计

### 3.1 几何（`navbot_constants.xacro`，全部自定义）

| 项 | 值 | 说明 |
|---|---|---|
| 底盘 | 0.320 × 0.240 × 0.060 m | 中心离地 `base_z = 0.065` |
| 驱动轮 | 半径 0.036、宽 0.024 m | 轮心 y = ±0.098 → **轮距 0.196 m** |
| 驱动轴位置 | x = −0.040 m | 轴在几何中心后方，前部留给万向球 |
| 前万向球 | 半径 0.018 m，x = +0.120 | 摩擦系数 0，等效随动轮 |
| 激光雷达 | 半径 0.035、高 0.042，光面离地 0.129 m | x = +0.060 |
| 相机 | 0.090 × 0.025 × 0.025 m | 光心 x = +0.160，离地 0.110 |
| IMU | 0.020 × 0.020 × 0.006 m | 放在底盘中心下方，杠杆臂最小 |
| 整机质量 | 2.235 kg | 各部件质量之和 |

轮心 z 由 `wheel_radius - base_z` 反推，因此**改轮径就自动保持触地**，
不会出现"轮子悬空/陷地"这类 URDF 常见错误。

### 3.2 惯量

不抄 CAD 报表，直接在 xacro 里由几何公式推导：

```
长方体  I = m/12 · (a² + b²)
圆柱    轴向 I = ½mr²，径向 I = m(3r² + l²)/12
球      I = 2/5 mr²
```

因此改任何一个尺寸或质量，惯量自动跟着变，物理行为始终自洽。

### 3.3 传感器参数

| 传感器 | 参数 |
|---|---|
| 激光雷达 | 540 线、±157°、量程 0.15–8.0 m、10 Hz、高斯测距噪声 σ=10 mm |
| IMU | 200 Hz，世界系姿态（`initialOrientationAsReference=false`） |
| 相机 | 640×480、70° 水平视场、15 Hz |

雷达量程 8 m 与场景尺寸（12 × 9 m）匹配：单点能看到对墙，扫描匹配有足够约束。

---

## 4. TF 设计：每条边只有一个发布者

```
map ──(slam_toolbox 建图时 / AMCL 导航时)── odom ──(EKF 或差速插件，二选一)── base_footprint ── base_link ──┬─ left/right_wheel_link
                                                                                                             ├─ caster_link
                                                                                                             ├─ scanner_link ── scanner_frame
                                                                                                             ├─ camera_link ── camera_optical_frame
                                                                                                             └─ imu_link
```

规则写进模型而不是靠 launch 打补丁：

- `odom → base_footprint` 由 xacro 参数 `publish_odom_tf` 控制，默认 `false`（EKF 持有）。
  `sim.launch.py` 探测 `robot_localization` 是否安装：装了就把发布权给 EKF，
  没装则自动把该参数置 `true` 让差速插件顶替。**任何时刻不会出现两个发布者。**
- `map → odom` 只由 slam_toolbox（建图）或 AMCL（导航）发布，工程内不存在任何静态 `map→odom`。
- 全栈 `base_frame` 统一为 `base_footprint`（AMCL、bt_navigator、两个 costmap、behavior_server、EKF）。

---

## 5. 定位与导航参数依据

`params/nav2_params.yaml` 里与几何相关的数值全部由模型反推，不用模板默认值：

| 参数 | 值 | 依据 |
|---|---|---|
| `robot_radius` | 0.21 m | 底盘外接圆 √(0.16²+0.12²)=0.20，相机前伸到 0.205 → 取 0.21 |
| `inflation_radius` | 0.45 m | ≈ 2× 半径，窄门（1.4 m）仍可通行 |
| `max_vel_x / min_vel_x` | 0.30 / −0.12 m/s | 允许倒车，U 型死角可退出 |
| `max_vel_theta` | 1.5 rad/s | ω = 2·v_axle/track = 2×0.30/0.196 ≈ 3.1 的上限留余量 |
| `acc_lim_x` | 0.5 m/s² | 插件 `max_wheel_acceleration 15 rad/s² × 0.036 = 0.54 m/s²`，控制器不得超出物理能力 |
| `laser_max_range / obstacle_max_range` | 8.0 / 4.5 m | 与雷达量程一致 |
| `xy_goal_tolerance` | 0.20 m | 半径 0.21 的底盘在 1.4 m 门宽内的可达重复精度 |
| 规划器 | NavFN + A*，`allow_unknown: false` | 使用离线完整地图时禁止穿越未知区 |
| 控制器 | DWB，7 个 critic | 差速室内机标准配法 |

`config/ekf.yaml`：里程计只喂 `vx` 与 `yaw_rate`（差速车的 `vy` 不可测，喂进去等于把噪声当状态），
姿态与角速度由 IMU 提供，`imu0_relative: true` 以开机第一帧为零点，
避免把仿真世界的绝对航向当成无误差真值。

---

## 6. 场景

`worlds/lab.world`：12 × 9 m 内部空间，墙厚 0.12、高 1.0 m，两道隔断形成西区 / 中央通道 / 东区，
门洞分别为 1.4 m（x = −1.8）与 2.0 m（x = +2.0），5 个障碍物（货箱、长凳、立柱、托盘、圆桶）。
机器人出生点在世界原点、yaw = 0，因此 **SLAM 的 map 原点与世界原点重合**，
`config/waypoints.yaml` 里的每个航点都能与场景坐标直接对照，
`scripts/gen_map_from_world.py` 也按同一组坐标生成离线地图。

巡检航线 9 点、折线长 29.58 m，逐点做过净空校验（距墙与障碍 ≥ 0.5 m > `robot_radius` + 膨胀余量）。

---

## 7. 任务层

`navbot_patrol` 基于 `nav2_simple_commander.BasicNavigator`：

```
setInitialPose → waitUntilNav2Active → followWaypoints
    → 轮询 isTaskComplete / getFeedback → getResult
```

实现要点：

- 航点、超时、初始位姿全部来自 `config/waypoints.yaml`；查找顺序
  `--waypoints` > 环境变量 `NAVBOT_WAYPOINTS` > 包内默认配置。
- 服务就绪等待有上限（默认 30 s），超时以退出码 2 结束并提示下一步，不会无限挂住。
- 任务总超时（默认 420 s）触发 `cancelTask`；Ctrl+C 与外部关闭信号都走同一条清理路径。
- 反馈日志按"航点序号变化 + 最小时间间隔"节流。
- 结果按 `TaskResult` 分类，多轮巡检（`task.loop`）中途取消或失败即停止后续轮次。
- 退出码：0 成功 / 1 配置错误 / 2 Nav2 未就绪 / 3 被取消 / 4 执行失败 —— 便于脚本与 CI 判定。

---

## 8. 诊断脚本

| 脚本 | 用途 |
|---|---|
| `scripts/env.sh` | source ROS 与工作区、导出 WSL2 专用 DDS 配置 |
| `scripts/build.sh` | 修正执行位 + `colcon build --symlink-install` |
| `scripts/kill_sim.sh` | 清理孤儿进程、gzmaster 套接字、SHM 端口锁 |
| `scripts/check_system.sh` | 话题与发布者数量、EKF 输入、TF 归属、Nav2 服务端 |
| `scripts/tf_probe.py` | 按仿真时钟查 TF，校验"每条边唯一父帧" |
| `scripts/gen_map_from_world.py` | 按场景坐标生成 nav2 栅格地图（跳过 SLAM） |
| `scripts/save_map.sh` | `map_saver_cli` 绝对路径存图 |
| `scripts/measure_sim_speed.sh` | 按传感器组合量仿真实时倍率 |
| `scripts/verify_odometry.py` → 已并入 `navbot_gazebo/tools/` | 驱动转圈，拟合 `yaw_odom / yaw_true` |
| `scripts/selftest.sh` | 健康检查 + 轮距标定两阶段自测 |
| `scripts/nav_check.sh` | 地图 → AMCL → Nav2 → 巡检受理 的端到端联调 |
| `scripts/reset_dds.sh` | 复位 Fast DDS 共享内存状态 |

---

## 9. 验证状态

**已验证**

- `colcon build` 三包全绿；9 个 launch 文件构造通过
- `check_urdf` 通过；TF 树形状与第 4 节一致；轮心 z = 0.036、万向球 z = 0.018 均正好触地
- xacro 四个参数（`publish_odom_tf` / `enable_camera` / `enable_lidar` / `lidar_visualize`）全部生效
- 展开后的 URDF 中 `wheel_separation = 0.196`、`wheel_diameter = 0.072` 与关节几何一致
- headless 与窗口两种模式均能完成实体 spawn；`/odom`、`/scan`、`/imu/data`、`/cmd_vel` 正常
- Nav2 全栈用本工程参数启动：15 个进程、`/map /plan /odom /scan /imu/data` 与
  `/navigate_to_pose`、`/follow_waypoints` 均在；巡检节点读取 9 点 / 29.58 m 航线并进入等待

**受环境限制未完成**

- 完整 9 点巡检走完、SLAM 实跑建图：在无 GPU 直通的 WSL2 上物理步进明显低于实时，
  需要几十分钟到数小时的墙上时间；在原生 Linux 或带 GPU 的环境正常。
- `verify.launch.py` 的轮距斜率 k≈1 实测：同上，脚本与步骤已就位。

---

## 10. WSL2 注意事项

1. **`/etc/profile.d` 里的 GPU 渲染设置会让 gzclient 卡死**。若 `LIBGL_ALWAYS_SOFTWARE=0`
   走 Mesa D3D12 通道，gzclient 一开就饿死 gzserver 的 ROS 服务线程，表现为
   `Service /spawn_entity unavailable`。本工程在 launch 内部强制
   `LIBGL_ALWAYS_SOFTWARE=1`（仅带窗口时），无需手工 export。
2. **Fast DDS 共享内存端口锁泄漏**。被 `kill -9` 打断的节点会在 `/dev/shm` 留下
   `fastrtps_portXXXX` 锁，之后新进程出现"话题能列出、订阅收不到数据"。
   `config/fastdds_wsl.xml` 只禁用 SHM、**不限制网络接口**（WSL2 的 `lo` 不支持组播，
   把接口白名单写死成 `127.0.0.1` 会破坏发现机制）。`scripts/reset_dds.sh` 可复位。
3. **验证类节点要与仿真同一个 launch 会话**。跨会话 DDS 投递不稳定，
   所以轮距标定做成 `verify.launch.py` 而不是另开终端跑脚本。
4. **查 TF 用仿真时钟**。`ros2 run tf2_ros tf2_echo` 用系统时间查询，仿真里会得到
   `frame does not exist` 这类误导结果；用 `scripts/tf_probe.py`。
5. **`pkill -f` 的自匹配陷阱**：在 `wsl.exe -e bash -lc '...'` 里内联写
   `pkill -f "gzserver"`，模式会匹配到调用者自己的命令行并把自己的 shell 杀掉，
   表现为"命令毫无输出"。清理逻辑放在脚本文件里，pattern 锚定到安装路径。

---

## 11. 许可

MIT，见 `LICENSE`。场景中所有几何、参数、脚本与文档均为本工程自定义，不含第三方模型或网格文件。

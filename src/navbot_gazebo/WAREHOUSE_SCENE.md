# nav2 warehouse 场景接入说明

## 来源

| 项 | 值 |
|---|---|
| 地图 | https://github.com/ros-navigation/navigation2/blob/main/nav2_bringup/maps/warehouse.pgm |
| 地图 yaml | 同上 `warehouse.yaml`（resolution 0.03，origin [-15.1, -25, 0]） |
| md5 | `03b91be75de9a704872b0eac5a5665d1`（与上游一致，未改动一字节） |
| 许可 | Apache-2.0 |

**上游不附带 world 文件**。`navigation2` 全仓库（main / humble / jazzy / iron / kilted 均查过）递归搜不到任何 `.world`/`.model`，只有 humble 分支的 `world_only.model`（空世界）。所以 `warehouse.world` 是按地图反推出来的，见下节。

## 场景规模

- 30.18 × 50.22 m（1006 × 1674 px @ 0.03 m）
- x ∈ [-15.10, 15.08]，y ∈ [-25.00, 25.22]
- 自由空间 1279.6 m²，按 0.45 m 膨胀后仍可行 1040 m²

## 地图语义（trinary 模式下的实际含义）

| 灰度 | 含义 | 占比 |
|---|---|---|
| 0 | 激光命中障碍（货架外表、墙） | 1.84% |
| 205 | 未知：货架内部 / 未探测区 | 13.71% |
| 254 / 255 | 自由 | 84.45% |

货架是**实心块**：外圈 0 是扫到的面，内部整片 205 是激光穿不透的部分。因此 world 里必须建成实心 box，不能建成空心。

## world 的生成方法

从地图反推，不是手写：

1. `solid = gray < 250`（障碍 ∪ 未知）取连通域
2. 内部实体按「占用率 ≥ 0.5 收紧边界」消除 205 未探测带造成的膨胀
3. 外墙按地图边缘的 occ（`gray ≤ 10`）连续段实测定位，不用猜测厚度
4. 高度按语义赋值：大货架 2.4 m、护栏 1.2 m、立柱 2.4 m、墙 2.5 m、地面 0.05 m

产出 30 个 box（1 地面 + 5 外墙段 + 24 内部实体）。

## 关键实体尺寸

| 名称 | 尺寸 (m) | 中心 |
|---|---|---|
| S1 纵货架 | 2.10 × 18.00 | (6.08, -12.94) |
| S2 横货架 | 18.06 × 2.10 | (3.50, 9.08) |
| S3 纵货架 | 2.07 × 18.00 | (-1.95, -12.97) |
| S4 纵货架 | 2.04 × 17.97 | (-8.95, -12.96) |
| 外墙 西/东 | 0.06 / 0.12 厚 | x = -14.92 / 15.02 |
| 外墙 北 | 0.09 厚 | y = 25.16 |
| 外墙 南（两段） | 0.06 厚 | y = -24.90，中间开口 x ∈ [-7.36, 7.55] |

## 一致性校验结果

| 指标 | 结果 |
|---|---|
| world 栅格化 vs 地图 IoU | 70.71% |
| 侵入机器人可达自由区 | 5.94 m² |
| 侵入深度中位数 | 0.030 m |
| 侵入深度 ≤ 0.09 m（94.6%） | 是 |
| **超机器人半径的侵入** | **0.000 m²** |
| 出生点 (0,0) 与 10 个航点碰撞检测 | 0/10 碰撞 |
| 单实体压到自由区 > 25% 的模型 | **0 个**（修正北墙后） |

### 缺口说明（必须交代清楚）

地图 `solid` 比 world 实体多 64.81 m²，其中：

- **8.11 m² 是真实 occ 障碍** —— 主要是 1–2 px 的边缘量化差（0.03–0.06 m），以及地图上的扫描散点
- **56.70 m² 是 205 未知带** —— 最大一块 50.17 m² 位于 x∈[-14.89, 7.82]、y∈[4.58, 19.34]，该区 76.11% 是自由空间、2.13% 是障碍（横货架边缘）、21.76% 未知

**未知区不建实体是正确的**：AMCL 的 `likelihood_field` 只对栅格里的 occupied 打分，未知格不参与；若给未知区造实体，反而会让激光在自由空间里打出虚假障碍，导致定位发散。

IoU 偏低的另一半原因是分母含未知：把货架内部 205 也计入 `solid`，而这些在 world 里是实心体（正确），却被算作"仅 world"，属统计口径差异，不是几何错误。

## 航点

10 点 / 121.44 m，全部经 A\* 在「自由 + 0.45 m 膨胀」安全栅格上验证连通：

| 航点 | x | y | yaw |
|---|---|---|---|
| home | 0.0 | 0.0 | 0 |
| rack_west | -5.6 | -12.0 | 180 |
| rack_south | -5.6 | -21.0 | 180 |
| rack_east | 2.2 | -21.0 | 0 |
| rack_east_up | 2.2 | -12.0 | 90 |
| east_passage | 13.5 | 1.0 | 90 |
| north_row | 13.5 | 13.0 | 90 |
| north_mid | 0.0 | 13.0 | 180 |
| north_west | -8.0 | 13.0 | 180 |
| return_home | 0.0 | 0.0 | 0 |

**拓扑限制**：横货架 S2 在 x∈[-5.53, 12.53]、y∈[8.03, 10.13] 把场景分成南北两块，只能从东侧（x≈13.5）或西侧（x≈-7.5）绕行。西侧 x∈[-12.6, -11] 有一处被货架延伸结构封死的孤立区（离主区 72 m 绕行），航线已避开。

## 参数调整

`nav2_params.yaml` 三组：

| 参数 | 原值 | 新值 | 原因 |
|---|---|---|---|
| `raytrace_max_range` | 5.0 | 8.0 | 雷达量程 8.0 m，原值小于量程导致 costmap 清除不彻底 |
| `obstacle_max_range` | 4.5 | 7.5 | 同上，标注障碍的有效距离 |
| `z_hit` | 0.5 | 0.9 | 原值把一半概率质量给随机噪声，大场景下粒子发散 |
| `z_rand` | 0.5 | 0.03 | 同上 |
| `sigma_hit` | 0.2 | 0.12 | 地图 0.03 m 分辨率下提高打分明锐度 |
| `max_beams` | 60 | 90 | 540 线雷达，60 束过稀 |
| `min_particles` / `max_particles` | 500 / 2000 | 800 / 4000 | 30×50 m 场景需更多粒子 |
| `pf_err` | 0.05 | 0.02 | 收紧重采样阈值 |
| `update_min_d` / `update_min_a` | 0.3 / 0.25 | 0.15 / 0.15 | 更频繁更新，提升跟踪精度 |
| `laser_max_range` | 8.0 | 7.5 | 与 `obstacle_max_range` 对齐 |
| `laser_likelihood_max_dist` | 2.0 | 1.0 | 缩小说然场影响半径，减少远处误匹配 |

`inflation_radius` 保持 0.45 m——场景最窄通道为 S3 与 S4 之间（约 4.95 m）、S1 与东墙之间（约 7.8 m），0.45 m 有充裕余量。

### AMCL 参数是必须改的（实测证据）

原始参数下实测：

| 指标 | 原参数 | 修正后 |
|---|---|---|
| AMCL vs odom 偏差 | **0.677 m** | **0.231 m** |
| σx | 0.585 → **2.159（发散）** | 0.239（收敛） |
| 判定 | FAIL | PASS |

`z_hit: 0.5 / z_rand: 0.5` 在 540 线雷达 + 0.03 m 分辨率地图上会让粒子滤波直接发散，这不是调优偏好，是能否收敛的问题。

## 实跑验收（WSL2 Ubuntu 22.04 + ROS 2 Humble + Gazebo Classic 11）

### 环境

- Ubuntu 22.04.5 LTS，ROS 2 Humble，Gazebo 11，nav2 30 个包
- 12 核 / 7 GB，headless 模式，`LIBGL_ALWAYS_SOFTWARE=1`

### 编译

```
Summary: 3 packages finished [5.41s]
```

### 启动链

```
gzserver .../worlds/warehouse.world  ✔
spawn_entity: Successfully spawned entity [navbot]  ✔
nav2 进程 11 个  ✔
/amcl active [3]、/bt_navigator active [3]  ✔
话题 /scan /odom /map /amcl_pose /plan /cmd_vel 全在位  ✔
动作 /navigate_to_pose /follow_waypoints 全在位  ✔
```

### 传感器实测

| 项 | 实测值 | 与设计一致性 |
|---|---|---|
| `/scan` 频率 | 9.959 – 9.997 Hz | 设计 10 Hz ✔ |
| 光束数 | 540（±157°） | 设计 540 线 ✔ |
| 角度范围 | ±2.7402 rad = ±157° | ✔ |
| 量程 | 0.150 / 8.000 m | ✔ |
| 有效光束 | 195 – 196 / 540 | 场景遮挡所致，正常 |
| 有效距离 | 1.677 – 7.964 m | 未触量程上限 ✔ |

### AMCL 收敛

驱动 1.2 m + 转 90° 后与里程计比对：

| 项 | 值 |
|---|---|
| AMCL 位姿 | (1.102, -0.205) |
| odom 位姿 | (1.144, 0.055) |
| 跟踪误差 | **0.263 m** |
| σx / σy / σyaw | 0.250 / 0.244 / 0.132 |
| 判定 | **PASS** |

### 完整巡检

```
航点 10 个，航线 105.03 m
home -> rack_west -> rack_south -> rack_east -> rack_east_up
     -> east_passage -> north_row -> north_mid -> north_west -> return_home
本轮完成，累计 440.3 s
失败/超时：0 条
```

终点回到 home：AMCL `(-0.085, 0.101)`，目标 `(0.0, 0.0)`，**误差 0.135 m**，优于 `xy_goal_tolerance: 0.20`。

两次独立复跑结果一致（434.4 s / 误差 0.121 m 与 440.3 s / 误差 0.135 m），差异在仿真步进抖动范围内。

### 两个跑通的关键修复

1. **AMCL 参数**（见上）：不改则定位发散，巡检必然失败
2. **北墙位置**：原建在 `y=25.16、厚 0.09`，但地图上北墙 occ 只占 row 0–1（`y∈[25.16, 25.22]`，厚 0.06）。修正为 `y=25.19、厚 0.06` 后，"压到自由区 >25% 的实体"从 1 个降为 0 个

## 运行

```bash
source /opt/ros/humble/setup.bash
cd ~/navbot_ws
./scripts/build.sh
source install/setup.bash

ros2 launch navbot_gazebo navigate.launch.py
ros2 launch navbot_patrol patrol.launch.py
```

回退到原 lab 场景（12.24 × 9.4 m）：

```bash
ros2 launch navbot_gazebo navigate.launch.py \
  map_file_path:=$(ros2 pkg prefix navbot_gazebo)/share/navbot_gazebo/maps/lab_map.yaml
```

world 需改 `sim.launch.py` 的默认值，或临时把 `warehouse.world` 改名。

## 未验证项

以下没有实测，如实列出：

- **带窗口（gzclient）模式**：只在 headless 下验证，WSLg 下 gzclient 是否稳定未测
- **`slam.launch.py` 建图链路**：本次只验了 `navigate.launch.py`（AMCL 定位），未跑 slam_toolbox
- **`verify.launch.py` 轮距标定**：未在这套新场景下重跑
- **`scripts/selftest.sh`**：其阶段 1 会调 `check_system.sh`，预期通过，但未在新场景实跑
- **长时间稳定性**：只跑了单轮 434 s，未做多轮循环（`task.loop > 1`）
- **相机链路**：本次以 `enable_camera:=false` 启动，`/navbot_camera/*` 未验证

### 复验命令

```bash
cd ~/navbot_ws
source /opt/ros/humble/setup.bash && source install/setup.bash

./scripts/check_system.sh          # 话题/TF/Nav2 体检
./scripts/tf_probe.py 'map->odom' 'odom->base_footprint' 'base_footprint->base_link'
ros2 topic hz /scan                # 应约 10 Hz
ros2 topic echo /scan --once --field ranges | head

ros2 launch navbot_gazebo navigate.launch.py headless:=true use_rviz:=false
ros2 launch navbot_patrol patrol.launch.py
```

### 若 AMCL 仍漂移

按以下顺序排查，不要先怀疑参数：

1. RViz 里把 `/map`（地图）与 `/scan`（激光）叠加，看货架轮廓是否重合
2. 不重合 → 回到 world 几何，用 `map_vs_scan.py` 式的射线投射定位偏差实体
3. 重合但 σ 不降 → 才怀疑 AMCL 的 `z_hit/z_rand/sigma_hit`

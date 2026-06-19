# percep_localization

感知组位姿估计包：融合 Gazebo 仿真输出的多源传感器数据，估计车辆位姿与速度，维护环境地图，并把车辆状态输出给规划/控制模块。

---

## 1. 包的作用

在完整的自动驾驶链路中，`percep_localization` 处于“感知之后、规划控制之前”的位置，核心职责是：

1. **位姿估计**：把轮式里程计、IMU、磁力计、GPS 等多种传感器数据融合在一起，得到比单一传感器更稳定、漂移更小的车辆位姿（`x, y, yaw`）和速度（`v, omega`）。
2. **坐标系与 TF 维护**：发布 `odom → base_link` 的 TF，让锥桶检测、路径规划、RViz 可视化等模块能够在统一的全局坐标系下工作。
3. **环境建图**：
   - 通过 `slam_toolbox` 根据激光雷达 `/scan` 建立二维栅格地图 `/map`。
   - 通过 `cone_map_node` 把仿真感知输出的锥桶坐标转换到全局坐标系，维护一个全局锥桶地图，供 RViz 可视化或作为规划参考。
4. **向下游输出车辆状态**：把融合后的位姿和速度发布到 `/percep/vehicle_state`，供 `racecar_control` / `racecar_control_mpc` 使用。

---

## 2. 内部结构

```
percep_localization/
├── percep_localization/           # Python 节点源码
│   ├── ekf_fusion_node.py         # EKF 传感器融合（核心，默认启用）
│   ├── vehicle_state_publisher.py # 最小车辆状态转发（默认不启用，降级用）
│   ├── cone_map_node.py           # 锥桶地图与可视化（默认启用但仅作可视化旁路）
│   └── utils.py                   # 四元数/yaw 转换、角度归一化、磁力计转 yaw 等工具函数
├── launch/
│   ├── perception.launch.py       # 感知组一键启动（默认启用 EKF + 锥桶可视化 + SLAM）
│   ├── ekf.launch.py              # 单独启动 EKF 节点，用于调试
│   └── slam.launch.py             # 单独启动 SLAM，用于调试
├── config/
│   ├── ekf.yaml                   # robot_localization 官方 EKF 配置（本项目手写 EKF 的备用方案）
│   └── slam_toolbox.yaml          # slam_toolbox 参数配置
└── package.xml / setup.py         # ROS2 包元数据
```

### 2.1 各节点默认启用情况及作用

#### `ekf_fusion_node` — 默认启用（核心）

- **运行作用**：
  - 订阅 `/odom`（轮式里程计）、`/imu`（惯性测量单元）、`/magnetometer`（磁力计）、`/navsat`（GPS）。
  - 使用扩展卡尔曼滤波（EKF）融合上述传感器，估计状态 `X = [x, y, yaw, v, omega]^T`。
  - 发布 `/odom/filtered`（融合后里程计）、`/percep/vehicle_state`（给规划/控制使用的车辆状态）。
  - 广播 TF：`odom → base_link`，供 SLAM、锥桶地图、路径规划查询车辆位姿。
- **运行入口**：`perception.launch.py` 默认启动；`full_system.launch.py` 也会直接启动它。

#### `cone_map_node` — 默认启用，但仅作 RViz 可视化旁路

- **运行作用**：
  - 订阅 `sim_perception` 发布的 `/perception/cones`（`fsd_common_msgs/ConeDetections`）。
  - 通过 TF 把锥桶从输入坐标系转换到 `map_frame`（默认 `odom`）。
  - 维护一个全局锥桶地图，对重复观测到的锥桶做加权平均，减少抖动。
  - 默认发布 `/percep/cone_map_markers`（`visualization_msgs/MarkerArray`）供 RViz 显示。
- **默认关闭的功能**：
  - `/percep/cone_map`（`geometry_msgs/PoseArray`）。
  - `/percep/cone_detections`（`fsd_common_msgs/ConeDetections`）。
  - 这两个输出可在参数中打开，但**规划组应继续直接订阅 `/perception/cones`**，避免覆盖主感知链路。
- **其他作用**：
  - 支持订阅原始 `/scan`（`sensor_msgs/LaserScan`）或 `PointCloud2`，自行做欧氏距离聚类生成锥桶，作为真实雷达/点云感知的备用接入点（需要在参数中指定对应 topic）。
  - 2D 激光雷达看不到颜色，节点内使用“车身左侧蓝、右侧黄”的启发式规则猜测颜色。

#### `slam_toolbox`（通过 `slam.launch.py`）— `perception.launch.py` 默认启用，但 `full_system.launch.py` 未启用

- **默认运行作用（当使用 `perception.launch.py` 时）**：
  - 订阅 `/scan` 和 TF `odom → base_link`。
  - 在线同步建图，发布 `/map`（`nav_msgs/OccupancyGrid`）和 TF `map → odom`。
- **默认未启用的原因**：
  - 当前 `full_system.launch.py` 主链路直接依赖 `sim_perception` 输出的 `/perception/cones` 进行规划，暂未把 `/map` 用于路径规划。
  - SLAM 作为独立建图能力保留，方便后续接入真实雷达、做全局定位或保存地图。
- **其他作用**：
  - 可单独运行 `ros2 launch percep_localization slam.launch.py` 调试 2D 激光 SLAM。
  - 保存的地图后续可用于 `localization` 模式做全局重定位。

#### `vehicle_state_publisher` — 默认不启用（降级/备用）

- **运行作用**：
  - 在 `perception.launch.py` 中设置 `use_ekf:=false` 时启用。
  - 直接订阅 `/odom`，转发到 `/percep/vehicle_state`。
  - 广播 TF：`odom → base_link`。
- **适用场景**：
  - EKF 节点出现故障、传感器数据缺失，或只需要最简位姿链路时，保证规划/控制仍能收到车辆状态。

#### `robot_localization`（`config/ekf.yaml`）— 默认不启用（官方备用方案）

- **作用**：
  - 本项目默认使用手写的 `ekf_fusion_node.py`，便于教学和调试 EKF 每一步。
  - `config/ekf.yaml` 是 ROS2 官方 `robot_localization` 包的 EKF 配置模板，作为后续替换或对比基准保留。

---

## 3. 与上下游的对接

### 3.1 上游输入

| 来源 | 话题 | 消息类型 | 说明 |
|------|------|----------|------|
| `racecar_description` → Gazebo → `ros_gz_bridge` | `/odom` | `nav_msgs/Odometry` | 轮式里程计，高频、短时准确但会漂移 |
| `racecar_description` → Gazebo → `ros_gz_bridge` | `/imu` | `sensor_msgs/Imu` | 角速度、加速度 |
| `racecar_description` → Gazebo → `ros_gz_bridge` | `/magnetometer` | `sensor_msgs/MagneticField` | 地磁场测量，用于绝对航向 |
| `racecar_description` → Gazebo → `ros_gz_bridge` | `/navsat` | `sensor_msgs/NavSatFix` | GPS，用于绝对位置 |
| `racecar_description` → Gazebo → `ros_gz_bridge` | `/scan` | `sensor_msgs/LaserScan` | 2D 激光雷达，SLAM 与锥桶聚类用 |
| `sim_perception` | `/perception/cones` | `fsd_common_msgs/ConeDetections` | 仿真锥桶检测结果 |
| `racecar_description` | 静态 TF `world → odom` | `tf2_msgs/TFMessage` | world 与 odom 原点重合 |

> 坐标基准：GPS 切平面原点为 `lat=23.16°, lon=113.40°`，地磁场参考向量为 `(5.5645e-6, 22.8758e-6, -42.3884e-6) T`（ENU），与 `racecar_description/接口.md` 保持一致。

### 3.2 下游输出

| 话题 | 消息类型 | 消费者 | 说明 |
|------|----------|--------|------|
| `/percep/vehicle_state` | `nav_msgs/Odometry` | `racecar_control/control_node`、`racecar_control_mpc/mpc_node` | 融合后的车辆位姿与速度 |
| `/odom/filtered` | `nav_msgs/Odometry` | RViz、调试用 | 与 `/percep/vehicle_state` 内容一致 |
| TF `odom → base_link` | `tf2_msgs/TFMessage` | `path_planning`、`cone_map_node`、RViz | 车辆位姿坐标变换 |
| `/map` | `nav_msgs/OccupancyGrid` | RViz、潜在规划模块 | SLAM 栅格地图 |
| TF `map → odom` | `tf2_msgs/TFMessage` | RViz | SLAM 坐标系 |
| `/percep/cone_map_markers` | `visualization_msgs/MarkerArray` | RViz | 全局锥桶地图可视化 |
| `/percep/cone_map` | `geometry_msgs/PoseArray` | 可选规划模块 | 默认关闭 |
| `/percep/cone_detections` | `fsd_common_msgs/ConeDetections` | 可选规划模块 | 默认关闭 |

---

## 4. 主要用到的原理

### 4.1 扩展卡尔曼滤波（EKF）

`ekf_fusion_node.py` 实现了一个五维状态的 EKF：

- 状态向量：`X = [x, y, yaw, v, omega]^T`
  - `x, y`：车辆在 `odom` 坐标系下的位置（m）
  - `yaw`：车头朝向（rad）
  - `v`：纵向速度（m/s）
  - `omega`：横摆角速度（rad/s）

**预测步骤**（运动学模型）：

```
x_{k+1}     = x_k     + v_k * cos(yaw_k) * dt
y_{k+1}     = y_k     + v_k * sin(yaw_k) * dt
yaw_{k+1}   = yaw_k   + omega_k * dt
v_{k+1}     = v_k
omega_{k+1} = omega_k
```

其中 `v` 与 `omega` 优先使用 `/odom` 的 twist，并与 `/imu` 的 `angular_velocity.z` 做互补加权。

**更新步骤**：分别用以下观测对预测状态进行修正：

- `/odom` 的位置、姿态 `yaw`、速度
- `/magnetometer` 解算出的绝对航向 `yaw`
- `/navsat` 转换后的 ENU 平面位置

通用 EKF 更新公式：

```
y = z - H * X              # 观测残差
S = H * P * H^T + R        # 残差协方差
K = P * H^T * S^{-1}       # 卡尔曼增益
X = X + K * y
P = (I - K * H) * P
```

### 4.2 GPS 经纬度 → ENU 平面坐标

以 `gps_origin_lat/lon` 为原点，采用局部切平面近似：

```
x = R_earth * cos(lat0) * dlon
y = R_earth * dlat
```

其中 `R_earth = 6371000 m`，`dlat, dlon` 为相对原点的弧度差。

### 4.3 磁力计航向估计

根据地磁场参考向量与实测向量在水平面上的方向差异计算 `yaw`：

```
yaw = atan2(ref_y * meas_x - ref_x * meas_y,
            ref_x * meas_x + ref_y * meas_y)
```

公式推导基于“参考磁场相对测量磁场旋转的角度等于车体航向”。

### 4.4 2D 激光 SLAM（slam_toolbox）

- **扫描匹配**：将当前 `/scan` 与历史子图进行匹配，估计 `base_link` 在 `map` 下的位姿。
- **位姿图优化**：当车辆移动超过 `minimum_travel_distance` 或转动超过 `minimum_travel_heading` 时，把新激光帧加入位姿图，并通过 Ceres Solver 做图优化，减小回环误差。
- **TF 发布**：`map → odom`，与 EKF 发布的 `odom → base_link` 共同构成完整坐标链。

### 4.5 锥桶地图维护

`cone_map_node` 的核心流程：

1. **坐标变换**：通过 TF 把输入锥桶从 `base_link` / `lidar_link` 转换到 `map_frame`（默认 `odom`）。
2. **聚类（可选旁路）**：对原始 LaserScan/PointCloud2 做有序欧氏距离聚类，把点云分成潜在锥桶簇。
3. **颜色启发式**：2D 雷达无法识别颜色，按 `local_y > 0` 蓝、`local_y < 0` 黄猜测；若输入已有 `color` 字段则优先采用。
4. **地图合并**：新锥桶与地图中旧锥桶距离小于 `merge_dist` 时，按 `0.8`（旧）+ `0.2`（新）加权平均更新位置，抑制噪声。
5. **可视化发布**：生成 RViz `MarkerArray`，用不同颜色显示蓝/黄/未知锥桶。

---

## 5. 使用方式

### 5.1 一键启动感知组（含 EKF + 锥桶可视化 + SLAM）

```bash
# 先启动 Gazebo 仿真
ros2 launch racecar_description view_in_gazebo.launch.py

# 再启动感知定位
ros2 launch percep_localization perception.launch.py
```

### 5.2 关闭 EKF，使用最简位姿转发

```bash
ros2 launch percep_localization perception.launch.py use_ekf:=false
```

### 5.3 单独调试 EKF / SLAM

```bash
ros2 launch percep_localization ekf.launch.py
ros2 launch percep_localization slam.launch.py
```

### 5.4 完整自动驾驶链路

```bash
ros2 launch racecar_description full_system.launch.py
# 或使用 MPC 控制器
ros2 launch racecar_description full_system.launch.py controller:=mpc target_speed:=1.5
```

---

## 6. 注意事项

- **GPS 原点与地磁场参考值**必须与 `racecar_description/worlds/racecar_world.sdf` 及 `racecar_description/接口.md` 保持一致，否则绝对位置/航向会偏离。
- **TF 链**：`world → odom`（静态）由 `racecar_description` 发布；`odom → base_link` 由 `ekf_fusion_node`（或 `vehicle_state_publisher`）发布。不要同时启动多个 `odom → base_link` 广播器，否则 TF 会冲突。
- **SLAM 与 EKF 的依赖**：`slam_toolbox` 需要 `odom → base_link` 的 TF，因此启动 SLAM 前务必确保 EKF（或降级节点）已经运行。
- **锥桶地图目前仅作可视化**：规划组应直接订阅 `sim_perception` 的 `/perception/cones`；`/percep/cone_detections` 默认关闭，避免覆盖主感知链路。

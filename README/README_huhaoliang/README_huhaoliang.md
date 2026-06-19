 我们在控制节点输入订阅/track_points也就是规划节点发布的参考路径、绑定在回调函数里提升读取效率、然后把路径存在ref_path变量、等待50hz的定时循环主函数入口接受、定位融合节点发布的车的坐标和速度以同样的流程和路径list一起传入横向控制lateral_controller的核心函数、将坐标投影算出横向误差、根据车速带入相关公式计算前瞻距离扫出路径上的前瞻点叠加横向误差的修正项（去掉了）从而推出转向角、用车速v、转向角δ算出Twist需要的 angular.z、同样需要判断是否到达路径终点、到达直接置零速度、同时Gazebo AckermannDrive系统插件 和 ros_gz_bridge 消息不互通控制节点到URDF里加 ROS 插件libgazebo_ros_ackermann_drive.so（能直接发Twist）控制车辆运动、并转发可视化的路径到rviz用于调试同时在交流中发现当带有横向误差修正项时、小车会乱飞、在感知的环节加入滑动均值滤波可以发布锥桶位置固定、rviz和规划接收到的锥桶就不会再乱动了、保证路径规划的稳定、从而让小车更稳定地行驶、个人在尝试相关环境依赖和配置的时候遇到非常严重的问题、起初因为变换关系不对导致感知锥桶位置呈现无规则棋盘式的分布、这直接导致蜿蜒的规划路径出现、完全无法还原赛道的真实轮廓、小车随后在赛道疯狂抖动、按照坐标校准同时增加时段均值滤波、保证map到base_link完整稳定
ROS接口
订阅话题
 话题名  消息类型  来源  说明
/track_points  nav_msgs/Path 路径规划 / 建图模块  世界坐标系下的目标路径点列 
odometry/filtered  nav_msgs/Odometry  EKF 状态估计  车辆位姿 (x, y, yaw) 与速度  
发布话题
 话题名  消息类型  说明 
/model/racecar/cmd_vel geometry_msgs/Twist  车辆控制指令 Twist(linear.x, angular.z)
/track_points_viz nav_msgs/Path  路径可视化转发（transient_local latched，供 RViz 订阅） 
坐标约定
世界坐标系：右手系，x向前，y向左
车辆航向角yaw：与x轴夹角，逆时针为正
转向角 steering_angle：正值 = 左转（Ackermann 前轮偏角）
Twist.angular.z：ω = v · tan(δ) / L，由转向角换算得到
纯跟踪的运动模型及相关公式原理：
纯跟踪轨迹跟踪控制器实现车在转弯的时候，四个轮子的转弯半径各不相同，内侧轮子必须比外侧轮子转更多角度，因为内侧轮子离圆心近，画的小圆，轨迹曲率大，所以角度必须大；外侧轮子离圆心远，画的大圆，轨迹曲率小，角度自然就小，保证所有轮的轴线相较于同一点，而纯跟踪就有点向我们考驾照，我们会预瞄车头的某个点位，让那个位置始终和地面的黄线边界靠近，只是这个追踪点在车头，而春跟踪跟踪点在后轴中心，它会计算后轴中心到前方目标点的弧线，L就是后轴中心和路径目标点的距离，你想让车沿着一条完美的圆弧从 O 行驶到 G，并且到达 G 时车头方向刚好沿着路径的切线方向，会得到一个转弯半径R，R越小方向盘打得越急，公式tan(δ) = L_w / R，这就是其中一个循环，每走一步，都重新找G 运动学自行车模型核心思想问题本质：用简化模型描述车辆运动，忽略复杂的动力学效应（如轮胎力、惯性力），只考虑几何约束。适用场景：低速、稳态行驶场景，用于初步分析车辆运动轨迹数学原理车辆运动的几何约束：车辆只能沿后轮切线方向运动（非完整约束）前轮转角决定转向半径转向半径公式： R=L/tan(σ) 其中L是轴距，σ是前轮转角。
运动学方程： φ_dot=v/R=vtan(σ)/L %横摆角速度 X_dot=vcos(φ) %纵向速度 Y_dot=v*sin(φ) %横向速度物理意义：横摆角速度与车速成正比，与轴距成反比前轮转角越大，转向半径越小，车辆转向越灵活
参数默认值及说明
wheelbase 0.6  轴距 (m) 
lookahead_distance  1.5  基础前瞻距离 (m) 
min_lookahead 1.2  前瞻距离下限 (m) 
lookahead_ratio  0.70  速度对前瞻的贡献系数 
max_steering_angle 0.50 | 最大转向角 (rad) 
steering_scale 0.70  转向角整体缩放系数 
steering_filter_alpha 0.60  低通滤波系数（越大越平滑）
target_speed 2.0  直道目标速度 (m/s) 
speed_kp ki kd  1.5 / 0.05 / 0.02  纵向 PID 系数 
max_accel 2.0  最大加速度限幅 (m/s²) 
curve_gain  8.0  弯道减速强度，越大弯道越慢 
min_speed  0.4  最低速度 (m/s) 
control_frequency  50.0  控制频率 (Hz) 
已禁用的修正项：
CTE修正：纯追踪本身通过前瞻点即能自然消除横向偏差，叠加反而可能导致振荡（代码中 `if False` 保留，可按需启用）
曲率前馈：与纯追踪的曲率计算重复，叠加会导致过度转向当前ff_gain=0
控制限幅度映射
δ ← δ × steering_scale
δ ← clip(δ, -max_steering, +max_steering)
| 函数/方法 | 行号 | 类型 |
|-----------|------|------|
| `LateralController.__init__(wheelbase)` | 22 | 构造函数 |
| `LateralController.compute(path_xy, vehicle_x, vehicle_y, vehicle_yaw, current_speed, ...)` | 25–102 | 核心计算方法 |
| `LateralController._find_lookahead_point(path_xy, vx, vy, vyaw, lookahead)` | 105–156 | 静态方法，搜索前瞻点 |
| `LateralController.wheelbase` (property getter) | 158–160 | 属性访问器 |
| `LateralController.wheelbase` (property setter) | 162–164 | 属性修改器 |

#!/usr/bin/env python3
"""
cone_map_node.py

这是感知组位姿估计的"加分项节点"。

功能：
    订阅 sim_perception 发布的 `/perception/cones`，
    把锥桶坐标转换到 map/odom 坐标系下，
    维护一个全局锥桶地图，并发布给 RViz 和规划节点。
    另外保留 `/scan` / `PointCloud2` 作为可选旁路输入。

颜色怎么处理？
    2D 激光雷达看不到颜色。
    这里我们用简单启发式：
        - y > 0（车身左侧）的锥桶标为蓝色（blue，赛道通常左蓝右黄）
        - y < 0（车身右侧）的锥桶标为黄色（yellow）
    这不一定对，只是演示。如果要准确颜色，需要接相机或 sim_perception。

注意：
    这个节点依赖 EKF 节点发布的 TF（odom -> base_link）。
    如果输入是局部坐标系，还需要对应传感器 TF。
"""

import math
import struct
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray
from fsd_common_msgs.msg import ConeDetections, Cone
from tf2_ros import Buffer, TransformListener
from percep_localization.utils import quaternion_to_yaw


class ConeMapNode(Node):
    """
    全局锥桶地图节点。

    主要职责：
        1. 接收来自 sim_perception 的锥桶检测结果（ConeDetections）。
        2.（可选）接收原始 LaserScan / PointCloud2，自行聚类生成锥桶。
        3. 通过 TF 把锥桶从传感器/车身坐标系转换到 map_frame（默认 odom）。
        4. 维护并更新一个全局锥桶地图，对重复检测到的锥桶做加权平均。
        5. 发布可视化 MarkerArray、PoseArray 和 ConeDetections 供 RViz/规划使用。

    坐标系说明：
        - base_link：车体坐标系，x 向前，y 向左，z 向上。
        - odom/world/map：世界固定坐标系，这里默认用 odom 作为地图帧。
        - 如果输入消息的 header.frame_id 不是全局帧，则通过 lookup_transform
          查询 source_frame -> map_frame 的变换，把锥桶坐标转到 map_frame。
    """

    def __init__(self):
        super().__init__('cone_map_node')

        # ==================== 参数配置 ====================
        # 所有参数都可以在 launch 文件或命令行中覆盖
        self.declare_parameter('sim_cone_topic', '/perception/cones')
        self.declare_parameter('scan_topic', '')
        self.declare_parameter('pointcloud_topic', '')
        self.declare_parameter('map_frame', 'odom')
        self.declare_parameter('max_range', 10.0)        # 雷达最远距离
        self.declare_parameter('min_range', 0.2)         # 雷达最近距离
        self.declare_parameter('cluster_dist', 0.30)     # 聚类距离阈值（m）
        self.declare_parameter('min_points', 1)          # 一个簇最少点数
        self.declare_parameter('max_cluster_size', 0.45) # 锥桶簇最大宽度（m）
        self.declare_parameter('merge_dist', 0.5)        # 全局地图合并距离
        self.declare_parameter('publish_markers', True)
        self.declare_parameter('publish_pose_array', False)
        self.declare_parameter('publish_cone_detections', True)
        self.declare_parameter('lock_samples', 5)       # 锁定所需最少样本数
        self.declare_parameter('lock_variance', 0.04)    # 锁定方差阈值 (m²)，std < 0.2m
        self.declare_parameter('buffer_max_size', 10)    # 样本缓冲区最大长度

        self.map_frame = self.get_parameter('map_frame').value
        self.max_range = self.get_parameter('max_range').value
        self.min_range = self.get_parameter('min_range').value
        self.cluster_dist = self.get_parameter('cluster_dist').value
        self.min_points = self.get_parameter('min_points').value
        self.max_cluster_size = self.get_parameter('max_cluster_size').value
        self.merge_dist = self.get_parameter('merge_dist').value
        self.publish_markers = self.get_parameter('publish_markers').value
        self.publish_pose_array = self.get_parameter('publish_pose_array').value
        self.publish_cone_detections = self.get_parameter('publish_cone_detections').value
        self.lock_samples = self.get_parameter('lock_samples').value
        self.lock_variance = self.get_parameter('lock_variance').value
        self.buffer_max_size = self.get_parameter('buffer_max_size').value

        # ==================== TF 监听 ====================
        # 需要知道车辆在 map_frame 下的位姿，才能把局部锥桶坐标转到全局地图
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ==================== 订阅话题 ====================
        # 主输入：sim_perception 发布的锥桶检测结果（已经在 base_link 或相机坐标系下）
        self.create_subscription(
            ConeDetections,
            self.get_parameter('sim_cone_topic').value,
            self.sim_cone_callback,
            10
        )

        # 可选输入：原始 LaserScan（2D 雷达）
        # 如果 sim_perception 不可用或想用真实雷达，可以启用这个回调做聚类
        scan_topic = self.get_parameter('scan_topic').value
        if scan_topic:
            self.create_subscription(
                LaserScan,
                scan_topic,
                self.scan_callback,
                10
            )

        # 可选输入：原始 PointCloud2（三维点云）
        # 默认关闭（空字符串），需要时可在参数中指定 topic 名
        pointcloud_topic = self.get_parameter('pointcloud_topic').value
        if pointcloud_topic:
            self.create_subscription(
                PointCloud2,
                pointcloud_topic,
                self.pointcloud_callback,
                10
            )

        # ==================== 发布话题 ====================
        # 注意：cone_map_node 目前只作为 RViz 可视化旁路使用。
        # 规划组应继续直接订阅 /perception/cones，
        # 避免 /percep/cone_detections 覆盖主感知链路。
        self.marker_pub = (
            self.create_publisher(MarkerArray, '/percep/cone_map_markers', 10)
            if self.publish_markers else None
        )

        # PoseArray 发布，默认关闭
        self.pose_array_pub = (
            self.create_publisher(PoseArray, '/percep/cone_map', 10)
            if self.publish_pose_array else None
        )

        # ConeDetections 发布，默认关闭
        self.cone_pub = (
            self.create_publisher(ConeDetections, '/percep/cone_detections', 10)
            if self.publish_cone_detections else None
        )

        # ==================== 全局地图数据 ====================
        # 每个锥桶为字典:
        #   x, y:     锁定后位置（或当前临时最佳估计）
        #   color:    0=unknown, 1=blue, 2=yellow
        #   locked:   是否已锁定
        #   buffer:   [(x1,y1), ...] 最近样本
        #   lock_count: 锁定后被重复观测的次数
        self.global_map = []   # list of dict
        self._debug_counter = 0

        self.get_logger().info(
            f'锥桶地图节点已启动 | 锁定模式: {self.lock_samples}样本 '
            f'方差<{self.lock_variance}m² 即锁定')

    def sim_cone_callback(self, msg: ConeDetections):
        """处理 sim_perception 输出的锥桶信息。"""
        source_frame = (msg.header.frame_id or 'base_link').strip().lstrip('/')
        self.update_from_cones(msg.cone_detections, source_frame, msg.header.stamp)

    def scan_callback(self, msg: LaserScan):
        """
        LaserScan 回调（可选旁路）。

        处理流程：
            1. 把 scan 转成直角坐标点云（在 lidar_link/雷达坐标系下）。
            2. 交给 process_points 做聚类、坐标变换、地图更新和发布。
        """
        # LaserScan 转换为 numpy 数组 (N, 2)
        points_lidar = self.scan_to_points(msg)
        if len(points_lidar) == 0:
            return

        self.process_points(points_lidar, msg.header.frame_id, msg.header.stamp)

    def pointcloud_callback(self, msg: PointCloud2):
        """
        PointCloud2 回调（可选旁路）。

        处理流程：
            1. 从 PointCloud2 中提取 xy 坐标。
            2. 交给 process_points 做聚类、坐标变换、地图更新和发布。
        """
        # PointCloud2 转换为 numpy 数组 (N, 2)
        points = self.pointcloud2_to_points(msg)
        if len(points) == 0:
            return
        self.process_points(points, msg.header.frame_id, msg.header.stamp)

    def process_points(self, points_sensor, frame_id, stamp):
        """
        把传感器坐标系下的二维点聚类为锥桶，并更新全局地图。

        步骤：
            1. 对点云做欧氏距离聚类。
            2. 查询 frame_id -> map_frame 的 TF 变换。
            3. 对每个有效簇，取中心点并转到 map_frame。
            4. 根据簇在传感器 y 方向的位置猜测颜色（左蓝右黄）。
            5. 合并到全局地图并发布。
        """
        clusters = self.cluster_points(points_sensor)

        # 查询传感器坐标系到地图坐标系的变换
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except Exception as e:
            self.get_logger().warning(f'查不到 TF: {frame_id} -> {self.map_frame}: {e}')
            return

        # 提取变换的平移和航向
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        q = transform.transform.rotation
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

        new_cones = []
        for cluster in clusters:
            if len(cluster) < self.min_points:
                continue
            if not self.is_cone_cluster(cluster):
                continue

            # 簇中心（传感器坐标系下）
            cx = float(np.mean(cluster[:, 0]))
            cy = float(np.mean(cluster[:, 1]))

            # 2D 旋转 + 平移，转到 map_frame
            mx = tx + cx * math.cos(yaw) - cy * math.sin(yaw)
            my = ty + cx * math.sin(yaw) + cy * math.cos(yaw)

            # 2D 雷达无法识别颜色，按车身左侧蓝、右侧黄猜测
            color = 1 if cy > 0 else 2
            new_cones.append([mx, my, color])

        if not new_cones and not self.global_map:
            self.get_logger().debug('本帧没有形成有效锥桶簇，暂不发布空地图')
            return

        self.update_global_map(new_cones)
        self._debug_counter += 1
        if self._debug_counter % 10 == 0:
            self.get_logger().info(
                f'锥桶聚类: points={len(points_sensor)}, clusters={len(clusters)}, '
                f'new={len(new_cones)}, map={len(self.global_map)}'
            )
        self.publish_map(stamp)

    def update_from_cones(self, cones, frame_id, stamp):
        """
        把 ConeDetections 转换到 map_frame，并更新全局锥桶地图。

        说明：
            - 如果 ConeDetections 的 frame_id 已经是全局帧（odom/world/map），
              则直接使用其 position。
            - 否则通过 TF 把 position 从 source_frame 转到 map_frame。
            - 颜色优先使用消息中携带的 color 字段；缺失时按 local_y 猜测。
        """
        source_frame = frame_id or 'base_link'
        global_frames = {'odom', 'world', 'map'}
        tf_to_odom = None

        # 非全局帧时需要查 TF
        if source_frame not in global_frames:
            try:
                tf_to_odom = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    source_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
            except Exception as e:
                self.get_logger().warning(f'查不到 TF: {source_frame} -> {self.map_frame}: {e}')
                return

        new_cones = []
        for cone in cones:
            if tf_to_odom is None:
                # 输入已经在全局坐标系下
                mx = cone.position.x
                my = cone.position.y
            else:
                # 提取变换的平移和航向
                tx = tf_to_odom.transform.translation.x
                ty = tf_to_odom.transform.translation.y
                q = tf_to_odom.transform.rotation
                yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

                # 2D 旋转 + 平移
                mx = tx + cone.position.x * math.cos(yaw) - cone.position.y * math.sin(yaw)
                my = ty + cone.position.x * math.sin(yaw) + cone.position.y * math.cos(yaw)

            color = self.cone_color_to_id(cone.color, cone.position.y)
            new_cones.append([mx, my, color])

        if not new_cones and not self.global_map:
            self.get_logger().debug('sim_perception 没有输出有效锥桶，暂不发布空地图')
            return

        self.update_global_map(new_cones)
        self._debug_counter += 1
        if self._debug_counter % 10 == 0:
            self.get_logger().info(
                f'锥桶更新: sim_cones={len(cones)}, new={len(new_cones)}, map={len(self.global_map)}'
            )
        self.publish_map(stamp)

    def scan_to_points(self, msg: LaserScan):
        """
        把 LaserScan 转成 (N, 2) 的 numpy 点云数组。
        LaserScan 数据格式：
            ranges[i]: 第 i 个方向的距离
            angle = msg.angle_min + i * msg.angle_increment
            x = range * cos(angle)
            y = range * sin(angle)
        """
        points = []
        angle = msg.angle_min
        for r in msg.ranges:
            # 过滤无效点
            if self.min_range < r < self.max_range and not math.isinf(r) and not math.isnan(r):
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                points.append([x, y])
            angle += msg.angle_increment
        return np.array(points)

    def pointcloud2_to_points(self, msg: PointCloud2):
        """从 PointCloud2 中提取 xy 点，假设 x/y/z 字段为 float32。"""
        offsets = {field.name: field.offset for field in msg.fields}
        if 'x' not in offsets or 'y' not in offsets:
            self.get_logger().warning('PointCloud2 缺少 x/y 字段')
            return np.empty((0, 2))

        points = []
        step = msg.point_step
        data = msg.data
        for i in range(0, len(data), step):
            try:
                # 从数据中提取 x/y 坐标，offset 是字段在数据中的偏移量，单位是字节,为了数据对齐
                x = struct.unpack_from('f', data, i + offsets['x'])[0]
                y = struct.unpack_from('f', data, i + offsets['y'])[0]
            except struct.error:
                break
            if math.isnan(x) or math.isnan(y) or math.isinf(x) or math.isinf(y):
                continue
            r = math.hypot(x, y)
            if self.min_range < r < self.max_range:
                points.append([x, y])
        return np.array(points)

    def cluster_points(self, points):
        """
        简单欧氏距离聚类（ordered Euclidean clustering）。

        假设：
            雷达点按角度顺序排列，同一锥桶上的相邻点距离很近，
            不同锥桶之间的点距离会突然变大。
        因此遍历点云，相邻点距离小于 cluster_dist 就认为属于同一个簇。
        """
        if len(points) == 0:
            return []

        clusters = []
        current_cluster = [points[0]]

        for i in range(1, len(points)):
            dist = np.linalg.norm(points[i] - points[i - 1])
            if dist < self.cluster_dist:
                # 与上一个点属于同一簇
                current_cluster.append(points[i])
            else:
                # 距离突变，结束当前簇，开启新簇
                clusters.append(np.array(current_cluster))
                current_cluster = [points[i]]

        # 别忘了最后一个簇
        clusters.append(np.array(current_cluster))
        return clusters

    def is_cone_cluster(self, cluster):
        """
        根据簇的尺寸判断它是否可能是锥桶。

        规则：
            - 空簇：否
            - 单点簇：是（允许 1-2 个雷达点的小锥桶）
            - 簇直径 <= max_cluster_size：是
        """
        if len(cluster) == 0:
            return False
        if len(cluster) == 1:
            return True

        span_x = float(np.max(cluster[:, 0]) - np.min(cluster[:, 0]))
        span_y = float(np.max(cluster[:, 1]) - np.min(cluster[:, 1]))
        diameter = math.hypot(span_x, span_y)
        return diameter <= self.max_cluster_size

    def update_global_map(self, new_cones):
        """
        把新检测到的锥桶合并进全局地图。

        新策略（样本缓冲 + 方差锁定）：
            - 未锁定: 添加样本到 buffer，样本足够且方差低 → 锁定
            - 已锁定: lock_count += 1，位置不变
            - 无匹配: 创建新条目
        """
        for new_cone in new_cones:
            nx, ny, ncolor = new_cone
            merged = False
            for entry in self.global_map:
                dist = math.hypot(nx - entry['x'], ny - entry['y'])
                if dist < self.merge_dist:
                    merged = True
                    if entry['locked']:
                        entry['lock_count'] += 1
                    else:
                        entry['buffer'].append((nx, ny))
                        if len(entry['buffer']) > self.buffer_max_size:
                            entry['buffer'].pop(0)
                        # 用 buffer 均值作为临时位置
                        xs = [p[0] for p in entry['buffer']]
                        ys = [p[1] for p in entry['buffer']]
                        entry['x'] = sum(xs) / len(xs)
                        entry['y'] = sum(ys) / len(ys)
                        # 检查锁定条件
                        if len(entry['buffer']) >= self.lock_samples:
                            var_x = sum((v - entry['x']) ** 2 for v in xs) / len(xs)
                            var_y = sum((v - entry['y']) ** 2 for v in ys) / len(ys)
                            total_var = var_x + var_y
                            if total_var < self.lock_variance:
                                entry['locked'] = True
                                entry['lock_count'] = 1
                                self.get_logger().info(
                                    f'锥桶已锁定 at ({entry["x"]:.2f}, {entry["y"]:.2f}) '
                                    f'color={entry["color"]} var={total_var:.4f}')
                    break

            if not merged:
                self.global_map.append({
                    'x': nx, 'y': ny, 'color': ncolor,
                    'locked': False,
                    'buffer': [(nx, ny)],
                    'lock_count': 0,
                })

    @staticmethod
    def cone_color_to_id(color, local_y=0.0):
        """
        把颜色字符串转成内部颜色 ID。

        颜色 ID：
            1 = blue（蓝）
            2 = yellow（黄）
            3 = red（红，这里也映射到 yellow，因为目前只区分蓝/黄）
            缺失时根据 local_y 猜测：y > 0 为蓝，否则为黄
        """
        c = (color or '').lower().strip().strip("'\"")
        if c in ('blue', 'b'):
            return 1
        if c in ('yellow', 'y'):
            return 2
        if c in ('red', 'r'):
            return 2
        return 1 if local_y > 0.0 else 2

    def publish_map(self, stamp):
        """
        发布全局锥桶地图。

        根据参数开关，可能发布：
            - /percep/cone_map_markers（MarkerArray，RViz 可视化用）
            - /percep/cone_map（PoseArray）
            - /percep/cone_detections（ConeDetections）
        """
        marker_array = MarkerArray()
        pose_array = PoseArray()
        cone_msg = ConeDetections()
        pose_array.header.stamp = stamp
        pose_array.header.frame_id = self.map_frame
        cone_msg.header.stamp = stamp
        cone_msg.header.frame_id = self.map_frame

        # 只发布已锁定的锥桶
        locked_cones = [c for c in self.global_map if c['locked']]
        for i, cone in enumerate(locked_cones):
            x, y, color = cone['x'], cone['y'], cone['color']

            # Marker
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = self.map_frame
            marker.ns = 'cones'
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = 0.075  # 锥桶高 0.15m，中心在 0.075
            marker.scale.x = 0.15
            marker.scale.y = 0.15
            marker.scale.z = 0.15

            if color == 1:  # blue
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
                marker.color.a = 1.0
            elif color == 2:  # yellow
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
            else:  # unknown
                marker.color.r = 0.5
                marker.color.g = 0.5
                marker.color.b = 0.5
                marker.color.a = 1.0

            marker_array.markers.append(marker)

            # Pose
            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)

            cone_item = Cone()
            cone_item.position.x = float(x)
            cone_item.position.y = float(y)
            cone_item.position.z = 0.0
            if color == 1:
                cone_item.color = 'blue'
            elif color == 2:
                cone_item.color = 'yellow'
            else:
                cone_item.color = 'unknown'
            cone_item.pose_confidence = 1.0
            cone_item.color_confidence = 1.0
            cone_msg.cone_detections.append(cone_item)

        if self.marker_pub is not None:
            self.marker_pub.publish(marker_array)
        if self.pose_array_pub is not None:
            self.pose_array_pub.publish(pose_array)
        if self.cone_pub is not None:
            self.cone_pub.publish(cone_msg)

        self.get_logger().debug(f'全局锥桶地图: {len(self.global_map)} 个, 已锁定 {sum(1 for c in self.global_map if c["locked"])} 个')


def main(args=None):
    rclpy.init(args=args)
    node = ConeMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('锥桶地图节点被手动中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

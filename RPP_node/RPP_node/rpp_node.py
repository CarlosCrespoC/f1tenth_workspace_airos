#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math, csv
from pathlib import Path
from typing import List, Tuple, Optional
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry, Path as PathMsg
from geometry_msgs.msg import PoseStamped, Point, PoseWithCovarianceStamped
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy

BUILD_TAG = "rpp_node_pp_csv r8_adaptive_speed_dynamic_lookahead_debug"

def clamp(x, a, b): return max(a, min(b, x))
def yaw_from_quat(qx, qy, qz, qw):
    s = 2.0 * (qw * qz + qx * qy)
    c = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(s, c)
def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw/2.0), math.cos(yaw/2.0))

class WaypointFollower(Node):
    def __init__(self) -> None:
        super().__init__("rpp_follower")

        # ------------ Parámetros ------------
        self.declare_parameter("topic_odom", "/odom")
        self.declare_parameter("topic_cmd",  "/autonomous")
        
        # Parámetros de localización
        self.declare_parameter("use_amcl", True)
        self.declare_parameter("topic_amcl", "/amcl_pose")
        self.declare_parameter("amcl_timeout", 2.0)
        self.declare_parameter("amcl_priority", True)

        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("wheelbase", 0.33)
        self.declare_parameter("path_frame", "map")
        self.declare_parameter("loop_track", True)
        self.declare_parameter("LOOKAHEAD_DIST", 1.5)
        self.declare_parameter("v_cmd", 3.0)
        self.declare_parameter("steer_limit", 0.50)
        self.declare_parameter("ramp_rate", 2.0)
        self.declare_parameter("odom_timeout", 5.0)
        self.declare_parameter("local_path_len", 80)
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("waypoints_csv", "/tmp/waypoints.csv")
        self.declare_parameter("min_lap_time_s", 5.0)
        self.declare_parameter("cross_hi_frac", 0.75)
        self.declare_parameter("cross_lo_frac", 0.25)

        # Parámetros de velocidad adaptativa
        self.declare_parameter("adaptive_speed", True)
        self.declare_parameter("v_min", 0.5)
        self.declare_parameter("curvature_lookahead", 5.0)
        self.declare_parameter("curve_detection_points", 15)
        self.declare_parameter("speed_reduction_gain", 2.0)
        
        # NUEVO: Parámetros de lookahead dinámico
        self.declare_parameter("lookahead_min", 0.8)
        self.declare_parameter("lookahead_max", 2.5)
        self.declare_parameter("lookahead_gain", 0.4)

        self.declare_parameter("use_scan", True)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("avoid_enabled", True)
        self.declare_parameter("danger_dist", 2.5)
        self.declare_parameter("sector_deg", 150.0)
        self.declare_parameter("min_gap_deg", 12.0)
        self.declare_parameter("gap_margin", 0.4)
        self.declare_parameter("gap_mix_alpha", 0.5)
        self.declare_parameter("publish_avoid_markers", True)

        self.declare_parameter("publish_actual_path", True)
        self.declare_parameter("actual_path_len", 4000)
        self.declare_parameter("actual_path_frame", "")

        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("r1_button_index", 5)
        self.declare_parameter("x_button_index", 0)
        self.declare_parameter("autonomous_start", False)

        # ------------ Leer parámetros ------------
        g = self.get_parameter
        self.topic_odom: str = g("topic_odom").get_parameter_value().string_value
        self.topic_cmd:  str = g("topic_cmd").get_parameter_value().string_value
        
        self.use_amcl: bool = g("use_amcl").get_parameter_value().bool_value
        self.topic_amcl: str = g("topic_amcl").get_parameter_value().string_value
        self.amcl_timeout: float = g("amcl_timeout").get_parameter_value().double_value
        self.amcl_priority: bool = g("amcl_priority").get_parameter_value().bool_value

        self.rate_hz: float   = g("rate_hz").get_parameter_value().double_value
        self.L: float         = g("wheelbase").get_parameter_value().double_value
        self.path_frame: str  = g("path_frame").get_parameter_value().string_value
        self.loop_track: bool = g("loop_track").get_parameter_value().bool_value
        self.Ld: float        = g("LOOKAHEAD_DIST").get_parameter_value().double_value
        self.v_des: float     = g("v_cmd").get_parameter_value().double_value
        self.steer_limit: float = g("steer_limit").get_parameter_value().double_value
        self.ramp_rate: float   = g("ramp_rate").get_parameter_value().double_value
        self.odom_timeout: float = g("odom_timeout").get_parameter_value().double_value
        self.local_path_len: int  = g("local_path_len").get_parameter_value().integer_value
        self.publish_markers: bool = g("publish_markers").get_parameter_value().bool_value
        self.csv_path = Path(g("waypoints_csv").get_parameter_value().string_value).expanduser()
        self.min_lap_time_s: float = g("min_lap_time_s").get_parameter_value().double_value
        self.cross_hi_frac: float  = g("cross_hi_frac").get_parameter_value().double_value
        self.cross_lo_frac: float  = g("cross_lo_frac").get_parameter_value().double_value

        # Parámetros de velocidad adaptativa
        self.adaptive_speed: bool = g("adaptive_speed").get_parameter_value().bool_value
        self.v_min: float = g("v_min").get_parameter_value().double_value
        self.curvature_lookahead: float = g("curvature_lookahead").get_parameter_value().double_value
        self.curve_detection_points: int = g("curve_detection_points").get_parameter_value().integer_value
        self.speed_reduction_gain: float = g("speed_reduction_gain").get_parameter_value().double_value
        
        # NUEVO: Parámetros de lookahead dinámico
        self.lookahead_min: float = g("lookahead_min").get_parameter_value().double_value
        self.lookahead_max: float = g("lookahead_max").get_parameter_value().double_value
        self.lookahead_gain: float = g("lookahead_gain").get_parameter_value().double_value

        self.use_scan: bool      = g("use_scan").get_parameter_value().bool_value
        self.scan_topic: str     = g("scan_topic").get_parameter_value().string_value
        self.avoid_enabled: bool = g("avoid_enabled").get_parameter_value().bool_value
        self.danger_dist: float  = g("danger_dist").get_parameter_value().double_value
        self.sector_deg: float   = g("sector_deg").get_parameter_value().double_value
        self.min_gap_deg: float  = g("min_gap_deg").get_parameter_value().double_value
        self.gap_margin: float   = g("gap_margin").get_parameter_value().double_value
        self.gap_mix_alpha: float = g("gap_mix_alpha").get_parameter_value().double_value
        self.publish_avoid_markers: bool = g("publish_avoid_markers").get_parameter_value().bool_value

        self.publish_actual_path: bool = g("publish_actual_path").get_parameter_value().bool_value
        self.actual_path_len: int      = g("actual_path_len").get_parameter_value().integer_value
        self.actual_path_frame: str    = g("actual_path_frame").get_parameter_value().string_value

        self.joy_topic: str        = g("joy_topic").get_parameter_value().string_value
        self.r1_button_index: int  = g("r1_button_index").get_parameter_value().integer_value
        self.x_button_index: int   = g("x_button_index").get_parameter_value().integer_value
        self.autonomous_active: bool = g("autonomous_start").get_parameter_value().bool_value

        # ------------ Estado ------------
        self.actual_path = deque(maxlen=self.actual_path_len)
        
        self.have_odom = False
        self.last_odom_t = 0.0
        self.x_odom = self.y_odom = self.yaw_odom = 0.0
        
        self.have_amcl = False
        self.last_amcl_t = 0.0
        self.x_amcl = self.y_amcl = self.yaw_amcl = 0.0
        
        self.x = self.y = self.yaw = 0.0
        self.using_amcl = False
        
        self.v_pub = 0.0
        self.nearest_idx = 0
        self.prev_idx = 0
        self.lap_started = False
        self.lap_start_time = 0.0
        self.lap_times: List[float] = []
        self.best_lap: Optional[float] = None
        self.last_scan: Optional[LaserScan] = None
        self.scan_has_data: bool = False
        self._last_r1 = 0
        self._last_x = 0

        # ------------ Pub/Sub ------------
        qos_drive = QoSProfile(depth=10,
                               reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.VOLATILE)
        self.pub_cmd = self.create_publisher(AckermannDriveStamped, self.topic_cmd, qos_drive)

        latched_qos = QoSProfile(depth=1)
        latched_qos.history = HistoryPolicy.KEEP_LAST
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.pub_path_global = self.create_publisher(PathMsg, "/rpp/global_path", latched_qos)
        self.pub_markers     = self.create_publisher(MarkerArray, "/rpp/waypoints", latched_qos)
        self.pub_path_local  = self.create_publisher(PathMsg, "/rpp/local_path", 10)
        self.pub_avoid_markers = self.create_publisher(MarkerArray, "/rpp/avoidance", 10)
        self.pub_path_actual  = self.create_publisher(PathMsg, "/rpp/actual_path", 10)

        self.sub_odom = self.create_subscription(Odometry, self.topic_odom, self._odom_cb, 20)
        
        if self.use_amcl:
            self.sub_amcl = self.create_subscription(
                PoseWithCovarianceStamped, 
                self.topic_amcl, 
                self._amcl_cb, 
                10
            )
        
        if self.use_scan:
            self.sub_scan = self.create_subscription(LaserScan, self.scan_topic, self._scan_cb, 10)
        
        self.sub_joy = self.create_subscription(Joy, self.joy_topic, self._joy_cb, 10)

        # ------------ Cargar CSV ------------
        self.waypoints: List[Tuple[float, float]] = self._load_csv(self.csv_path)
        if len(self.waypoints) < 3:
            self.get_logger().error(f"No existe o insuficiente CSV: {self.csv_path} (≥3 puntos)")
        else:
            self._publish_global_path()
            if self.publish_markers:
                self._publish_waypoint_markers()

        self.timer = self.create_timer(1.0 / max(1.0, self.rate_hz), self._on_timer)
        self.timer_rep = self.create_timer(1.0, self._republish_static)

        loc_mode = "AMCL+Odom" if self.use_amcl else "Odom only"
        speed_mode = f"Adaptive ({self.v_min:.1f}-{self.v_des:.1f} m/s)" if self.adaptive_speed else f"Fixed ({self.v_des:.1f} m/s)"
        self.get_logger().info(
            f"{BUILD_TAG} | pub-> {self.topic_cmd} | sub<- {self.topic_odom}"
            f"{','+self.topic_amcl if self.use_amcl else ''},{self.scan_topic if self.use_scan else '∅'},{self.joy_topic} | "
            f"csv={self.csv_path} (pts={len(self.waypoints)}) | frame={self.path_frame} | "
            f"localization={loc_mode} | speed={speed_mode} | lookahead=dynamic({self.lookahead_min:.1f}-{self.lookahead_max:.1f}) | "
            f"auto={'ON' if self.autonomous_active else 'OFF'}"
        )

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        self.x_amcl = msg.pose.pose.position.x
        self.y_amcl = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw_amcl = yaw_from_quat(q.x, q.y, q.z, q.w)
        self.have_amcl = True
        self.last_amcl_t = self.get_clock().now().nanoseconds * 1e-9

        if self.publish_actual_path:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose = msg.pose.pose
            self.actual_path.append(ps)

    def _update_pose(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        
        amcl_ok = (self.use_amcl and self.have_amcl and (now - self.last_amcl_t) <= self.amcl_timeout)
        
        if amcl_ok:
            self.x = self.x_amcl
            self.y = self.y_amcl
            self.yaw = self.yaw_amcl
            self.using_amcl = True
            return True
        else:
            self.using_amcl = False
            return False

    def _joy_cb(self, msg: Joy):
        r1 = 1 if (self.r1_button_index < len(msg.buttons) and msg.buttons[self.r1_button_index] == 1) else 0
        x  = 1 if (self.x_button_index  < len(msg.buttons) and msg.buttons[self.x_button_index]  == 1) else 0
        
        if x == 1 and self._last_x == 0:
            if self.autonomous_active:
                self.autonomous_active = False
                self.get_logger().warn("⚠️ PARADA DE EMERGENCIA - Autonomía DESACTIVADA (X)")
            else:
                self.get_logger().info("Autonomía ya estaba desactivada (X)")
        
        if r1 == 1 and self._last_r1 == 0:
            self.autonomous_active = not self.autonomous_active
            self.get_logger().info(f"{'🟢 Autonomía ACTIVADA' if self.autonomous_active else '🔴 Autonomía DESACTIVADA'} (R1)")
        
        self._last_r1 = r1; self._last_x = x

    def _load_csv(self, p: Path) -> List[Tuple[float, float]]:
        pts: List[Tuple[float, float]] = []
        if not p.exists():
            self.get_logger().error(f"No existe el CSV: {p}")
            return pts
        with open(p, "r", newline="") as f:
            r = csv.reader(f)
            for row in r:
                if len(row) < 2: continue
                try:
                    x = float(row[0]); y = float(row[1])
                    pts.append((x, y))
                except Exception:
                    continue
        return pts

    def _make_path_msg(self, pts_xy: List[Tuple[float, float]]) -> PathMsg:
        path = PathMsg()
        path.header.frame_id = self.path_frame
        path.header.stamp = self.get_clock().now().to_msg()
        n = len(pts_xy)
        for i, (x, y) in enumerate(pts_xy):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(x); ps.pose.position.y = float(y)
            if i < n - 1:
                dx = pts_xy[i+1][0] - x; dy = pts_xy[i+1][1] - y
            else:
                dx = x - pts_xy[i-1][0] if n >= 2 else 1.0
                dy = y - pts_xy[i-1][1] if n >= 2 else 0.0
            yaw = math.atan2(dy, dx)
            qx, qy, qz, qw = quat_from_yaw(yaw)
            ps.pose.orientation.x = qx; ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz; ps.pose.orientation.w = qw
            path.poses.append(ps)
        return path

    def _publish_global_path(self):
        if len(self.waypoints) < 3: return
        self.pub_path_global.publish(self._make_path_msg(self.waypoints))

    def _publish_waypoint_markers(self):
        if len(self.waypoints) == 0: return
        arr = MarkerArray()
        delm = Marker(); delm.action = Marker.DELETEALL
        arr.markers.append(delm)
        m = Marker()
        m.header.frame_id = self.path_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "waypoints"; m.id = 0
        m.type = Marker.SPHERE_LIST; m.action = Marker.ADD
        m.scale.x = m.scale.y = m.scale.z = 0.12
        m.color.r = 0.0; m.color.g = 1.0; m.color.b = 1.0; m.color.a = 0.9
        for (x, y) in self.waypoints:
            pt = Point(); pt.x = float(x); pt.y = float(y); pt.z = 0.0
            m.points.append(pt)
        arr.markers.append(m)
        self.pub_markers.publish(arr)

    def _publish_avoidance_marker(self, xt: float, yt: float):
        if not self.publish_avoid_markers: return
        arr = MarkerArray()
        delm = Marker(); delm.action = Marker.DELETEALL
        arr.markers.append(delm)
        m = Marker()
        m.header.frame_id = self.path_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "avoidance"; m.id = 1
        m.type = Marker.SPHERE; m.action = Marker.ADD
        m.scale.x = m.scale.y = m.scale.z = 0.18
        m.color.r = 1.0; m.color.g = 0.0; m.color.b = 1.0; m.color.a = 0.9
        m.pose.position.x = float(xt); m.pose.position.y = float(yt); m.pose.position.z = 0.0
        arr.markers.append(m)
        self.pub_avoid_markers.publish(arr)

    def _republish_static(self):
        if len(self.waypoints) >= 3:
            self._publish_global_path()
            if self.publish_markers: self._publish_waypoint_markers()

    def _odom_cb(self, msg: Odometry):
        self.x_odom = msg.pose.pose.position.x
        self.y_odom = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw_odom = yaw_from_quat(q.x, q.y, q.z, q.w)
        self.have_odom = True
        self.last_odom_t = self.get_clock().now().nanoseconds * 1e-9
        if not self.lap_started and len(self.waypoints) >= 3:
            self.lap_started = True
            self.lap_start_time = self.last_odom_t

    def _scan_cb(self, msg: LaserScan):
        self.last_scan = msg; self.scan_has_data = True

    def _nearest_wp_index(self, x: float, y: float) -> int:
        if len(self.waypoints) == 0: return 0
        dmin = float("inf"); idx = 0
        for i, (wx, wy) in enumerate(self.waypoints):
            dx = wx - x; dy = wy - y
            d = dx*dx + dy*dy
            if d < dmin: dmin = d; idx = i
        return idx

    def _target_by_lookahead(self, idx0: int, Ld: float) -> Tuple[float, float, int]:
        if len(self.waypoints) == 0: return (self.x, self.y, idx0)
        n = len(self.waypoints); acc = 0.0; i = idx0
        while acc < Ld:
            j = (i + 1) % n if self.loop_track else min(i + 1, n - 1)
            dx = self.waypoints[j][0] - self.waypoints[i][0]
            dy = self.waypoints[j][1] - self.waypoints[i][1]
            seg = math.hypot(dx, dy)
            if seg <= 1e-6:
                if j == i: break
                i = j; continue
            acc += seg; i = j
            if (not self.loop_track) and i == n - 1: break
        return (*self.waypoints[i], i)

    def _pure_pursuit(self, xt: float, yt: float, Ld: float) -> float:
        dx = xt - self.x; dy = yt - self.y
        s = math.sin(-self.yaw); c = math.cos(-self.yaw)
        x_local = c*dx - s*dy
        y_local = s*dx + c*dy
        alpha = math.atan2(y_local, x_local + 1e-6)
        delta = math.atan2(2.0 * self.L * math.sin(alpha), max(Ld, 1e-3))
        return clamp(delta, -self.steer_limit, self.steer_limit)

    def _update_lap_counter(self, now_sec: float):
        if not self.loop_track or len(self.waypoints) < 8: return
        n = len(self.waypoints)
        hi_thr = int(self.cross_hi_frac * n)
        lo_thr = int(self.cross_lo_frac * n)
        crossed = (self.prev_idx >= hi_thr) and (self.nearest_idx <= lo_thr)
        if crossed and self.lap_started:
            lap_time = now_sec - self.lap_start_time
            if lap_time >= self.min_lap_time_s:
                self.lap_times.append(lap_time)
                lap_no = len(self.lap_times)
                loc_src = "AMCL" if self.using_amcl else "Odom"
                msg = f"🏁 Lap {lap_no:02d} | {lap_time:6.2f}s | loc={loc_src}"
                if self.best_lap is None or lap_time < self.best_lap:
                    self.best_lap = lap_time; msg += " | 🥇 best"
                self.get_logger().info(msg)
                self.lap_start_time = now_sec
        self.prev_idx = self.nearest_idx

    def _calculate_dynamic_lookahead(self, current_speed: float) -> float:
        """Calcula lookahead dinámico basado en velocidad actual."""
        Ld_dynamic = self.lookahead_min + self.lookahead_gain * current_speed
        return clamp(Ld_dynamic, self.lookahead_min, self.lookahead_max)

    def _calculate_path_curvature(self, idx: int) -> float:
        """Calcula curvatura promedio del camino adelante."""
        if len(self.waypoints) < 3:
            return 0.0
        
        n = len(self.waypoints)
        num_points = min(self.curve_detection_points, n)
        
        points = []
        acc_dist = 0.0
        i = idx
        
        while len(points) < num_points and acc_dist < self.curvature_lookahead:
            j = (i + 1) % n if self.loop_track else min(i + 1, n - 1)
            
            dx = self.waypoints[j][0] - self.waypoints[i][0]
            dy = self.waypoints[j][1] - self.waypoints[i][1]
            seg_len = math.hypot(dx, dy)
            
            if seg_len > 1e-6:
                points.append(self.waypoints[i])
                acc_dist += seg_len
            
            i = j
            if (not self.loop_track) and i == n - 1:
                break
        
        if len(points) < 3:
            return 0.0
        
        total_angle_change = 0.0
        total_distance = 0.0
        
        for k in range(len(points) - 2):
            p1, p2, p3 = points[k], points[k+1], points[k+2]
            
            v1x, v1y = p2[0] - p1[0], p2[1] - p1[1]
            v2x, v2y = p3[0] - p2[0], p3[1] - p2[1]
            
            d1 = math.hypot(v1x, v1y)
            d2 = math.hypot(v2x, v2y)
            
            if d1 < 1e-6 or d2 < 1e-6:
                continue
            
            dot = v1x * v2x + v1y * v2y
            cross = v1x * v2y - v1y * v2x
            angle_change = abs(math.atan2(cross, dot))
            
            total_angle_change += angle_change
            total_distance += (d1 + d2) / 2.0
        
        if total_distance < 1e-6:
            return 0.0
        
        curvature = total_angle_change / total_distance
        return curvature

    def _calculate_adaptive_speed(self, curvature: float) -> float:
        """Calcula velocidad objetivo basada en curvatura usando física."""
        if not self.adaptive_speed:
            return self.v_des
        
        if curvature < 0.02:
            return self.v_des
        
        curvature = min(curvature, 2.0)
        
        a_lat_max = 8.0  # Aceleración lateral máxima (m/s²)
        v_phys_max = math.sqrt(a_lat_max / max(curvature, 0.01))
        
        safety_factor = 0.8
        v_curve = v_phys_max * safety_factor / self.speed_reduction_gain
        
        target_speed = clamp(v_curve, self.v_min, self.v_des)
        
        return target_speed

    def _on_timer(self):
        """Loop de control principal con debug."""
        now = self.get_clock().now().nanoseconds * 1e-9
        pose_ok = self._update_pose()
        
        # Calcular velocidad adaptativa y lookahead dinámico
        if pose_ok and self.autonomous_active and len(self.waypoints) >= 3:
            self.nearest_idx = self._nearest_wp_index(self.x, self.y)
            curvature = self._calculate_path_curvature(self.nearest_idx)
            v_target = self._calculate_adaptive_speed(curvature)
            Ld_dynamic = self._calculate_dynamic_lookahead(self.v_pub)
        else:
            v_target = 0.0
            Ld_dynamic = self.Ld
        
        # Rampa de aceleración suave
        dv_max = self.ramp_rate / max(1.0, self.rate_hz)
        if self.v_pub < v_target: 
            self.v_pub = min(self.v_pub + dv_max, v_target)
        else:                      
            self.v_pub = max(self.v_pub - dv_max, v_target)

        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.drive.speed = float(self.v_pub)
        cmd.drive.steering_angle = 0.0

        if len(self.waypoints) < 3 or (not pose_ok):
            self.pub_cmd.publish(cmd)
            return

        self._update_lap_counter(now)
        xt, yt, _ = self._target_by_lookahead(self.nearest_idx, Ld_dynamic)

        # Variables de debug para Follow the Gap
        ftg_status = "DISABLED"
        dmin = float("inf")
        num_valid_points = 0
        
        # Follow the Gap con debug
        used_gap = False
        if self.use_scan and self.avoid_enabled and self.scan_has_data and (self.last_scan is not None):
            msg_scan = self.last_scan
            ftg_status = "ENABLED"
            
            s_idx, e_idx = self._front_sector_indices(msg_scan)
            
            if e_idx > s_idx:
                valid = [r for r in msg_scan.ranges[s_idx:e_idx+1] 
                        if not (math.isinf(r) or math.isnan(r))]
                num_valid_points = len(valid)
                dmin = min(valid) if valid else float("inf")
            else:
                dmin = float("inf")
                ftg_status = "BAD_SECTOR"
            
            # Detección de obstáculo
            if dmin < self.danger_dist:
                ftg_status = "OBSTACLE_DETECTED"
                gap = self._largest_safe_gap(msg_scan, s_idx, e_idx)
                
                if gap is not None:
                    ftg_status = "GAP_FOUND"
                    gs, ge = gap
                    xt_gap, yt_gap = self._gap_midpoint_target(msg_scan, gs, ge, Ld_dynamic)
                    
                    a = clamp(self.gap_mix_alpha, 0.0, 1.0)
                    xt_original = xt
                    yt_original = yt
                    xt = a * xt_gap + (1.0 - a) * xt
                    yt = a * yt_gap + (1.0 - a) * yt
                    used_gap = True
                    ftg_status = "AVOIDING"
                    
                    self.get_logger().warn(
                        f"🚧 FTG ACTIVE | dmin={dmin:.2f}m | gap=[{gs},{ge}] | "
                        f"shift: dx={xt-xt_original:.2f}, dy={yt-yt_original:.2f}",
                        throttle_duration_sec=0.3
                    )
                else:
                    ftg_status = "NO_SAFE_GAP"
                    self.get_logger().error(
                        f"⚠️ OBSTACLE but NO GAP! dmin={dmin:.2f}m",
                        throttle_duration_sec=0.5
                    )
        
        elif not self.use_scan:
            ftg_status = "SCAN_DISABLED"
        elif not self.scan_has_data:
            ftg_status = "NO_SCAN_DATA"
        
        # Log periódico cada 0.5s
        if int(now * 2) % 1 == 0:
            self.get_logger().info(
                f"FTG: {ftg_status} | pts={num_valid_points} | "
                f"dmin={dmin:.2f}m | thr={self.danger_dist:.2f}m | "
                f"v={self.v_pub:.2f} | Ld={Ld_dynamic:.2f}",
                throttle_duration_sec=0.5
            )
        
        # Publicar marcador de evasión
        if used_gap: 
            self._publish_avoidance_marker(xt, yt)
        else:
            if self.publish_avoid_markers:
                arr = MarkerArray()
                delm = Marker(); delm.action = Marker.DELETEALL
                arr.markers.append(delm)
                self.pub_avoid_markers.publish(arr)

        # Calcular steering y publicar
        delta = self._pure_pursuit(xt, yt, Ld_dynamic)
        cmd.drive.steering_angle = float(delta)
        
        # PUBLICAR a /autonomous
        self.pub_cmd.publish(cmd)
        
        # Log cuando FTG está activo
        if used_gap:
            self.get_logger().info(
                f"✅ Published to {self.topic_cmd} | v={cmd.drive.speed:.2f} | δ={delta:.3f}",
                throttle_duration_sec=0.3
            )

        # Publicar paths locales
        n = len(self.waypoints); L = min(self.local_path_len, n)
        if self.loop_track: 
            idxs = [(self.nearest_idx + k) % n for k in range(L)]
        else:               
            idxs = [min(self.nearest_idx + k, n - 1) for k in range(L)]
        pts_local = [self.waypoints[i] for i in idxs]
        self.pub_path_local.publish(self._make_path_msg(pts_local))

        if self.publish_actual_path and len(self.actual_path) > 1:
            pmsg = PathMsg()
            pmsg.header.frame_id = self.actual_path_frame if self.actual_path_frame else self.actual_path[-1].header.frame_id
            pmsg.header.stamp = self.get_clock().now().to_msg()
            pmsg.poses = list(self.actual_path)
            self.pub_path_actual.publish(pmsg)

    def _front_sector_indices(self, msg: LaserScan) -> Tuple[int, int]:
        half = math.radians(self.sector_deg) * 0.5
        a0 = msg.angle_min
        inc = msg.angle_increment if msg.angle_increment != 0.0 else 1e-3
        start = max(0, int(( -half - a0) / inc))
        end   = min(len(msg.ranges)-1, int(( +half - a0) / inc))
        return start, end

    def _largest_safe_gap(self, msg: LaserScan, start: int, end: int) -> Optional[Tuple[int, int]]:
        safe = self.danger_dist + self.gap_margin
        min_count = max(1, int(math.radians(self.min_gap_deg) / max(msg.angle_increment, 1e-6)))
        best_len, best_s, best_e = 0, None, None
        cur_s = None
        for i in range(start, end+1):
            r = msg.ranges[i]
            ok = (not math.isinf(r)) and (not math.isnan(r)) and (r >= safe)
            if ok:
                if cur_s is None: cur_s = i
            else:
                if cur_s is not None:
                    L = i - cur_s
                    if L >= min_count and L > best_len:
                        best_len, best_s, best_e = L, cur_s, i-1
                    cur_s = None
        if cur_s is not None:
            L = (end+1) - cur_s
            if L >= min_count and L > best_len:
                best_len, best_s, best_e = L, cur_s, end
        if best_s is None: return None
        return best_s, best_e

    def _gap_midpoint_target(self, msg: LaserScan, s_idx: int, e_idx: int, Ld: float) -> Tuple[float, float]:
        mid = (s_idx + e_idx) // 2
        ang = msg.angle_min + mid * msg.angle_increment
        xt_local = Ld * math.cos(ang)
        yt_local = Ld * math.sin(ang)
        c = math.cos(self.yaw); s = math.sin(self.yaw)
        xt = self.x + c*xt_local - s*yt_local
        yt = self.y + s*xt_local + c*yt_local
        return xt, yt


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = WaypointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

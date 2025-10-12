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

BUILD_TAG = "rpp_node_pp_csv r6_amcl"

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
        
        # NUEVO: Parámetros de localización
        self.declare_parameter("use_amcl", True)
        self.declare_parameter("topic_amcl", "/amcl_pose")
        self.declare_parameter("amcl_timeout", 2.0)
        self.declare_parameter("amcl_priority", True)  # True = AMCL prioritario, False = odometría prioritaria

        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("wheelbase", 0.33)
        self.declare_parameter("path_frame", "map")
        self.declare_parameter("loop_track", True)
        self.declare_parameter("LOOKAHEAD_DIST", 2.5)
        self.declare_parameter("v_cmd", 2.0)
        self.declare_parameter("steer_limit", 0.41)
        self.declare_parameter("ramp_rate", 2.0)
        self.declare_parameter("odom_timeout", 5.0)
        self.declare_parameter("local_path_len", 80)
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("waypoints_csv", "/tmp/waypoints.csv")
        self.declare_parameter("min_lap_time_s", 5.0)
        self.declare_parameter("cross_hi_frac", 0.75)
        self.declare_parameter("cross_lo_frac", 0.25)

        self.declare_parameter("use_scan", True)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("avoid_enabled", True)
        self.declare_parameter("danger_dist", 1.5)
        self.declare_parameter("sector_deg", 120.0)
        self.declare_parameter("min_gap_deg", 8.0)
        self.declare_parameter("gap_margin", 0.15)
        self.declare_parameter("gap_mix_alpha", 0.7)
        self.declare_parameter("publish_avoid_markers", True)

        self.declare_parameter("publish_actual_path", True)
        self.declare_parameter("actual_path_len", 4000)
        self.declare_parameter("actual_path_frame", "")

        # Gate por joystick
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("r1_button_index", 5)
        self.declare_parameter("x_button_index", 0)
        self.declare_parameter("autonomous_start", False)

        # ------------ Leer parámetros ------------
        g = self.get_parameter
        self.topic_odom: str = g("topic_odom").get_parameter_value().string_value
        self.topic_cmd:  str = g("topic_cmd").get_parameter_value().string_value
        
        # NUEVO: Localización
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
        
        # Odometría
        self.have_odom = False
        self.last_odom_t = 0.0
        self.x_odom = self.y_odom = self.yaw_odom = 0.0
        
        # AMCL
        self.have_amcl = False
        self.last_amcl_t = 0.0
        self.x_amcl = self.y_amcl = self.yaw_amcl = 0.0
        
        # Pose actual (fusionada)
        self.x = self.y = self.yaw = 0.0
        self.using_amcl = False  # Flag para saber qué fuente se está usando
        
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

        # Suscripciones
        self.sub_odom = self.create_subscription(Odometry, self.topic_odom, self._odom_cb, 20)
        
        # NUEVO: Subscribe a AMCL si está habilitado
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

        # ------------ Cargar CSV y publicar path global ------------
        self.waypoints: List[Tuple[float, float]] = self._load_csv(self.csv_path)
        if len(self.waypoints) < 3:
            self.get_logger().error(f"No existe o insuficiente CSV: {self.csv_path} (≥3 puntos)")
        else:
            self._publish_global_path()
            if self.publish_markers:
                self._publish_waypoint_markers()

        # Timers
        self.timer = self.create_timer(1.0 / max(1.0, self.rate_hz), self._on_timer)
        self.timer_rep = self.create_timer(1.0, self._republish_static)

        loc_mode = "AMCL+Odom" if self.use_amcl else "Odom only"
        self.get_logger().info(
            f"{BUILD_TAG} | pub-> {self.topic_cmd} | sub<- {self.topic_odom}"
            f"{','+self.topic_amcl if self.use_amcl else ''},{self.scan_topic if self.use_scan else '∅'},{self.joy_topic} | "
            f"csv={self.csv_path} (pts={len(self.waypoints)}) | frame={self.path_frame} | "
            f"localization={loc_mode} | auto={'ON' if self.autonomous_active else 'OFF'}"
        )

    # ------------ NUEVO: Callback AMCL ------------
    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        """Callback para recibir pose de AMCL"""
        self.x_amcl = msg.pose.pose.position.x
        self.y_amcl = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw_amcl = yaw_from_quat(q.x, q.y, q.z, q.w)
        self.have_amcl = True
        self.last_amcl_t = self.get_clock().now().nanoseconds * 1e-9

    # ------------ NUEVO: Fusión de localización ------------
    def _update_pose(self):
        """
        Actualiza la pose actual fusionando AMCL y odometría.
        Estrategia:
        - Si use_amcl=True y AMCL está disponible y no ha expirado -> usa AMCL
        - Si no, usa odometría
        """
        now = self.get_clock().now().nanoseconds * 1e-9
        
        # Verificar disponibilidad de AMCL
        amcl_ok = (self.use_amcl and 
                   self.have_amcl and 
                   (now - self.last_amcl_t) <= self.amcl_timeout)
        
        # Verificar disponibilidad de odometría
        odom_ok = self.have_odom and (now - self.last_odom_t) <= self.odom_timeout
        
        # Decidir qué fuente usar
        if self.amcl_priority:
            # Prioridad a AMCL
            if amcl_ok:
                self.x = self.x_amcl
                self.y = self.y_amcl
                self.yaw = self.yaw_amcl
                self.using_amcl = True
            elif odom_ok:
                self.x = self.x_odom
                self.y = self.y_odom
                self.yaw = self.yaw_odom
                self.using_amcl = False
            else:
                # Sin localización válida
                return False
        else:
            # Prioridad a odometría
            if odom_ok:
                self.x = self.x_odom
                self.y = self.y_odom
                self.yaw = self.yaw_odom
                self.using_amcl = False
            elif amcl_ok:
                self.x = self.x_amcl
                self.y = self.y_amcl
                self.yaw = self.yaw_amcl
                self.using_amcl = True
            else:
                return False
        
        return True

    # ------------ Joy gate ------------
    def _joy_cb(self, msg: Joy):
        r1 = 1 if (self.r1_button_index < len(msg.buttons) and msg.buttons[self.r1_button_index] == 1) else 0
        x  = 1 if (self.x_button_index  < len(msg.buttons) and msg.buttons[self.x_button_index]  == 1) else 0

        if r1 == 1 and self._last_r1 == 0:
            self.autonomous_active = not self.autonomous_active
            self.get_logger().info(f"Autonomía {'ACTIVADA' if self.autonomous_active else 'DESACTIVADA'} (R1)")

        if x == 1 and self._last_x == 0:
            if self.autonomous_active:
                self.autonomous_active = False
                self.get_logger().info("Autonomía DESACTIVADA (X)")

        self._last_r1 = r1
        self._last_x = x

    # ------------ Utilidades ------------
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
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
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

    # ------------ Callbacks ------------
    def _odom_cb(self, msg: Odometry):
        """Callback para odometría (ahora guarda en variables separadas)"""
        self.x_odom = msg.pose.pose.position.x
        self.y_odom = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw_odom = yaw_from_quat(q.x, q.y, q.z, q.w)
        self.have_odom = True
        self.last_odom_t = self.get_clock().now().nanoseconds * 1e-9

        if not self.lap_started and len(self.waypoints) >= 3:
            self.lap_started = True
            self.lap_start_time = self.last_odom_t

        if self.publish_actual_path:
            ps = PoseStamped(); ps.header = msg.header; ps.pose = msg.pose.pose
            self.actual_path.append(ps)

    def _scan_cb(self, msg: LaserScan):
        self.last_scan = msg; self.scan_has_data = True

    # ------------ Helpers de control ------------
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

    # ------------ Loop principal ------------
    def _on_timer(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        
        # NUEVO: Actualizar pose fusionada
        pose_ok = self._update_pose()

        # Rampa de velocidad
        v_target = (self.v_des if (pose_ok and self.autonomous_active) else 0.0)
        dv_max = self.ramp_rate / max(1.0, self.rate_hz)
        if self.v_pub < v_target: self.v_pub = min(self.v_pub + dv_max, v_target)
        else:                      self.v_pub = max(self.v_pub - dv_max, v_target)

        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.drive.speed = float(self.v_pub)
        cmd.drive.steering_angle = 0.0

        if len(self.waypoints) < 3 or (not pose_ok):
            self.pub_cmd.publish(cmd)
            return

        # 1) waypoint más cercano
        self.nearest_idx = self._nearest_wp_index(self.x, self.y)

        # 2) cronometraje
        self._update_lap_counter(now)

        # 3) objetivo por lookahead
        xt, yt, _ = self._target_by_lookahead(self.nearest_idx, self.Ld)

        # 4) evitación simple con LIDAR
        used_gap = False
        if self.use_scan and self.avoid_enabled and self.scan_has_data and (self.last_scan is not None):
            msg = self.last_scan
            s_idx, e_idx = self._front_sector_indices(msg)
            if e_idx > s_idx:
                valid = [r for r in msg.ranges[s_idx:e_idx+1] if not (math.isinf(r) or math.isnan(r))]
                dmin = min(valid) if valid else float("inf")
            else:
                dmin = float("inf")
            if dmin < self.danger_dist:
                gap = self._largest_safe_gap(msg, s_idx, e_idx)
                if gap is not None:
                    gs, ge = gap
                    xt_gap, yt_gap = self._gap_midpoint_target(msg, gs, ge, self.Ld)
                    a = clamp(self.gap_mix_alpha, 0.0, 1.0)
                    xt = a * xt_gap + (1.0 - a) * xt
                    yt = a * yt_gap + (1.0 - a) * yt
                    used_gap = True

        if used_gap: self._publish_avoidance_marker(xt, yt)
        else:
            if self.publish_avoid_markers:
                arr = MarkerArray()
                delm = Marker(); delm.action = Marker.DELETEALL
                arr.markers.append(delm)
                self.pub_avoid_markers.publish(arr)

        # 5) pure pursuit
        delta = self._pure_pursuit(xt, yt, self.Ld)

        # 6) publicar
        cmd.drive.steering_angle = float(delta)
        self.pub_cmd.publish(cmd)

        # 7) path local
        n = len(self.waypoints); L = min(self.local_path_len, n)
        if self.loop_track: idxs = [(self.nearest_idx + k) % n for k in range(L)]
        else:               idxs = [min(self.nearest_idx + k, n - 1) for k in range(L)]
        pts_local = [self.waypoints[i] for i in idxs]
        self.pub_path_local.publish(self._make_path_msg(pts_local))

        # 8) trayectoria real
        if self.publish_actual_path and len(self.actual_path) > 1:
            pmsg = PathMsg()
            pmsg.header.frame_id = self.actual_path_frame if self.actual_path_frame else self.actual_path[-1].header.frame_id
            pmsg.header.stamp = self.get_clock().now().to_msg()
            pmsg.poses = list(self.actual_path)
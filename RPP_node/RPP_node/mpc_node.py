#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# MPC follower (ROS 2, OSQP) para F1TENTH — control + predicción + LiDAR Propagation
#- Speed gate frontal (v_max por distancia libre)
#- Propagation: offset lateral por LiDAR (conos laterales) propagado en el horizonte
#- Visualización de ref base y ref desplazada

import math
import csv
import time
from pathlib import Path as FsPath
from typing import Tuple, List, Optional

import numpy as np
from numpy.linalg import norm

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
)

from nav_msgs.msg import Odometry, Path as PathMsg
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32

import osqp
from scipy import sparse


def wrap_angle(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _latched_qos() -> QoSProfile:
    qos = QoSProfile(depth=1)
    qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    qos.reliability = QoSReliabilityPolicy.RELIABLE
    qos.history = QoSHistoryPolicy.KEEP_LAST
    return qos


class MPCFollowerOSQP(Node):
    def __init__(self) -> None:
        super().__init__("MPC")

        # ---------- parámetros básicos ----------
        self.declare_parameter("waypoints_csv", "BrandsHatch_map1_Dijkstra_1p0m_BSpline_smooth.csv")
        self.declare_parameter("topic_odom", "/ego_racecar/odom")
        self.declare_parameter("topic_cmd", "/drive")
        self.declare_parameter("loop_track", True)

        # Modelo y sampleo temporal
        self.declare_parameter("L", 0.33)
        self.declare_parameter("dt", 0.02)

        # Perfil global base (luego se refina por horizonte)
        self.declare_parameter("ds_ref", 0.20)
        self.declare_parameter("v_top", 2.2)
        self.declare_parameter("a_lat_max", 3.7)
        self.declare_parameter("a_x_max", 2.0)
        self.declare_parameter("a_x_min", -5.0)

        # Horizonte y límites
        self.declare_parameter("N", 12)
        self.declare_parameter("a_min", -4.0)
        self.declare_parameter("a_max", 4.0)
        self.declare_parameter("w_min", -2.5)
        self.declare_parameter("w_max", 2.5)
        self.declare_parameter("da_min", -4.0)
        self.declare_parameter("da_max", 4.0)
        self.declare_parameter("dw_min", -1.6)
        self.declare_parameter("dw_max", 1.6)

        # Costos
        self.declare_parameter("Q_perp", 8.0)
        self.declare_parameter("Q_psi", 3.0)
        self.declare_parameter("Q_v", 0.6)
        self.declare_parameter("R_a", 0.25)
        self.declare_parameter("R_w", 0.9)
        self.declare_parameter("Rd_a", 0.05)
        self.declare_parameter("Rd_w", 0.20)
        self.declare_parameter("Qf_gain", 3.0)

        # Feed-forward
        self.declare_parameter("use_feedforward", True)
        self.declare_parameter("kappa_sign", 1.0)
        self.declare_parameter("ff_clip_ratio", 0.8)

        # Filtro de dirección
        self.declare_parameter("steer_alpha", 0.22)

        # Visual
        self.declare_parameter("pred_start_at_car", True)

        # ---------- boost rectas + LiDAR ----------
        self.declare_parameter("use_straight_boost", False)
        self.declare_parameter("v_top_straight", 14.0)
        self.declare_parameter("straight_window_m", 3.0)
        self.declare_parameter("straight_yaw_span_deg", 5.0)
        self.declare_parameter("straight_kappa_max", 0.04)
        self.declare_parameter("straight_hyst_m", 1.5)

        self.declare_parameter("use_lidar_gate", True)
        self.declare_parameter("lidar_topic", "/scan")
        self.declare_parameter("lidar_window_deg", 8.0)
        self.declare_parameter("a_brake", 6.0)
        self.declare_parameter("stop_margin", 0.4)

        self.declare_parameter("lidar_percentile", 10.0)
        self.declare_parameter("lidar_min_valid", 0.05)
        self.declare_parameter("prop_bias_gain", 0.20)

        # ---------- Propagation (LiDAR) ----------
        self.declare_parameter("use_propagation", True)
        self.declare_parameter("lidar_front_center_deg", 0.0)
        self.declare_parameter("lidar_debug", True)

        self.declare_parameter("prop_front_trigger_m", 6.0)
        self.declare_parameter("prop_side_min_deg", 25.0)
        self.declare_parameter("prop_side_max_deg", 85.0)
        self.declare_parameter("prop_offset_max_m", 0.70)
        self.declare_parameter("prop_gain", 0.25)
        self.declare_parameter("prop_alpha", 0.35)
        self.declare_parameter("prop_decay", 0.88)

        self.declare_parameter("use_strategic_cornering", True)
        self.declare_parameter("corner_kappa_threshold", 0.25)
        self.declare_parameter("corner_offset_m", 1.0)

        # ---------- cronómetro / logs ----------
        self.declare_parameter("lap_origin_x", 0.0)
        self.declare_parameter("lap_origin_y", 0.0)
        self.declare_parameter("lap_radius", 0.5)
        self.declare_parameter("lap_partial_period_s", 1.0)

        self.declare_parameter("traj_enable", True)
        self.declare_parameter("traj_publish_incremental", True)
        self.declare_parameter("traj_decimate", 5)
        self.declare_parameter("traj_csv_path", "Ruta.csv")

        # ---------- depuración OSQP ----------
        self.declare_parameter("osqp_debug", False)
        self.declare_parameter("eps_bounds", 1e-9)

        # ---------- lee parámetros ----------
        self.csv_path = FsPath(str(self.get_parameter("waypoints_csv").value))
        self.topic_odom = str(self.get_parameter("topic_odom").value)
        self.topic_cmd = str(self.get_parameter("topic_cmd").value)
        self.loop_track = bool(self.get_parameter("loop_track").value)

        self.L = float(self.get_parameter("L").value)
        self.dt = float(self.get_parameter("dt").value)
        self.N = int(self.get_parameter("N").value)

        self.osqp_debug = bool(self.get_parameter("osqp_debug").value)
        self.eps_bounds = float(self.get_parameter("eps_bounds").value)

        self.use_lidar_gate = bool(self.get_parameter("use_lidar_gate").value)
        self.lidar_topic = str(self.get_parameter("lidar_topic").value)
        self.lidar_window_deg = float(self.get_parameter("lidar_window_deg").value)
        self.a_brake = float(self.get_parameter("a_brake").value)
        self.stop_margin = float(self.get_parameter("stop_margin").value)
        self.lidar_percentile = float(self.get_parameter("lidar_percentile").value)
        self.lidar_min_valid = float(self.get_parameter("lidar_min_valid").value)

        self.use_propagation = bool(self.get_parameter("use_propagation").value)
        self.lidar_front_center = math.radians(float(self.get_parameter("lidar_front_center_deg").value))
        self.lidar_debug = bool(self.get_parameter("lidar_debug").value)
        self.prop_front_trigger_m = float(self.get_parameter("prop_front_trigger_m").value)
        self.prop_side_min_deg = float(self.get_parameter("prop_side_min_deg").value)
        self.prop_side_max_deg = float(self.get_parameter("prop_side_max_deg").value)
        self.prop_offset_max_m = float(self.get_parameter("prop_offset_max_m").value)
        self.prop_gain = float(self.get_parameter("prop_gain").value)
        self.prop_alpha = float(self.get_parameter("prop_alpha").value)
        self.prop_decay = float(self.get_parameter("prop_decay").value)

        # Estado del vehículo
        self.have_state = False
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v_meas = 0.0
        self.u_prev = np.zeros((2, 1))
        self.delta_prev = 0.0
        self._U_last: Optional[np.ndarray] = None

        # LiDAR buffers
        self.have_scan = False
        self.scan_ang: Optional[np.ndarray] = None
        self.scan_r: Optional[np.ndarray] = None
        self.front_clear = float("inf")
        self._prop_offset = 0.0
        self._scan_info_logged = False

        # Cronómetro
        self.lap_origin = (
            float(self.get_parameter("lap_origin_x").value),
            float(self.get_parameter("lap_origin_y").value),
        )
        self.lap_radius = float(self.get_parameter("lap_radius").value)
        self.partial_period = float(self.get_parameter("lap_partial_period_s").value)
        self.lap_count = -1
        self._lap_gate_inside = False
        self.lap_start_time = time.time()
        self.last_partial_t = time.time()
        self.best_lap_s = None

        # Trayectoria vuelta 0
        self.traj_enable = bool(self.get_parameter("traj_enable").value)
        self.traj_pub_incr = bool(self.get_parameter("traj_publish_incremental").value)
        self.traj_decimate = int(self.get_parameter("traj_decimate").value)
        self.traj_csv_path = str(self.get_parameter("traj_csv_path").value)
        self._traj_done = False
        self._traj_gate = False
        self._traj_dec_ctr = 0
        self.traj_path_msg = PathMsg()
        self.traj_path_msg.header.frame_id = "map"

        # Publishers / Subscribers
        self.pub_traj_path = self.create_publisher(PathMsg, "/mpc/lap1_path", _latched_qos())
        xy = self._load_csv_xy(self.csv_path)
        self.s_grid, self.cx, self.cy, self.cpsi, self.ckappa = self._resample_xy(
            xy, float(self.get_parameter("ds_ref").value)
        )

        self.sub_odom = self.create_subscription(Odometry, self.topic_odom, self._odom_cb, 20)
        if self.use_lidar_gate or self.use_propagation:
            self.sub_scan = self.create_subscription(LaserScan, self.lidar_topic, self._scan_cb, 30)

        self.pub_cmd = self.create_publisher(AckermannDriveStamped, self.topic_cmd, 10)
        self.pub_ref = self.create_publisher(PathMsg, "/mpc/local_ref", 1)
        self.pub_ref_raw = self.create_publisher(PathMsg, "/mpc/local_ref_raw", 1)
        self.pub_pred = self.create_publisher(PathMsg, "/mpc/predicted_path", 1)
        self.pub_prop_offset = self.create_publisher(Float32, "/mpc/propagation_offset", 1)
        self.pub_front_clear = self.create_publisher(Float32, "/mpc/front_clear", 1)

        self.timer = self.create_timer(self.dt, self.control_step)
        self.get_logger().info(
            f"MPC(OSQP) listo | N={self.N}, dt={self.dt:.3f}s, CSV={self.csv_path}"
        )
        self.get_logger().info("Vuelta 0 (inicio). Cronómetro arrancado.")

    def _load_csv_xy(self, path: FsPath) -> np.ndarray:
        if not path.exists():
            raise FileNotFoundError(f"{path} no existe")
        pts: List[Tuple[float, float]] = []
        with open(path, "r", newline="") as f:
            reader = csv.reader(f)
            for row in reader:
                if not row:
                    continue
                try:
                    x = float(row[0])
                    y = float(row[1])
                    pts.append((x, y))
                except Exception:
                    continue
        if len(pts) < 5:
            raise RuntimeError(f"Waypoints insuficientes en {path} (len={len(pts)})")
        return np.asarray(pts, dtype=float)

    def _resample_xy(self, xy: np.ndarray, ds: float):
        dxy = np.diff(xy, axis=0)
        seg = np.hypot(dxy[:, 0], dxy[:, 1])
        s = np.insert(np.cumsum(seg), 0, 0.0)
        if self.loop_track and norm(xy[-1] - xy[0]) > 1e-6:
            xy = np.vstack([xy, xy[0]])
            s = np.append(s, s[-1] + norm(xy[-1] - xy[-2]))

        s_end = s[-1]
        n_new = max(5, int(math.ceil(s_end / ds)))
        s_new = np.linspace(0.0, s_end, n_new)
        x_new = np.interp(s_new, s, xy[:, 0])
        y_new = np.interp(s_new, s, xy[:, 1])

        dx = np.gradient(x_new, s_new, edge_order=2)
        dy = np.gradient(y_new, s_new, edge_order=2)
        ddx = np.gradient(dx, s_new, edge_order=2)
        ddy = np.gradient(dy, s_new, edge_order=2)
        psi = np.arctan2(dy, dx)

        for i in range(1, len(psi)):
            psi[i] = psi[i - 1] + wrap_angle(psi[i] - psi[i - 1])

        kappa = (dx * ddy - dy * ddx) / np.maximum((dx * dx + dy * dy) ** 1.5, 1e-6)

        window_size = 7
        kernel = np.ones(window_size) / window_size
        kappa_smooth = np.convolve(kappa, kernel, mode='same')

        if self.loop_track:
            for i in range(window_size):
                blend = i / window_size
                kappa_smooth[i] = (1.0 - blend) * kappa_smooth[i] + blend * kappa_smooth[len(kappa) - window_size + i]
                kappa_smooth[len(kappa) - 1 - i] = (1.0 - blend) * kappa_smooth[len(kappa) - 1 - i] + blend * kappa_smooth[window_size - 1 - i]

        kappa = kappa_smooth
        return s_new, x_new, y_new, psi, kappa

    def _odom_cb(self, msg: Odometry) -> None:
        self.x = float(msg.pose.pose.position.x)
        self.y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        self.v_meas = float(msg.twist.twist.linear.x)
        self.have_state = True
        self._lap_logic()
        self._lap_partial()
        self._traj_record_step()

    def _scan_cb(self, msg: LaserScan) -> None:
        n = len(msg.ranges)
        if n == 0:
            self.front_clear = float("inf")
            self.have_scan = False
            return

        ang = msg.angle_min + np.arange(n) * msg.angle_increment
        rr = np.asarray(msg.ranges, dtype=float)

        if self.lidar_debug and not self._scan_info_logged:
            self.get_logger().info(
                f"[LiDAR] angle_min={math.degrees(msg.angle_min):.1f}°, "
                f"angle_max={math.degrees(msg.angle_max):.1f}°, "
                f"dθ={math.degrees(msg.angle_increment):.3f}° | "
                f"front_center={math.degrees(self.lidar_front_center):.1f}°, "
                f"win=±{self.lidar_window_deg:.1f}°"
            )
            self._scan_info_logged = True

        rr_min_valid = self.lidar_min_valid
        finite = np.isfinite(rr) & (rr >= rr_min_valid)
        ang = ang[finite]
        rr = rr[finite]

        win = math.radians(self.lidar_window_deg)
        rel = wrap_angle(ang - self.lidar_front_center)
        mask_front = (np.abs(rel) <= win)
        rr_front = rr[mask_front]

        p = self.lidar_percentile
        if rr_front.size:
            self.front_clear = float(np.percentile(rr_front, p))
        else:
            self.front_clear = float(np.percentile(rr, min(p, 50.0)))

        self.pub_front_clear.publish(Float32(data=self.front_clear))

        self.scan_ang = ang
        self.scan_r = rr
        self.have_scan = True

    def _lap_logic(self) -> None:
        dx = self.x - self.lap_origin[0]
        dy = self.y - self.lap_origin[1]
        inside = (dx * dx + dy * dy) <= (self.lap_radius * self.lap_radius)
        if inside and not self._lap_gate_inside:
            now = time.time()
            lap_time = now - self.lap_start_time
            self.lap_count += 1
            if self.lap_count > 0:
                mm = int(lap_time // 60)
                ss = lap_time % 60.0
                txt = f"Vuelta {self.lap_count} en {mm}:{ss:05.2f}"
                if self.best_lap_s is None or lap_time < self.best_lap_s:
                    self.best_lap_s = lap_time
                    txt += "  (¡Mejor vuelta!)"
                self.get_logger().info(txt)
            else:
                self.get_logger().info("Vuelta 0 (inicio)")

            self.lap_start_time = now
            if self.traj_enable and not self._traj_done and self.lap_count == 1:
                self.traj_path_msg.header.stamp = self.get_clock().now().to_msg()
                self.pub_traj_path.publish(self.traj_path_msg)
                try:
                    with open(self.traj_csv_path, "w", newline="") as f:
                        w = csv.writer(f)
                        w.writerow(["x", "y", "yaw_rad", "t_sec"])
                        for ps in self.traj_path_msg.poses:
                            x = ps.pose.position.x
                            y = ps.pose.position.y
                            z = ps.pose.orientation.z
                            wq = ps.pose.orientation.w
                            yaw = 2.0 * math.atan2(z, wq)
                            t = ps.header.stamp.sec + 1e-9 * ps.header.stamp.nanosec
                            w.writerow([f"{x:.6f}", f"{y:.6f}", f"{yaw:.6f}", f"{t:.6f}"])
                    self.get_logger().info(f"[TRAZA] {self.traj_csv_path} guardado con {len(self.traj_path_msg.poses)} puntos.")
                except Exception as e:
                    self.get_logger().error(f"[TRAZA] Error al escribir {self.traj_csv_path}: {e}")
                self._traj_done = True
        self._lap_gate_inside = inside

    def _lap_partial(self) -> None:
        # if self.lap_count < 0: return
        now = time.time()
        if now - self.last_partial_t >= self.partial_period:
            t = now - self.lap_start_time
            mm = int(t // 60)
            ss = t % 60.0
            v = self.v_meas
            best = "-" if self.best_lap_s is None else f"{int(self.best_lap_s // 60)}:{self.best_lap_s % 60:05.2f}"
            self.get_logger().info(f"Parcial V{self.lap_count}: {mm}:{ss:05.2f} | v={v:.2f} m/s | best={best}")
            self.last_partial_t = now

    def _traj_record_step(self) -> None:
        if not self.traj_enable or self._traj_done or self.lap_count != 0:
            return
        dx = self.x - self.lap_origin[0]
        dy = self.y - self.lap_origin[1]
        inside = (dx * dx + dy * dy) <= (self.lap_radius * self.lap_radius)
        if not inside:
            self._traj_gate = True
        if not self._traj_gate:
            return

        self._traj_dec_ctr += 1
        if self._traj_dec_ctr < max(1, self.traj_decimate):
            return
        self._traj_dec_ctr = 0

        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(self.x)
        ps.pose.position.y = float(self.y)
        ps.pose.orientation.z = math.sin(self.yaw * 0.5)
        ps.pose.orientation.w = math.cos(self.yaw * 0.5)
        self.traj_path_msg.poses.append(ps)

        if self.traj_pub_incr:
            self.traj_path_msg.header.stamp = ps.header.stamp
            self.pub_traj_path.publish(self.traj_path_msg)

    def _nearest_index(self, x: float, y: float) -> int:
        dx = self.cx - x
        dy = self.cy - y
        return int(np.argmin(dx * dx + dy * dy))

    def _is_straight_ahead(self, idx0: int) -> bool:
        ds = float(self.get_parameter("ds_ref").value)
        win_pts = max(3, int(round(self.get_parameter("straight_window_m").value / max(ds, 1e-3))))
        M = len(self.cx)
        yaws = []
        kapp = []
        i = idx0
        for _ in range(win_pts):
            yaws.append(self.cpsi[i])
            kapp.append(abs(self.ckappa[i]))
            i = (i + 1) % M if self.loop_track else min(i + 1, M - 1)

        yaws = np.unwrap(np.array(yaws))
        yaw_span = float(np.max(yaws) - np.min(yaws))
        kmax = float(np.max(kapp))
        yaw_span_deg_limit = math.radians(float(self.get_parameter("straight_yaw_span_deg").value))
        kappa_max_limit = float(self.get_parameter("straight_kappa_max").value)

        straight_now = (yaw_span <= yaw_span_deg_limit) and (kmax <= kappa_max_limit)

        if getattr(self, "_straight_latch", False):
            if straight_now:
                self._straight_latch = True
            else:
                loosen = 1.3
                self._straight_latch = (yaw_span <= loosen * yaw_span_deg_limit) and (kmax <= loosen * kappa_max_limit)
        else:
            self._straight_latch = straight_now

        return bool(self._straight_latch)

    def _build_horizon(self, idx0: int):
        N = self.N
        dt = self.dt
        ds = float(self.get_parameter("ds_ref").value)
        xref, yref, psiref, vref, kapp = [], [], [], [], []
        i = idx0
        M = len(self.cx)
        for _ in range(N):
            xref.append(self.cx[i])
            yref.append(self.cy[i])
            psiref.append(self.cpsi[i])
            vref.append(0.0)
            kapp.append(self.ckappa[i])
            step = max(1, int(round(max(0.1, self.v_meas) * dt / max(ds, 1e-3))))
            i = (i + step) % M if self.loop_track else min(i + step, M - 1)
        return (np.asarray(xref), np.asarray(yref), np.asarray(psiref),
                np.asarray(vref), np.asarray(kapp))

    def _error_now(self, xref: float, yref: float, psiref: float, vref: float):
        nx = -math.sin(psiref)
        ny = math.cos(psiref)
        ex = self.x - xref
        ey = self.y - yref
        e_perp = nx * ex + ny * ey
        e_psi = wrap_angle(self.yaw - psiref)
        e_v = self.v_meas - vref
        return np.array([e_perp, e_psi, e_v], dtype=float)

    def _condense(self, A_list, B_list, d_list):
        n, m, N = 3, 2, self.N
        Sx_rows, Su_rows, c_rows = [], [], []

        Sx_prev = A_list[0]
        Su_prev = np.zeros((n, N * m))
        Su_prev[:, 0:m] = B_list[0]
        c_prev = d_list[0]

        Sx_rows.append(Sx_prev)
        Su_rows.append(Su_prev.copy())
        c_rows.append(c_prev.copy())

        for k in range(1, N):
            A_k, B_k, d_k = A_list[k], B_list[k], d_list[k]
            Sx_curr = A_k @ Sx_prev
            Su_curr = A_k @ Su_prev
            Su_curr[:, k * m:(k + 1) * m] += B_k
            c_curr = A_k @ c_prev + d_k
            Sx_rows.append(Sx_curr.copy())
            Su_rows.append(Su_curr.copy())
            c_rows.append(c_curr.copy())
            Sx_prev, Su_prev, c_prev = Sx_curr, Su_curr, c_curr

        Sx = np.vstack(Sx_rows)
        Su = np.vstack(Su_rows)
        c = np.hstack(c_rows)[:, None]
        return Sx, Su, c

    def _compute_propagation_offset(self, kapp_horizon: np.ndarray) -> float:
        if not (self.use_propagation and self.have_scan and self.scan_ang is not None and self.scan_r is not None):
            self._prop_offset *= 0.8
            return self._prop_offset

        ang = self.scan_ang
        rr = self.scan_r
        if rr.size == 0:
            self._prop_offset *= 0.8
            return self._prop_offset

        use_strat = bool(self.get_parameter("use_strategic_cornering").value)
        kappa_thresh = float(self.get_parameter("corner_kappa_threshold").value)
        corner_off = float(self.get_parameter("corner_offset_m").value)

        desired = 0.0
        is_in_corner = False

        if use_strat and kapp_horizon.size > 5:
            avg_kappa = np.mean(kapp_horizon[0:5])
            if abs(avg_kappa) > kappa_thresh:
                is_in_corner = True
                desired = corner_off if avg_kappa > 0 else -corner_off

        if not is_in_corner:
            trigger = (self.front_clear < self.prop_front_trigger_m)
            if trigger:
                rel = wrap_angle(ang - self.lidar_front_center)
                a_min = math.radians(self.prop_side_min_deg)
                a_max = math.radians(self.prop_side_max_deg)
                maskL = (rel >= a_min) & (rel <= a_max)
                maskR = (rel <= -a_min) & (rel >= -a_max)

                def pct(x, p=40):
                    return float(np.percentile(x, p)) if x.size else float("nan")

                dL = pct(rr[maskL])
                dR = pct(rr[maskR])

                if not math.isnan(dL) and not math.isnan(dR):
                    desired = self.prop_gain * (dL - dR)

        desired = float(np.clip(desired, -self.prop_offset_max_m, self.prop_offset_max_m))

        alpha = self.prop_alpha
        self._prop_offset = (1.0 - alpha) * self._prop_offset + alpha * desired
        return self._prop_offset

    def _solve_mpc(self, x0, A_list, B_list, d_list, vref, kapp):
        n, m, N = 3, 2, self.N
        Q_perp = float(self.get_parameter("Q_perp").value)
        Q_psi = float(self.get_parameter("Q_psi").value)
        Q_v = float(self.get_parameter("Q_v").value)
        R_a = float(self.get_parameter("R_a").value)
        R_w = float(self.get_parameter("R_w").value)
        Rd_a = float(self.get_parameter("Rd_a").value)
        Rd_w = float(self.get_parameter("Rd_w").value)
        Qf_gain = float(self.get_parameter("Qf_gain").value)

        a_min = float(self.get_parameter("a_min").value)
        a_max = float(self.get_parameter("a_max").value)
        w_min = float(self.get_parameter("w_min").value)
        w_max = float(self.get_parameter("w_max").value)
        da_min = float(self.get_parameter("da_min").value)
        da_max = float(self.get_parameter("da_max").value)
        dw_min = float(self.get_parameter("dw_min").value)
        dw_max = float(self.get_parameter("dw_max").value)

        use_ff = bool(self.get_parameter("use_feedforward").value)
        kappa_sign = float(self.get_parameter("kappa_sign").value)
        ff_clip_ratio = float(self.get_parameter("ff_clip_ratio").value)

        Sx, Su, c = self._condense(A_list, B_list, d_list)

        blocks = []
        for k in range(N):
            diag_vals = [Q_perp, Q_psi, Q_v]
            if k == N - 1:
                diag_vals = [d * Qf_gain for d in diag_vals]
            blocks.append(sparse.diags(diag_vals, format="csc"))
        Q_bar = sparse.block_diag(blocks, format="csc")

        R_block = sparse.diags([R_a, R_w], format="csc")
        Rk = sparse.kron(sparse.eye(N, format="csc"), R_block, format="csc")
        Rd_block = sparse.diags([Rd_a, Rd_w], format="csc")
        Rd_bar = sparse.kron(sparse.eye(N, format="csc"), Rd_block, format="csc")

        D = sparse.lil_matrix((N * m, N * m))
        I2 = sparse.eye(m, format="csc")
        D[0:m, 0:m] = I2
        for k in range(1, N):
            D[k * m:(k + 1) * m, (k - 1) * m:k * m] = -I2
            D[k * m:(k + 1) * m, k * m:(k + 1) * m] = I2
        D = D.tocsc()

        if use_ff:
            dv = np.diff(vref, prepend=vref[0])
            a_ff = dv / max(self.dt, 1e-3)
            a_ff = np.clip(a_ff, -ff_clip_ratio * a_max, ff_clip_ratio * a_max)
            w_ff = vref * (kappa_sign * kapp)
            w_ff = np.clip(w_ff, -ff_clip_ratio * w_max, ff_clip_ratio * w_max)
            U_bias = np.vstack([a_ff, w_ff]).T.reshape(N * m, 1)
        else:
            U_bias = np.zeros((N * m, 1))

        Suc = sparse.csc_matrix(Su)
        P0 = (Suc.T @ (Q_bar @ Suc)) + Rk + (D.T @ Rd_bar @ D)
        P = 0.5 * (P0 + P0.T)
        P = P.tocsc()

        rhs = (Sx @ x0[:, None] + c)
        q_vec = np.asarray(Suc.T @ (Q_bar @ (rhs + Suc @ U_bias))).ravel()
        q_vec += np.asarray(Rk @ U_bias).ravel()
        q_vec += np.asarray(D.T @ (Rd_bar @ (D @ U_bias))).ravel()

        self.u_prev[0, 0] = np.clip(self.u_prev[0, 0], a_min, a_max)
        self.u_prev[1, 0] = np.clip(self.u_prev[1, 0], w_min, w_max)

        Umin_tile = np.tile([a_min, w_min], N)
        Umax_tile = np.tile([a_max, w_max], N)
        l_box = Umin_tile - U_bias.ravel()
        u_box = Umax_tile - U_bias.ravel()

        du_min_tile = np.tile([da_min, dw_min], N)
        du_max_tile = np.tile([da_max, dw_max], N)
        du_min_tile[0:2] += self.u_prev.ravel()
        du_max_tile[0:2] += self.u_prev.ravel()
        Dbias = (D @ U_bias).ravel()
        l_du = du_min_tile - Dbias
        u_du = du_max_tile - Dbias

        A_total = sparse.vstack([sparse.eye(N * m, format="csc"), D], format="csc")
        l_total = np.hstack([l_box, l_du])
        u_total = np.hstack([u_box, u_du])

        eps = self.eps_bounds
        l_total -= eps
        u_total += eps

        solver = osqp.OSQP()
        solver.setup(P=P, q=q_vec, A=A_total, l=l_total, u=u_total,
                     polish=True, eps_rel=1e-3, eps_abs=1e-3,
                     max_iter=8000, verbose=self.osqp_debug)

        if self._U_last is not None:
            try:
                solver.warm_start(x=(self._U_last - U_bias).ravel())
            except Exception:
                pass

        res = solver.solve()
        status_ok = res.info.status_val in (1, 2)

        if not status_ok:
            return None, res.info.status_val

        Utilde_opt = res.x.reshape(-1, 1)
        Uopt = Utilde_opt + U_bias
        self._U_last = Uopt.copy()
        u0 = Uopt[0:2, :].copy()
        return u0, res.info.status_val

    def _publish_local_ref(self, xref, yref, psiref, publisher=None):
        pub = self.pub_ref if publisher is None else publisher
        path = PathMsg()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y, psi in zip(xref, yref, psiref):
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.z = math.sin(psi * 0.5)
            ps.pose.orientation.w = math.cos(psi * 0.5)
            path.poses.append(ps)
        pub.publish(path)

    def _publish_predicted(self, xref, yref, psiref, Xpred):
        path = PathMsg()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        if self.get_parameter("pred_start_at_car").value:
            ps0 = PoseStamped()
            ps0.header.frame_id = "map"
            ps0.pose.position.x = self.x
            ps0.pose.position.y = self.y
            ps0.pose.orientation.z = math.sin(self.yaw * 0.5)
            ps0.pose.orientation.w = math.cos(self.yaw * 0.5)
            path.poses.append(ps0)

        for k in range(min(len(xref), Xpred.shape[0])):
            e_perp = Xpred[k, 0]
            psi = psiref[k]
            nx = -math.sin(psi)
            ny = math.cos(psi)
            px = xref[k] + nx * e_perp
            py = yref[k] + ny * e_perp
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.pose.position.x = px
            ps.pose.position.y = py
            ps.pose.orientation.z = math.sin(psi * 0.5)
            ps.pose.orientation.w = math.cos(psi * 0.5)
            path.poses.append(ps)
        self.pub_pred.publish(path)

    def _need_emergency_stop(self) -> bool:
        if not (self.use_lidar_gate and np.isfinite(self.front_clear)):
            return False
        d_stop = (self.v_meas ** 2) / (2.0 * self.a_brake) + self.stop_margin
        return self.front_clear <= d_stop

    def _publish_brake_hold(self):
        cmd = AckermannDriveStamped()
        v_next = max(0.0, self.v_meas - self.a_brake * self.dt)
        cmd.drive.speed = v_next
        cmd.drive.steering_angle = self.delta_prev
        self.pub_cmd.publish(cmd)

    def control_step(self):
        if not self.have_state:
            return

        if self._need_emergency_stop():
            self._publish_brake_hold()
            return

        idx0 = self._nearest_index(self.x, self.y)
        xref, yref, psiref, _, kapp = self._build_horizon(idx0)

        self._publish_local_ref(xref, yref, psiref, publisher=self.pub_ref_raw)

        d_off = self._compute_propagation_offset(kapp) if self.use_propagation else 0.0
        self.pub_prop_offset.publish(Float32(data=float(d_off)))
        if abs(d_off) > 1e-6:
            decay = self.prop_decay
            decay_factors = decay ** np.arange(len(xref))
            nx = -np.sin(psiref)
            ny = np.cos(psiref)
            xref += nx * d_off * decay_factors
            yref += ny * d_off * decay_factors

        abs_k = np.abs(kapp)
        a_lat_max = float(self.get_parameter("a_lat_max").value)
        v_curve = np.sqrt(np.maximum(a_lat_max / np.maximum(abs_k, 1e-6), 0.0))

        v_top = float(self.get_parameter("v_top").value)
        if bool(self.get_parameter("use_straight_boost").value) and self._is_straight_ahead(idx0):
            top = float(self.get_parameter("v_top_straight").value)
        else:
            top = v_top
        v_cap = np.minimum(v_curve, top)

        if self.use_lidar_gate and np.isfinite(self.front_clear):
            d_free = max(0.0, self.front_clear - self.stop_margin)
            v_lidar_max = math.sqrt(max(0.0, 2.0 * self.a_brake * d_free))
            v_cap = np.minimum(v_cap, v_lidar_max)

        ds = float(self.get_parameter("ds_ref").value)
        a_x_max = float(self.get_parameter("a_x_max").value)
        a_x_min = float(self.get_parameter("a_x_min").value)
        v_ref = v_cap.copy()
        v_ref = np.maximum(v_ref, 0.2)

        for i in range(len(v_ref) - 1):
            v_ref[i + 1] = min(v_ref[i + 1], math.sqrt(max(v_ref[i] ** 2 + 2.0 * a_x_max * ds, 0.0)))
        axb = abs(a_x_min)
        for i in range(len(v_ref) - 1, 0, -1):
            v_ref[i - 1] = min(v_ref[i - 1], math.sqrt(max(v_ref[i] ** 2 + 2.0 * axb * ds, 0.0)))

        x0 = self._error_now(xref[0], yref[0], psiref[0], v_ref[0])

        A_list, B_list, d_list = [], [], []
        dt = self.dt
        for k in range(self.N):
            v = v_ref[k]
            kap = kapp[k]
            A = np.array([[1, dt * v, 0], [0, 1, 0], [0, 0, 1]])
            B = np.array([[0, 0], [0, dt], [dt, 0]])
            d = np.array([0, -dt * kap * v, 0])
            A_list.append(A)
            B_list.append(B)
            d_list.append(d)

        u0, status = self._solve_mpc(x0, A_list, B_list, d_list, v_ref, kapp)
        if u0 is None:
            cmd = AckermannDriveStamped()
            cmd.drive.speed = max(0.0, self.v_meas - 0.5)
            cmd.drive.steering_angle = self.delta_prev
            self.pub_cmd.publish(cmd)
            self.get_logger().warn("[MPC] QP no resoluble, fallback brake/straight.")
            return

        a_cmd, w_cmd = u0[0, 0], u0[1, 0]
        v_cmd = self.v_meas + a_cmd * self.dt
        delta = math.atan2(self.L * w_cmd, max(v_cmd, 1e-2))

        alpha = float(self.get_parameter("steer_alpha").value)
        if 0.0 < alpha < 1.0:
            delta = (1.0 - alpha) * delta + alpha * self.delta_prev
        self.delta_prev = delta

        cmd = AckermannDriveStamped()
        cmd.drive.speed = v_cmd
        cmd.drive.steering_angle = delta
        self.pub_cmd.publish(cmd)
        self.u_prev = np.array([[a_cmd], [w_cmd]], dtype=float)

        self._publish_local_ref(xref, yref, psiref, publisher=self.pub_ref)
        if u0 is not None:
            # Re-uso de la solucion. En este caso no hacemos nada con ella.
            # En un futuro, U_stack = res.x.reshape(self.N, 2)
            U_stack = np.tile(u0.T, (self.N, 1)).flatten().reshape(-1, 1)
            Sx, Su, c = self._condense(A_list, B_list, d_list)
            X_stack = (Sx @ x0[:, None] + Su @ U_stack + c)
            Xpred = np.asarray(X_stack).reshape(self.N, 3)
            self._publish_predicted(xref, yref, psiref, Xpred)


def main(args=None):
    rclpy.init(args=args)
    node = MPCFollowerOSQP()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

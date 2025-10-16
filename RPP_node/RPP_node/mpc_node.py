#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ==============================================================================
# ==        MPC Follower para F1TENTH con Integración AMCL (Versión Final)      ==
# ==============================================================================
# Este controlador utiliza un Modelo Predictivo (MPC) para guiar un coche F1TENTH
# a lo largo de una trayectoria. Su característica principal es un sistema de
# localización robusto que prioriza AMCL y utiliza un respaldo de Estimación
# por Cuentas (Dead Reckoning) si AMCL falla, evitando el error acumulado de la
# odometría.

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

# Se importa PoseWithCovarianceStamped para recibir los mensajes de AMCL
from nav_msgs.msg import Odometry, Path as PathMsg
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32

import osqp
from scipy import sparse

# --- Funciones de Utilidad ---

def wrap_angle(a: float) -> float:
    """Asegura que un ángulo esté en el rango [-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi

def yaw_from_quat(q) -> float:
    """Extrae el ángulo Yaw (rotación en Z) de un cuaternión."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def _latched_qos() -> QoSProfile:
    """Define un perfil de QoS 'latched' para topics que solo se publican una vez."""
    qos = QoSProfile(depth=1)
    qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    qos.reliability = QoSReliabilityPolicy.RELIABLE
    qos.history = QoSHistoryPolicy.KEEP_LAST
    return qos

# --- Clase Principal del Controlador ---

class MPCFollowerOSQP(Node):
    def __init__(self) -> None:
        super().__init__("MPC_AMCL_Follower")

        # --- Declaración de Parámetros desde el archivo YAML ---
        self._declare_all_parameters()
        
        # --- Lectura de Parámetros ---
        self._load_all_parameters()
        
        # --- Estado del Vehículo ---
        self._initialize_state_variables()
        
        # --- Carga de Waypoints ---
        self._load_and_process_waypoints()
        
        # --- Suscripciones y Publicaciones ---
        self._setup_ros_communications()

        # --- Bucle de Control Principal ---
        self.timer = self.create_timer(self.dt, self.control_step)
        
        loc_mode = "AMCL + Dead Reckoning" if self.use_amcl else "Odom_Only"
        self.get_logger().info(f"✅ MPC Controller (AMCL Integrated) listo. Modo: {loc_mode}")
        self.get_logger().info(f"   Leyendo waypoints de: {self.csv_path}")

    # --- Métodos de Inicialización ---

    def _declare_all_parameters(self):
        """Declara todos los parámetros de ROS para que puedan ser configurados desde un archivo YAML."""
        # Localización
        self.declare_parameter("use_amcl", True)
        self.declare_parameter("topic_amcl", "/amcl_pose")
        self.declare_parameter("topic_odom", "/odom")
        self.declare_parameter("amcl_timeout", 2.0)
        # Archivos y Topics
        self.declare_parameter("waypoints_csv", "waypoints.csv")
        self.declare_parameter("topic_cmd", "/autonomous")
        self.declare_parameter("lidar_topic", "/scan")
        # Modelo y Horizonte
        self.declare_parameter("L", 0.33); self.declare_parameter("dt", 0.02)
        self.declare_parameter("ds_ref", 0.12); self.declare_parameter("N", 10)
        self.declare_parameter("steer_alpha", 0.22)
        # Perfil de Velocidad
        self.declare_parameter("v_top", 4.5); self.declare_parameter("a_lat_max", 4.0)
        self.declare_parameter("a_x_max", 4.0); self.declare_parameter("a_x_min", -5.5)
        # Límites Físicos
        self.declare_parameter("a_min", -5.0); self.declare_parameter("a_max", 5.0)
        self.declare_parameter("w_min", -3.2); self.declare_parameter("w_max", 3.2)
        self.declare_parameter("da_min", -1.0); self.declare_parameter("da_max", 1.0)
        self.declare_parameter("dw_min", -1.00); self.declare_parameter("dw_max", 1.00)
        # Costos del Optimizador (Tuning)
        self.declare_parameter("Q_perp", 12.0); self.declare_parameter("Q_psi", 7.0)
        self.declare_parameter("Q_v", 2.5); self.declare_parameter("Qf_gain", 5.0)
        self.declare_parameter("R_a", 0.20); self.declare_parameter("R_w", 0.25)
        self.declare_parameter("Rd_a", 0.10); self.declare_parameter("Rd_w", 0.40)
        # LiDAR y Seguridad
        self.declare_parameter("use_lidar_gate", True); self.declare_parameter("lidar_window_deg", 20.0)
        self.declare_parameter("a_brake", 12.0); self.declare_parameter("stop_margin", 0.5)
        # Cronómetro
        self.declare_parameter("lap_origin_x", 0.0); self.declare_parameter("lap_origin_y", 0.0)
        self.declare_parameter("lap_radius", 0.5); self.declare_parameter("lap_partial_period_s", 2.0)

    def _load_all_parameters(self):
        """Lee los valores de los parámetros declarados."""
        g = self.get_parameter
        self.use_amcl = g("use_amcl").value; self.topic_amcl = g("topic_amcl").value
        self.topic_odom = g("topic_odom").value; self.amcl_timeout = g("amcl_timeout").value
        self.csv_path = FsPath(g("waypoints_csv").value); self.topic_cmd = g("topic_cmd").value
        self.lidar_topic = g("lidar_topic").value; self.L = g("L").value
        self.dt = g("dt").value; self.N = g("N").value

    def _initialize_state_variables(self):
        """Inicializa todas las variables de estado del controlador."""
        # Fuentes de datos
        self.have_odom, self.last_odom_t = False, 0.0
        self.x_odom, self.y_odom, self.yaw_odom, self.v_meas = 0.0, 0.0, 0.0, 0.0
        self.have_amcl, self.last_amcl_t = False, 0.0
        self.x_amcl, self.y_amcl, self.yaw_amcl = 0.0, 0.0, 0.0
        # Estado fusionado
        self.pose_ok, self.x, self.y, self.yaw = False, 0.0, 0.0, 0.0
        # Estado de Dead Reckoning
        self.is_dead_reckoning = False
        self.x_dr, self.y_dr, self.yaw_dr = 0.0, 0.0, 0.0
        # Estado del control
        self.u_prev, self.delta_prev = np.zeros((2, 1)), 0.0
        self._U_last: Optional[np.ndarray] = None
        # Estado de LiDAR y cronómetro
        self.have_scan, self.front_clear = False, float("inf")
        self.lap_count, self._lap_gate_inside = -1, False
        self.lap_start_time, self.last_partial_t = time.time(), time.time()
        self.best_lap_s = None

    def _load_and_process_waypoints(self):
        """Carga, remuestrea y suaviza la trayectoria desde el archivo CSV."""
        try:
            xy = self._load_csv_xy(self.csv_path)
            self.s_grid, self.cx, self.cy, self.cpsi, self.ckappa = self._resample_xy(
                xy, self.get_parameter("ds_ref").value
            )
        except (FileNotFoundError, RuntimeError) as e:
            self.get_logger().fatal(f"Error crítico al cargar waypoints: {e}")
            self.s_grid, self.cx, self.cy, self.cpsi, self.ckappa = [],[],[],[],[]

    def _setup_ros_communications(self):
        """Configura todos los suscriptores y publicadores de ROS."""
        self.sub_odom = self.create_subscription(Odometry, self.topic_odom, self._odom_cb, 20)
        if self.use_amcl:
            self.sub_amcl = self.create_subscription(PoseWithCovarianceStamped, self.topic_amcl, self._amcl_cb, 20)
        if self.get_parameter("use_lidar_gate").value:
            self.sub_scan = self.create_subscription(LaserScan, self.lidar_topic, self._scan_cb, 30)
        
        self.pub_cmd = self.create_publisher(AckermannDriveStamped, self.topic_cmd, 10)
        self.pub_ref = self.create_publisher(PathMsg, "/mpc/local_ref", 1)

    # --- Callbacks para la Recepción de Datos ---

    def _amcl_cb(self, msg: PoseWithCovarianceStamped) -> None:
        """Callback para AMCL. Guarda la pose y resetea la estimación de Dead Reckoning."""
        self.x_amcl, self.y_amcl = msg.pose.pose.position.x, msg.pose.pose.position.y
        self.yaw_amcl = yaw_from_quat(msg.pose.pose.orientation)
        self.have_amcl, self.last_amcl_t = True, self.get_clock().now().nanoseconds * 1e-9

        if self.is_dead_reckoning:
            self.get_logger().info("✅ AMCL recuperado. Saliendo de modo Dead Reckoning.")
            self.is_dead_reckoning = False
        
        self.x_dr, self.y_dr, self.yaw_dr = self.x_amcl, self.y_amcl, self.yaw_amcl

    def _odom_cb(self, msg: Odometry) -> None:
        """Callback para la odometría. Guarda la pose (para DR) y la velocidad (para control)."""
        self.x_odom, self.y_odom = msg.pose.pose.position.x, msg.pose.pose.position.y
        self.yaw_odom = yaw_from_quat(msg.pose.pose.orientation)
        self.v_meas = float(msg.twist.twist.linear.x)
        self.have_odom, self.last_odom_t = True, self.get_clock().now().nanoseconds * 1e-9
        self._lap_logic()
        self._lap_partial()

    def _scan_cb(self, msg: LaserScan) -> None:
        """Callback para el LiDAR. Calcula la distancia libre al frente para el frenado de emergencia."""
        if len(msg.ranges) == 0: self.front_clear = float("inf"); return
        rr = np.array(msg.ranges)
        rr[np.isinf(rr) | np.isnan(rr) | (rr < msg.range_min)] = msg.range_max
        win_rad = math.radians(self.get_parameter("lidar_window_deg").value)
        
        center_angle = 0.0 # Asumimos que el centro del LIDAR apunta al frente
        angle_min_in_scan = msg.angle_min
        
        start_idx = int((center_angle - win_rad / 2.0 - angle_min_in_scan) / msg.angle_increment)
        end_idx = int((center_angle + win_rad / 2.0 - angle_min_in_scan) / msg.angle_increment)
        start_idx = max(0, start_idx)
        end_idx = min(len(rr) - 1, end_idx)

        front_ranges = rr[start_idx : end_idx]
        if len(front_ranges) > 0:
            self.front_clear = float(np.percentile(front_ranges, 10.0))
        else:
            self.front_clear = msg.range_max
        self.have_scan = True

    # --- Lógica Principal ---

    def _update_pose(self) -> bool:
        """Lógica de fusión: Prioriza AMCL, usa Dead Reckoning como respaldo."""
        now = self.get_clock().now().nanoseconds * 1e-9
        amcl_is_fresh = self.use_amcl and self.have_amcl and (now - self.last_amcl_t <= self.amcl_timeout)
        
        if amcl_is_fresh:
            # Plan A: AMCL está funcionando
            self.x, self.y, self.yaw = self.x_amcl, self.y_amcl, self.yaw_amcl
            return True
        elif self.have_odom:
            # Plan B: AMCL ha fallado, intentamos Dead Reckoning
            if not self.is_dead_reckoning:
                self.get_logger().warn("⚠️ AMCL perdido. Entrando en modo Dead Reckoning...")
                self.is_dead_reckoning = True

            # Modelo cinemático para estimar la nueva pose
            v = self.v_meas; delta = self.delta_prev
            yaw_rate = (v / self.L) * math.tan(delta)
            
            self.yaw_dr = wrap_angle(self.yaw_dr + yaw_rate * self.dt)
            self.x_dr += v * math.cos(self.yaw_dr) * self.dt
            self.y_dr += v * math.sin(self.yaw_dr) * self.dt

            self.x, self.y, self.yaw = self.x_dr, self.y_dr, self.yaw_dr
            return True
        return False # Plan C: Falla total

    def control_step(self):
        """El bucle de control principal."""
        if len(self.cx) == 0: self.get_logger().error("No hay waypoints cargados. El controlador no se ejecutará.", throttle_duration_sec=5); return
        
        self.pose_ok = self._update_pose()
        if not self.pose_ok: self._publish_brake_hold(); return
        if self._need_emergency_stop(): self._publish_brake_hold(); return
        
        idx0 = self._nearest_index(self.x, self.y)
        xref, yref, psiref, _, kapp = self._build_horizon(idx0)
        v_ref = self._calculate_velocity_profile(kapp)
        x0 = self._error_now(xref[0], yref[0], psiref[0], v_ref[0])
        A_list, B_list, d_list = self._linearize_vehicle_model(v_ref, kapp)
        u0, status = self._solve_mpc(x0, A_list, B_list, d_list, v_ref, kapp)
        
        if u0 is None:
            self.get_logger().warn(f"[MPC] Solver falló (status={status}), frenando.", throttle_duration_sec=1)
            self._publish_brake_hold(); return
        
        a_cmd, w_cmd = u0[0, 0], u0[1, 0]
        v_cmd = self.v_meas + a_cmd * self.dt
        delta = math.atan2(self.L * w_cmd, max(v_cmd, 1e-2))
        alpha = self.get_parameter("steer_alpha").value
        delta = (1.0 - alpha) * delta + alpha * self.delta_prev
        self.delta_prev = delta # Crítico para Dead Reckoning
        
        cmd = AckermannDriveStamped()
        cmd.drive.speed, cmd.drive.steering_angle = v_cmd, delta
        self.pub_cmd.publish(cmd)
        self.u_prev = np.array([[a_cmd], [w_cmd]], dtype=float)
        self._publish_local_ref(xref, yref, psiref)

    # --- Funciones Auxiliares (completas) ---
    def _load_csv_xy(self, p: FsPath) -> np.ndarray:
        if not p.exists(): raise FileNotFoundError(f"{p} not found")
        pts = [];
        with open(p, "r") as f:
            for row in csv.reader(f):
                if len(row) >= 2:
                    try: pts.append((float(row[0]), float(row[1])))
                    except ValueError: continue
        if len(pts) < 5: raise RuntimeError(f"Not enough valid waypoints in {p}")
        return np.array(pts)

    def _resample_xy(self, xy: np.ndarray, ds: float):
        dxy = np.diff(xy, axis=0); seg = np.hypot(dxy[:,0], dxy[:,1])
        s = np.concatenate(([0], np.cumsum(seg)))
        if self.loop_track:
            xy = np.vstack([xy, xy[0,:]]); s = np.append(s, s[-1] + np.hypot(xy[-1,0]-xy[-2,0], xy[-1,1]-xy[-2,1]))
        s_new = np.arange(0, s[-1], ds)
        x = np.interp(s_new, s, xy[:,0]); y = np.interp(s_new, s, xy[:,1])
        dx, dy = np.gradient(x, s_new), np.gradient(y, s_new)
        psi = np.unwrap(np.arctan2(dy, dx))
        ddx, ddy = np.gradient(dx, s_new), np.gradient(dy, s_new)
        kappa = (dx * ddy - dy * ddx) / np.maximum((dx**2 + dy**2)**1.5, 1e-6)
        return s_new, x, y, psi, kappa

    def _lap_logic(self):
        o = (self.get_parameter("lap_origin_x").value, self.get_parameter("lap_origin_y").value)
        r = self.get_parameter("lap_radius").value
        inside = np.hypot(self.x-o[0], self.y-o[1]) < r
        if inside and not self._lap_gate_inside:
            now = time.time(); lap_time = now - self.lap_start_time; self.lap_count += 1
            if self.lap_count > 0:
                m, s = divmod(lap_time, 60); txt=f"🏁 Lap {self.lap_count}: {int(m):02d}:{s:05.2f}"
                if self.best_lap_s is None or lap_time < self.best_lap_s: self.best_lap_s = lap_time; txt += " (New Best!)"
                self.get_logger().info(txt)
            self.lap_start_time = now
        self._lap_gate_inside = inside

    def _lap_partial(self):
        p = self.get_parameter("lap_partial_period_s").value
        now = time.time()
        if self.lap_count >= 0 and now - self.last_partial_t > p:
            t = now - self.lap_start_time; m, s = divmod(t, 60)
            self.get_logger().info(f"  ... Lap {self.lap_count} partial: {int(m):02d}:{s:05.2f} | v={self.v_meas:.2f} m/s", throttle_duration_sec=p)
            self.last_partial_t = now

    def _nearest_index(self, x, y): return np.argmin((self.cx-x)**2 + (self.cy-y)**2)
    
    def _build_horizon(self, idx0):
        N, dt, ds = self.N, self.dt, self.get_parameter("ds_ref").value
        xref, yref, psiref, kapp = [],[],[],[]
        i, M = idx0, len(self.cx)
        for _ in range(N):
            xref.append(self.cx[i]); yref.append(self.cy[i]); psiref.append(self.cpsi[i]); kapp.append(self.ckappa[i])
            step = max(1, int(round(max(0.1, self.v_meas)*dt/ds)))
            i = (i+step)%M if self.loop_track else min(i+step, M-1)
        return np.array(xref), np.array(yref), np.array(psiref), None, np.array(kapp)

    def _calculate_velocity_profile(self, kapp):
        v_curve = np.sqrt(self.get_parameter("a_lat_max").value / np.maximum(np.abs(kapp), 1e-6))
        v_cap = np.minimum(v_curve, self.get_parameter("v_top").value)
        if self.get_parameter("use_lidar_gate").value and np.isfinite(self.front_clear):
            d_free = max(0, self.front_clear - self.get_parameter("stop_margin").value)
            v_lidar = np.sqrt(2 * self.get_parameter("a_brake").value * d_free)
            v_cap = np.minimum(v_cap, v_lidar)
        ds, ax_max, ax_min = self.get_parameter("ds_ref").value, self.get_parameter("a_x_max").value, self.get_parameter("a_x_min").value
        v_ref = np.copy(v_cap); v_ref = np.maximum(v_ref, 0.1) # Minimum speed
        for i in range(len(v_ref)-1): v_ref[i+1] = min(v_ref[i+1], np.sqrt(v_ref[i]**2 + 2*ax_max*ds))
        for i in range(len(v_ref)-1, 0, -1): v_ref[i-1] = min(v_ref[i-1], np.sqrt(v_ref[i]**2 + 2*abs(ax_min)*ds))
        return v_ref

    def _error_now(self, xref, yref, psiref, vref):
        e_perp = -np.sin(psiref)*(self.x-xref) + np.cos(psiref)*(self.y-yref)
        e_psi = wrap_angle(self.yaw - psiref)
        e_v = self.v_meas - vref
        return np.array([e_perp, e_psi, e_v])

    def _linearize_vehicle_model(self, v_ref, kapp):
        A,B,d = [],[],[]
        for k in range(self.N):
            v, kap, dt = v_ref[k], kapp[k], self.dt
            A.append(np.array([[1, dt*v, 0], [0, 1, 0], [0, 0, 1]]))
            B.append(np.array([[0,0],[0,dt],[dt,0]]))
            d.append(np.array([0, -dt*kap*v, 0]))
        return A,B,d
        
    def _solve_mpc(self, x0, A_list, B_list, d_list, vref, kapp):
        n, m, N = 3, 2, self.N
        g=self.get_parameter; Q_p,Q_psi,Q_v=g("Q_perp").value,g("Q_psi").value,g("Q_v").value
        R_a,R_w=g("R_a").value,g("R_w").value; Rd_a,Rd_w=g("Rd_a").value,g("Rd_w").value
        Qf_g=g("Qf_gain").value; Sx=np.zeros((N*n,n)); Su=np.zeros((N*n,N*m)); c=np.zeros((N*n,1))
        Sx_k,Su_k,c_k=A_list[0],np.zeros((n,N*m)),d_list[0].reshape(n,1); Su_k[:,0:m]=B_list[0]
        Sx[0:n,:],Su[0:n,:],c[0:n,:]=Sx_k,Su_k,c_k
        for k in range(1,N):
            Sx_k=A_list[k]@Sx_k; Su_k=A_list[k]@Su_k; Su_k[:,k*m:(k+1)*m]+=B_list[k]
            c_k=A_list[k]@c_k+d_list[k].reshape(n,1)
            Sx[k*n:(k+1)*n,:],Su[k*n:(k+1)*n,:],c[k*n:(k+1)*n,:]=Sx_k,Su_k,c_k
        Q_bar=sparse.block_diag([sparse.diags([Q_p,Q_psi,Q_v])]*(N-1)+[sparse.diags([Q_p*Qf_g,Q_psi*Qf_g,Q_v*Qf_g])])
        R_bar=sparse.kron(sparse.eye(N),sparse.diags([R_a,R_w]))
        Rd_bar=sparse.kron(sparse.eye(N),sparse.diags([Rd_a,Rd_w]))
        D_mat=sparse.eye(N*m)-sparse.diags([np.ones((N-1)*m)],k=-m)
        P=Su.T@Q_bar@Su+R_bar+D_mat.T@Rd_bar@D_mat
        q=Su.T@Q_bar@(Sx@x0+c.ravel())
        a_min,a_max=g("a_min").value,g("a_max").value; w_min,w_max=g("w_min").value,g("w_max").value
        da_min,da_max=g("da_min").value,g("da_max").value; dw_min,dw_max=g("dw_min").value,g("dw_max").value
        l_u=np.tile([a_min,w_min],N); u_u=np.tile([a_max,w_max],N)
        l_du=np.tile([da_min,dw_min],N); u_du=np.tile([da_max,dw_max],N)
        l_du[0:m]-=self.u_prev.ravel(); u_du[0:m]-=self.u_prev.ravel()
        A_con=sparse.vstack([sparse.eye(N*m),D_mat]); l_con,u_con=np.hstack([l_u,l_du]),np.hstack([u_u,u_du])
        solver=osqp.OSQP(); solver.setup(P=sparse.csc_matrix(P),q=q,A=sparse.csc_matrix(A_con),l=l_con,u=u_con,verbose=False,polish=True,eps_abs=1e-4,eps_rel=1e-4)
        if self._U_last is not None: solver.warm_start(x=self._U_last)
        res=solver.solve()
        if res.info.status_val not in [1,2]: return None,res.info.status
        self._U_last=res.x; return res.x[0:m].reshape(m,1),res.info.status

    def _need_emergency_stop(self):
        if not self.get_parameter("use_lidar_gate").value or not np.isfinite(self.front_clear): return False
        d_stop = self.v_meas**2 / (2*self.get_parameter("a_brake").value) + self.get_parameter("stop_margin").value
        return self.front_clear <= d_stop

    def _publish_brake_hold(self):
        cmd = AckermannDriveStamped()
        cmd.drive.speed = 0.0
        cmd.drive.steering_angle = self.delta_prev
        self.pub_cmd.publish(cmd)

    def _publish_local_ref(self, xref, yref, psiref):
        path = PathMsg(); path.header.frame_id = "map"; path.header.stamp = self.get_clock().now().to_msg()
        for x,y,psi in zip(xref,yref,psiref):
            ps = PoseStamped(); ps.pose.position.x,ps.pose.position.y=float(x),float(y)
            ps.pose.orientation.z,ps.pose.orientation.w=math.sin(psi/2),math.cos(psi/2)
            path.poses.append(ps)
        self.pub_ref.publish(path)

# --- Punto de Entrada Principal ---
def main(args=None):
    rclpy.init(args=args)
    node = MPCFollowerOSQP()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__": main()

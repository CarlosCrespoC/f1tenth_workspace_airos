#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# FRAGMENTO MEJORADO - Reemplazar funciones en tu código original

import math
from typing import Tuple

def clamp(x, a, b): 
    return max(a, min(b, x))

class WaypointFollower:
    # ... (mantener todo tu código existente, solo reemplazar estas funciones)
    
    def __init__(self):
        # ... (tu código existente)
        
        # NUEVOS PARÁMETROS RECOMENDADOS
        self.declare_parameter("lookahead_min", 0.8)   # Lookahead mínimo (curvas cerradas)
        self.declare_parameter("lookahead_max", 2.5)   # Lookahead máximo (rectas)
        self.declare_parameter("lookahead_gain", 0.4)  # Ganancia: Ld = Ld_min + gain * v
        
        # ... (leer parámetros)
        self.lookahead_min = g("lookahead_min").get_parameter_value().double_value
        self.lookahead_max = g("lookahead_max").get_parameter_value().double_value
        self.lookahead_gain = g("lookahead_gain").get_parameter_value().double_value
    
    def _calculate_dynamic_lookahead(self, current_speed: float) -> float:
        """
        Calcula lookahead dinámico basado en velocidad actual.
        A mayor velocidad, mayor lookahead para ver más adelante.
        """
        # Lookahead proporcional a la velocidad
        Ld_dynamic = self.lookahead_min + self.lookahead_gain * current_speed
        
        # Limitar entre mínimo y máximo
        return clamp(Ld_dynamic, self.lookahead_min, self.lookahead_max)
    
    def _calculate_path_curvature(self, idx: int) -> float:
        """
        MEJORADO: Calcula curvatura con mejor robustez.
        """
        if len(self.waypoints) < 3:
            return 0.0
        
        n = len(self.waypoints)
        num_points = min(self.curve_detection_points, n)
        
        # Recopilar puntos adelante
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
        
        # Calcular curvatura como tasa de cambio de ángulo
        total_angle_change = 0.0
        total_distance = 0.0
        
        for k in range(len(points) - 2):
            p1, p2, p3 = points[k], points[k+1], points[k+2]
            
            # Vectores entre puntos
            v1x, v1y = p2[0] - p1[0], p2[1] - p1[1]
            v2x, v2y = p3[0] - p2[0], p3[1] - p2[1]
            
            # Distancia entre puntos
            d1 = math.hypot(v1x, v1y)
            d2 = math.hypot(v2x, v2y)
            
            if d1 < 1e-6 or d2 < 1e-6:
                continue
            
            # Ángulo entre vectores usando producto punto y cruz
            dot = v1x * v2x + v1y * v2y
            cross = v1x * v2y - v1y * v2x
            angle_change = abs(math.atan2(cross, dot))
            
            total_angle_change += angle_change
            total_distance += (d1 + d2) / 2.0
        
        # Curvatura = cambio total de ángulo / distancia total
        if total_distance < 1e-6:
            return 0.0
        
        curvature = total_angle_change / total_distance
        return curvature
    
    def _calculate_adaptive_speed(self, curvature: float) -> float:
        """
        MEJORADO: Calcula velocidad con mapeo más inteligente.
        Usa fórmula física: v = sqrt(a_lat_max / curvature)
        """
        if not self.adaptive_speed:
            return self.v_des
        
        # Si es recta, velocidad máxima
        if curvature < 0.02:  # Muy poca curvatura
            return self.v_des
        
        # Limitar curvatura para curvas muy cerradas
        # (más grande que esto es prácticamente imposible de seguir)
        curvature = min(curvature, 2.0)
        
        # Aceleración lateral máxima permitida (ajustar según tu robot)
        # F1TENTH típico: 8-12 m/s² (con buen agarre)
        a_lat_max = 8.0  # m/s²
        
        # Velocidad máxima físicamente posible en la curva
        v_phys_max = math.sqrt(a_lat_max / max(curvature, 0.01))
        
        # Aplicar factor de seguridad y ganancia de usuario
        safety_factor = 0.8  # 80% de la velocidad física máxima
        v_curve = v_phys_max * safety_factor / self.speed_reduction_gain
        
        # Limitar entre v_min y v_des
        target_speed = clamp(v_curve, self.v_min, self.v_des)
        
        return target_speed
    
    def _on_timer(self):
        """
        MEJORADO: Loop de control con lookahead dinámico.
        """
        now = self.get_clock().now().nanoseconds * 1e-9
        pose_ok = self._update_pose()
        
        # Calcular velocidad adaptativa basada en curvatura
        if pose_ok and self.autonomous_active and len(self.waypoints) >= 3:
            self.nearest_idx = self._nearest_wp_index(self.x, self.y)
            
            # NUEVO: Calcular curvatura
            curvature = self._calculate_path_curvature(self.nearest_idx)
            v_target = self._calculate_adaptive_speed(curvature)
            
            # NUEVO: Calcular lookahead dinámico basado en velocidad publicada
            Ld_dynamic = self._calculate_dynamic_lookahead(self.v_pub)
            
            # LOG OPCIONAL para debug (comentar después)
            if int(now * 10) % 5 == 0:  # Log cada 0.5s
                self.get_logger().info(
                    f"curv={curvature:.3f} | v_tgt={v_target:.2f} | Ld={Ld_dynamic:.2f}",
                    throttle_duration_sec=0.5
                )
        else:
            v_target = 0.0
            Ld_dynamic = self.Ld  # Usar lookahead base si no hay pose
        
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
        
        # USAR LOOKAHEAD DINÁMICO
        xt, yt, _ = self._target_by_lookahead(self.nearest_idx, Ld_dynamic)

        # --- FOLLOW THE GAP (tu código existente) ---
        used_gap = False
        if self.use_scan and self.avoid_enabled and self.scan_has_data and (self.last_scan is not None):
            msg_scan = self.last_scan
            s_idx, e_idx = self._front_sector_indices(msg_scan)
            if e_idx > s_idx:
                valid = [r for r in msg_scan.ranges[s_idx:e_idx+1] 
                        if not (math.isinf(r) or math.isnan(r))]
                dmin = min(valid) if valid else float("inf")
            else:
                dmin = float("inf")
            
            if dmin < self.danger_dist:
                gap = self._largest_safe_gap(msg_scan, s_idx, e_idx)
                if gap is not None:
                    gs, ge = gap
                    xt_gap, yt_gap = self._gap_midpoint_target(msg_scan, gs, ge, Ld_dynamic)
                    a = clamp(self.gap_mix_alpha, 0.0, 1.0)
                    xt = a * xt_gap + (1.0 - a) * xt
                    yt = a * yt_gap + (1.0 - a) * yt
                    used_gap = True
                    
                    # LOG para debug
                    self.get_logger().warn(
                        f"🚧 OBSTACLE! dmin={dmin:.2f}m | Using gap", 
                        throttle_duration_sec=0.5
                    )

        if used_gap: 
            self._publish_avoidance_marker(xt, yt)
        else:
            if self.publish_avoid_markers:
                arr = MarkerArray()
                delm = Marker(); delm.action = Marker.DELETEALL
                arr.markers.append(delm)
                self.pub_avoid_markers.publish(arr)

        # USAR LOOKAHEAD DINÁMICO en Pure Pursuit
        delta = self._pure_pursuit(xt, yt, Ld_dynamic)
        cmd.drive.steering_angle = float(delta)
        self.pub_cmd.publish(cmd)

        # ... (resto de tu código para publicar paths)

El Código ya Desacelera Gradualmente

La lógica para la aceleración y desaceleración controlada ya existe en la función _on_timer.

Cuando la localización de AMCL falla, la función _update_pose() devuelve False. Observa lo que sucede después en _on_timer:
Python

# DENTRO DE LA FUNCIÓN _on_timer

    # 1. Si pose_ok es False, la velocidad objetivo se convierte en 0.0
    v_target = (self.v_des if (pose_ok and self.autonomous_active) else 0.0)
    
    # 2. Se calcula el cambio máximo de velocidad por ciclo
    dv_max = self.ramp_rate / max(1.0, self.rate_hz)
    
    # 3. La velocidad publicada (v_pub) se reduce gradualmente hacia la objetivo (v_target)
    if self.v_pub < v_target: self.v_pub = min(self.v_pub + dv_max, v_target)
    else:                      self.v_pub = max(self.v_pub - dv_max, v_target) # <-- ESTA LÍNEA DESACELERA

    # 4. Se publica el comando con la velocidad ya reducida
    cmd = AckermannDriveStamped()
    cmd.header.stamp = self.get_clock().now().to_msg()
    cmd.drive.speed = float(self.v_pub)

Lo que ocurre es un proceso de rampa (ramping):

    La velocidad objetivo (v_target) pasa a ser 0.0 instantáneamente.

    Pero la velocidad real publicada (v_pub) no. En cada ciclo, se reduce en una pequeña cantidad (dv_max) hasta que alcanza 0.0.

Este comportamiento es exactamente una desaceleración controlada.

Cómo Ajustar la Tasa de Desaceleración ⚙️

Puedes controlar qué tan rápida o lenta es esta desaceleración modificando el parámetro ramp_rate. Este parámetro define el cambio máximo de velocidad en metros por segundo, por cada segundo.

Lo encuentras en la sección de declaración de parámetros:
Python

# DENTRO DE LA FUNCIÓN __init__

        # ...
        self.declare_parameter("steer_limit", 0.41)
        self.declare_parameter("ramp_rate", 2.0) # <-- ESTE ES EL PARÁMETRO
        self.declare_parameter("odom_timeout", 5.0)
        # ...

    Para una desaceleración más lenta y suave: Disminuye el valor de ramp_rate. Por ejemplo, 0.8.

    Para una desaceleración más rápida y brusca: Aumenta el valor de ramp_rate. Por ejemplo, 3.5.

Por lo tanto, con la modificación que hicimos para depender solo de AMCL, el comportamiento del vehículo ya es el que buscas: desacelerará hasta detenerse de forma controlada por ramp_rate en cuanto falle la señal de AMCL.

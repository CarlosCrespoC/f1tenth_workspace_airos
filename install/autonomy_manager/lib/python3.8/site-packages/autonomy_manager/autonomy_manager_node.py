import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.qos import QoSProfile, Reliability, DurabilityPolicy
class AutonomyManager(Node):
    def __init__(self):
        super().__init__('autonomy_manager')
        # Parámetro para el índice del botón R1 (5 por defecto)
        self.r1_button_index = self.declare_parameter('r1_button_index', 5).get_parameter_value().integer_value
        
        # Variable para gestionar el estado del modo autónomo
        self.autonomous_active = False
        self.last_r1_state = 0

        # Suscriptor al topic "raw" del algoritmo
        self.algo_raw_sub = self.create_subscription(
            AckermannDriveStamped, '/algo_drive_raw', self.algo_raw_callback, 10)
        
        # Suscriptor al joystick para leer el botón R1
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # Publicador que envía los comandos al multiplexor (mux)
        #self.mux_pub = self.create_publisher(
        #    AckermannDriveStamped, '/autonomous', 10)
	qos_cmd = QoSProfile(depth=10,
                     reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
	self.mux_pub = self.create_publisher(AckermannDriveStamped, '/drive', qos_cmd)
        self.get_logger().info('✅ Autonomy Manager listo. Presiona R1 para activar/desactivar.')

    def joy_callback(self, msg):
        current_r1_state = msg.buttons[self.r1_button_index]
        # Detecta solo la pulsación (flanco de subida) del botón R1
        if current_r1_state == 1 and self.last_r1_state == 0:
            self.autonomous_active = not self.autonomous_active
            mode_str = "ACTIVADA" if self.autonomous_active else "DESACTIVADA"
            self.get_logger().info(f'--- Autonomía {mode_str} ---')
            
            # Si se desactiva, enviar un comando de parada al mux por seguridad
            if not self.autonomous_active:
                stop_cmd = AckermannDriveStamped()
                stop_cmd.drive.speed = 0.0
                self.mux_pub.publish(stop_cmd)

        self.last_r1_state = current_r1_state

    def algo_raw_callback(self, msg):
        # Si el modo autónomo está activo, reenvía el mensaje del algoritmo al mux.
        # Si no, esta función no hace nada (la compuerta está cerrada).
        if self.autonomous_active:
            self.mux_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AutonomyManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

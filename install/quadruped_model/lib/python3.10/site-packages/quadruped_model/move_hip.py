import rclpy
from rclpy.node import Node
import time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3
import numpy as np

class HipController(Node):

    def __init__(self):
        super().__init__('hip_controller')
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.current_position = Vector3()
        self.current_position.x = 0.0
        self.current_position.y = 0.0
        self.current_position.z = 0.1

        # Longitudes de las piernas
        self.L1 = 0.177  # Longitud del segmento superior
        self.L2 = 0.180  # Longitud del segmento inferior

        # Configuración de trayectoria
        self.num_puntos = 100  # Número de puntos en la trayectoria
        self.amplitud = 0.1  # Desplazamiento en metros

    def cinematica_directa(self, theta1, theta2):
        """Calcula la posición cartesiana de la cadera dado los ángulos articulares."""
        x = self.L1 * np.cos(theta1) + self.L2 * np.cos(theta1 + theta2)
        z = self.L1 * np.sin(theta1) + self.L2 * np.sin(theta1 + theta2)
        return x, z

    def cinematica_inversa(self, x, z):
        """Calcula los ángulos articulares dados los puntos cartesianos de la cadera."""
        d = np.sqrt(x**2 + z**2)
        if d > self.L1 + self.L2 or d < abs(self.L1 - self.L2):
            raise ValueError("La posición está fuera del alcance.")

        cos_theta2 = (x**2 + z**2 - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
        theta2 = np.arctan2(np.sqrt(1 - cos_theta2**2), cos_theta2)
        theta1 = np.arctan2(z, x) - np.arctan2(self.L2 * np.sin(theta2), self.L1 + self.L2 * np.cos(theta2))
        return theta1, theta2

    def generar_trayectoria_cadera(self, x_inicial, amplitud, num_puntos):
        """Genera una trayectoria rectilínea para la cadera en el eje X."""
        x = np.linspace(x_inicial - amplitud, x_inicial , num_puntos)
        z = np.full_like(x, 0.0)  # Mantener altura constante de la cadera
        return x, z

    def calcular_trayectoria(self):
        """Calcula y publica los ángulos necesarios para que la cadera siga la trayectoria."""
        # Posición inicial del pie (referencia fija)
        x_pie = 0.0
        z_pie = -0.3

        # Generar la trayectoria de la cadera
        x_trayectoria, z_trayectoria = self.generar_trayectoria_cadera(x_pie, self.amplitud, self.num_puntos)

        # Calcular ángulos para cada punto de la trayectoria de la cadera
        angulos_articulaciones = []
        for x, z in zip(x_trayectoria, z_trayectoria):
            try:
                # Calcular las posiciones relativas de la cadera respecto al pie
                dx = x - x_pie
                dz = z - z_pie
                theta1, theta2 = self.cinematica_inversa(dx, dz)
                angulos_articulaciones.append((theta1, theta2))
            except ValueError as e:
                self.get_logger().info(f"Error en posición ({x}, {z}): {e}")
                angulos_articulaciones.append((np.nan, np.nan))

        # Publicar los ángulos de las articulaciones
        joint_names = [
            'leg_FR_joint', 'foot_FR_joint',
            'leg_FL_joint', 'foot_FL_joint',
            'leg_BR_joint', 'foot_BR_joint',
            'leg_BL_joint', 'foot_BL_joint'
        ]
        
        for angulos in angulos_articulaciones:
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.name = joint_names

            # Aplicar los ángulos correctamente a las patas, asegurando que el pie permanezca fijo
            joint_state_msg.position = [
                -angulos[0], angulos[1] -3.1416,  # Frente derecha
                angulos[0], -angulos[1]+3.1416,  # Frente izquierda
                -angulos[0] -3.1416, angulos[1] -3.1416,  # Atrás derecha
                angulos[0] +3.1416, -angulos[1] +3.1416  # Atrás izquierda
            ]
            self.publisher_.publish(joint_state_msg)
            time.sleep(0.05)

        self.get_logger().info("Trayectoria completada.")

def main(args=None):
    rclpy.init(args=args)
    hip_controller = HipController()

    # Crear un temporizador para calcular la trayectoria periódicamente
    timer_period = 2.0  # Segundos
    hip_controller.create_timer(timer_period, hip_controller.calcular_trayectoria)

    rclpy.spin(hip_controller)
    hip_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
import time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3
import numpy as np


class LegController(Node):

    def __init__(self):
        super().__init__('leg_controller')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10)
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        timer_period = 0.1  # Segundos
        self.current_position = Vector3()
        self.current_position.x = 0.0
        self.current_position.y = 0.0
        self.current_position.z = 0.0
        self.intercalado = 1
        self.R = 1.0  # Radio de la rueda en metros
        self.num_puntos = 100  # Número de puntos en    la trayectoria
        self.punto_inicial = (0, 0)  # Punto inicial en (x, y)
        self.trajectory_index = 0
        self.joint_state = []
        self.punto_pie = []
        self.L1 = 0.177  # Longitud del primer segmento en metros
        self.L2 = 0.180

    def joint_state_callback(self, msg):
        self.joint_state = msg.position

    def cinemática_directa(self, theta1, theta2, L1, L2):
        x = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
        y = L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2)
        return x, y

    def cinemática_inversa(self, x, y, L1, L2):
        self.get_logger().info(f'Coordenadas ({x}, {y})')
        D = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
        if np.abs(D) > 1:
            raise ValueError("No es posible alcanzar la posición dada con las longitudes de los segmentos proporcionados")
        
        theta2 = np.arctan2(np.sqrt(1 - D**2), D)
        theta1 = np.arctan2(y, x) - np.arctan2(L2 * np.sin(theta2), L1 + L2 * np.cos(theta2))
        self.get_logger().info(f'Coordenadas ({theta1}, {theta2})')
        return theta1, theta2

    def cinematica_inversa_hip(self, x, z):
        """Calcula los ángulos articulares dados los puntos cartesianos de la cadera."""
        d = np.sqrt(x**2 + z**2)
        if d > self.L1 + self.L2 or d < abs(self.L1 - self.L2):
            raise ValueError("La posición está fuera del alcance.")

        cos_theta2 = (x**2 + z**2 - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
        theta2 = np.arctan2(np.sqrt(1 - cos_theta2**2), cos_theta2)
        theta1 = np.arctan2(z, x) - np.arctan2(self.L2 * np.sin(theta2), self.L1 + self.L2 * np.cos(theta2))
        return theta1, theta2

    def generar_trayectoria_cadera(self, x_punta_pie, amplitud):
        """Genera una trayectoria rectilínea de la cadera, fijando la punta del pie."""
        x = np.linspace(x_punta_pie, x_punta_pie + amplitud, self.num_puntos)
        z = np.full_like(x, 0.0)
        return x, z

    def generar_trayectoria_cicloide(self, punto_inicial, R, num_puntos=100):
        x0, y0 = punto_inicial
        t = np.linspace(0, 2 * np.pi, num_puntos)
        x = x0 + R * (t - np.sin(t)) -(2*np.pi*R)
        y = y0 - R * (1 - np.cos(t))
        return x, y

    def calcular_trayectoria(self):
          # Longitud del segundo segmento en metros
        R = 0.020  # Radio de la rueda en metros
        num_puntos = 50  # Número de puntos en la trayectoria

        
        # Obtener la posición actual usando cinemática directa
        self.joint_state = [1.047, 1.047]
        theta1, theta2 = self.joint_state[:2]
        x_actual, y_actual = self.cinemática_directa(theta1, theta2, self.L1, self.L2)
        
        # Generar la trayectoria cicloidal desde la posición actual
        x_trayectoria, y_trayectoria = self.generar_trayectoria_cicloide((x_actual, y_actual), R, num_puntos)
        self.punto_pie = [x_trayectoria[0],y_trayectoria[0]]
        x_trayectoria_recta, z_trayectoria_recta = self.generar_trayectoria_cadera(self.punto_pie[0], 2*np.pi*R)
    
        angulos_articulaciones_cadera = []
        for x, z in zip(x_trayectoria_recta, z_trayectoria_recta):
            try:
                # Calcular las posiciones relativas de la cadera respecto al pie
                dx = x - self.punto_pie[0]
                dz = z + self.punto_pie[1]
                theta1, theta2 = self.cinematica_inversa_hip(dx, dz)
                angulos_articulaciones_cadera.append((theta1, theta2))
            except ValueError as e:
                self.get_logger().info(f"Error en posición ({x}, {z}): {e}")
                angulos_articulaciones_cadera.append((np.nan, np.nan))

        # Calcular los ángulos de las articulaciones para cada punto de la trayectoria
        angulos_articulaciones = []
        for x, y in zip(x_trayectoria, y_trayectoria):
            try:
                theta1, theta2 = self.cinemática_inversa(x,y, self.L1, self.L2)
                angulos_articulaciones.append((theta1, theta2))
            except ValueError as e:
                self.get_logger().info(f"Error en la posición ({x}, {y}): {e}")
                angulos_articulaciones.append((np.nan, np.nan))
        joint_names = [
            'leg_FR_joint', 'foot_FR_joint',
            'leg_FL_joint', 'foot_FL_joint',
            'leg_BR_joint', 'foot_BR_joint',
            'leg_BL_joint', 'foot_BL_joint'
        ]
        angulos_articulaciones.reverse()
        while True :
            if self.intercalado % 2 == 0:
                i = 0
                for angulos in angulos_articulaciones:
                    joint_state_msg = JointState()
                    joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                    joint_state_msg.name = joint_names
                    joint_state_msg.position = [-angulos[0], angulos[1] -3.1416, self.joint_state[0], -self.joint_state[1]+3.1416, 
                                        -self.joint_state[0] -3.1416, self.joint_state[1] -3.1416, angulos[0] +3.1416 , -angulos[1]+3.1416]  # Example positions
                    #joint_state_msg.position = [-angulos[0], angulos[1] -3.1416, self.joint_state[0], -self.joint_state[1]+3.1416, 
                    #                            -self.joint_state[0] -3.1416, self.joint_state[1] -3.1416, angulos[0] +3.1416 , -angulos[1]+3.1416]  # Example positions
                    self.publisher_.publish(joint_state_msg)  
                    self.lifted = True
                    self.get_logger().info(f'Published joint states: {angulos}')
                    
                    time.sleep(0.05)
                self.get_logger().info('Robot lifted with joint states.')
                for angulos in angulos_articulaciones_cadera:
                    joint_state_msg = JointState()
                    joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                    joint_state_msg.name = joint_names

                    # Aplicar los ángulos correctamente a las patas, asegurando que el pie permanezca fijo
                    joint_state_msg.position = [
                        -angulos[0], angulos[1] -3.1416,  # Frente derecha
                        self.joint_state[0], -self.joint_state[1]+3.1416,  # Frente izquierda
                        -self.joint_state[0] -3.1416, self.joint_state[1] -3.1416,  # Atrás derecha
                        angulos[0] +3.1416, -angulos[1] +3.1416  # Atrás izquierda
                    ]
                    self.publisher_.publish(joint_state_msg)
                    i += 1
                    if i == len(angulos_articulaciones):
                        self.joint_state = [angulos[0], angulos[1]]
                        self.get_logger().info(f'Ultimos angulos: {angulos}')
                    time.sleep(0.05)
                self.get_logger().info("Trayectoria completada.")
                self.intercalado += 1
            
            else:
                i = 0
                for angulos in angulos_articulaciones:
                    joint_state_msg = JointState()
                    joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                    joint_state_msg.name = joint_names
                    joint_state_msg.position = [-self.joint_state[0], self.joint_state[1] -3.1416, angulos[0], -angulos[1]+3.1416, 
                                        -angulos[0] -3.1416, angulos[1] -3.1416, self.joint_state[0] +3.1416 , -self.joint_state[1]+3.1416]  # Example positions
                    #joint_state_msg.position = [-self.joint_state[0] -3.1416, self.joint_state[1] -3.1416, angulos[0] +3.1416 , -angulos[1]+3.1416
                    #                            -angulos[0] -3.1416, angulos[1] -3.1416, self.joint_state[0] +3.1416 , -self.joint_state[1]+3.1416]   # Example positions
                    self.publisher_.publish(joint_state_msg)
                    self.lifted = True
                    self.get_logger().info(f'Published joint states: {angulos}')
                    time.sleep(0.2)
                self.get_logger().info('Robot lifted with joint states.')
                for angulos in angulos_articulaciones_cadera:
                    joint_state_msg = JointState()
                    joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                    joint_state_msg.name = joint_names

                    # Aplicar los ángulos correctamente a las patas, asegurando que el pie permanezca fijo
                    joint_state_msg.position = [
                        -self.joint_state[0], self.joint_state[1] -3.1416,  # Frente derecha
                        angulos[0], -angulos[1]+3.1416,  # Frente izquierda
                        -angulos[0] -3.1416, angulos[1] -3.1416,  # Atrás derecha
                        self.joint_state[0] +3.1416, -self.joint_state[1] +3.1416  # Atrás izquierda
                    ]
                    self.publisher_.publish(joint_state_msg)
                    i += 1
                    if i == len(angulos_articulaciones):
                        self.joint_state = [angulos[0], angulos[1]]
                        self.get_logger().info(f'Ultimos angulos: {angulos}')
                    time.sleep(0.05)
                self.get_logger().info("Trayectoria completada.")
                self.intercalado += 1


        
def main(args=None):
    rclpy.init(args=args)
    leg_controller = LegController()

    # Crear un timer para calcular la trayectoria periódicamente
    timer_period = 5.0  # segundos
    leg_controller.create_timer(timer_period, leg_controller.calcular_trayectoria)

    rclpy.spin(leg_controller)

    leg_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
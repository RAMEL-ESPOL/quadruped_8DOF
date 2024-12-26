import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        #self.target_positions = [-0.57, -1.39, 0.57, 1.39, 2.57, 4.9, -2.57, -4.9]
        self.target_positions = [0.57, 1.39, -0.57, -1.39, -2.57, -4.9, 2.57, 4.9]
        self.current_positions = [0.0, 0.0 , 0.0, 0.0, -3.14, -6.24, 3.14, 6.24]
        self.increment = 0.01  # Incremento por cada paso
        self.timer = self.create_timer(0.1, self.update_joint_states)

    def update_joint_states(self):
        # Crear el mensaje JointState
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_names = [
            'leg_FR_joint', 'foot_FR_joint',
            'leg_FL_joint', 'foot_FL_joint',
            'leg_BR_joint', 'foot_BR_joint',
            'leg_BL_joint', 'foot_BL_joint'
        ] # Reemplaza con los nombres de tus joints

        # Incrementar la posición actual hasta llegar a la posición objetivo
        for i in range(len(self.current_positions)):
            #if i == 2 or i == 3 or i == 6 or i == 7:
            if i == 0 or i == 1 or i == 4 or i == 5:
                if self.current_positions[i] < self.target_positions[i]:
                    self.current_positions[i] += self.increment
                    if self.current_positions[i] > self.target_positions[i]:
                        self.current_positions[i] = self.target_positions[i]
            #if i == 0 or i == 1 or i == 4 or i == 5:
            if i == 2 or i == 3 or i == 6 or i == 7:
                if self.current_positions[i] > self.target_positions[i]:
                    self.current_positions[i] -= self.increment
                    if self.current_positions[i] < self.target_positions[i]:
                        self.current_positions[i] = self.target_positions[i]

        joint_state_msg.name = joint_names
        joint_state_msg.position = self.current_positions

        # Publicar el mensaje
        self.publisher_.publish(joint_state_msg)
        self.get_logger().info(f'Mensaje publicado: Joint positions: {self.current_positions}')

        # Detener la actualización si alcanzó las posiciones objetivo
        if self.current_positions == self.target_positions:
            self.get_logger().info('Posiciones objetivo alcanzadas')
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

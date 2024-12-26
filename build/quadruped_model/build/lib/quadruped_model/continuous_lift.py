import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numpy as np

class ContinuousJointMover(Node):
    def __init__(self):
        super().__init__('continuous_joint_mover')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.1, self.move_joints)
        self.joint_names = [
            'leg_FR_joint', 'foot_FR_joint',
            'leg_FL_joint', 'foot_FL_joint',
            'leg_BR_joint', 'foot_BR_joint',
            'leg_BL_joint', 'foot_BL_joint'
        ]
        self.target_positions = [-0.57, -1.4, +0.57, +1.4, 0.57, 1.4, -0.57, -1.4]
        self.current_positions = np.zeros(len(self.joint_names))
        self.step_size = 0.01  # Adjust this for smoother or faster movement

    def move_joints(self):
        if np.allclose(self.current_positions, self.target_positions, atol=self.step_size):
            self.get_logger().info('Joints have reached the target positions.')
            return

        # Calculate new positions by moving a step towards the target positions
        for i in range(len(self.joint_names)):
            if self.current_positions[i] < self.target_positions[i]:
                self.current_positions[i] += self.step_size
                if self.current_positions[i] > self.target_positions[i]:
                    self.current_positions[i] = self.target_positions[i]
            elif self.current_positions[i] > self.target_positions[i]:
                self.current_positions[i] -= self.step_size
                if self.current_positions[i] < self.target_positions[i]:
                    self.current_positions[i] = self.target_positions[i]

        joint_state_msg = JointState()
        joint_state_msg.header = Header()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = self.current_positions.tolist()

        self.publisher.publish(joint_state_msg)
        self.get_logger().info(f'Published joint states: {self.current_positions}')

def main(args=None):
    rclpy.init(args=args)
    node = ContinuousJointMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

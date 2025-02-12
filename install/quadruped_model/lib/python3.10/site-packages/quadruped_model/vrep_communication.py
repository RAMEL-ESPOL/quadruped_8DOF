#!/usr/bin/env python3

from time import sleep
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from sensor_msgs.msg import JointState
import rclpy
from rclpy.node import Node


class Vrep_Communication(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(f"{name} creado")
        self.joint_subscription = self.create_subscription(JointState, '/joint_states', self.listener_callback, 10)

        # Handles
        self.joint_handles = []

        # Crear cliente y conectarse al servidor
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        self.scriptHandle = self.sim.getObject('/moveJoints')

        try:
            # Comprobar conexión
            self.sim.startSimulation()
            self.get_logger().warn("Conexión establecida con el servidor.")

        except Exception as e:
            self.get_logger().error(f"Error durante la ejecución: {e}")
        

    def listener_callback(self, msg: JointState):
        print("joa")
        success = self.sim.callScriptFunction('sysCall_joint', self.scriptHandle, list(msg._position))


def main(args=None):
    rclpy.init(args=args)
    node = Vrep_Communication("vrep_communication")
    rclpy.spin(node)
    node.destroy_node()
    node.sim.stopSimulation()

    rclpy.shutdown()
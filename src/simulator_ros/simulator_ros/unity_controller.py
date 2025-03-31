#!/usr/bin/env python

import socket
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class UnityController(Node):
    def __init__(self):
        super().__init__('unity_controller')
    
        self.subscription = self.create_subscription(
            Twist, 
            '/cmd_vel', 
            self.cmd_vel_callback, 
            10)
        self.subscription

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        linear_z = msg.linear.z
        angular_x = msg.angular.x
        angular_y = msg.angular.y
        angular_z = msg.angular.z
        self.get_logger().info(f'Received cmd_vel: linear_x = {linear_x}, linear_y = {linear_y}, linear_z = {linear_z}, angular_x = {angular_x}, \
                               angular_y = {angular_y}, angular_z = {angular_z}')

        if linear_x > 0:
            self.send_command_to_unity("MoveAhead")
            self.get_logger().info('Moving forward!')
        elif linear_x < 0:
            self.get_logger().info('Moving backward!')
        if angular_z > 0:
            self.get_logger().info('Turning left!')
        elif angular_z < 0:
            self.get_logger().info('Turning right!')

    def send_command_to_unity(self, message):
        try:
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.connect(('localhost', 8888))
            client_socket.send(message.encode())
            client_socket.close()
        except ConnectionRefusedError:
            self.get_logger().error("Failed to connect to Unity server!")

def main(args=None):
    rclpy.init(args=args)
    node = UnityController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

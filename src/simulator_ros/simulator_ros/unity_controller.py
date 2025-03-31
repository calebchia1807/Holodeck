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

        self.height_checker = True  # True = Standing, False = Crouching

        self.get_logger().info('Starting Unity Controller Node')

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        linear_z = msg.linear.z
        angular_x = msg.angular.x
        angular_y = msg.angular.y
        angular_z = msg.angular.z
        # self.get_logger().info(f'Received cmd_vel: linear_x = {linear_x}, linear_y = {linear_y}, linear_z = {linear_z}, angular_x = {angular_x}, \
        #                        angular_y = {angular_y}, angular_z = {angular_z}')

        if (linear_x > 0 and angular_z > 0):
            self.send_command_to_unity("RotateLeft")
            self.get_logger().info('Turning left!')       
        elif (linear_x > 0 and angular_z < 0):
            self.send_command_to_unity("RotateRight")
            self.get_logger().info('Turning right!')  
        elif (linear_x < 0 and angular_z < 0):
            self.send_command_to_unity("LookUp")
            self.get_logger().info('Looking up!')  
        elif (linear_x < 0 and angular_z > 0):
            self.send_command_to_unity("LookDown")
            self.get_logger().info('Looking down!')
        elif linear_x > 0:
            self.send_command_to_unity("MoveAhead")
            self.get_logger().info('Moving forward!')
        elif linear_x < 0:
            self.send_command_to_unity("MoveBack")
            self.get_logger().info('Moving backward!')
        elif angular_z > 0:
            self.send_command_to_unity("MoveLeft")
            self.get_logger().info('Moving left!')
        elif angular_z < 0:
            self.send_command_to_unity("MoveRight")
            self.get_logger().info('Moving right!')
        elif (linear_x == 0 and angular_z == 0):
            if self.height_checker == True:
                self.send_command_to_unity("Crouch")
                self.get_logger().info('Crouching down!') 
                self.height_checker = False
            elif self.height_checker == False:
                self.send_command_to_unity("Stand")
                self.get_logger().info('Standing up!') 
                self.height_checker = True

    def send_command_to_unity(self, message):
        try:
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.connect(('localhost', 8888))
            client_socket.send(message.encode())
            client_socket.close()
        except ConnectionRefusedError:
            self.get_logger().error("Failed to connect to Unity server!")

def main(args=None):
    try:
        rclpy.init(args=args)
        node = UnityController()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

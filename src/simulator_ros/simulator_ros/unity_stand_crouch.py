#!/usr/bin/env python3

import socket
import json
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class UnityStandCrouchService(Node):
    def __init__(self):
        super().__init__('unity_stand_crouch')
    
        self.stand_srv = self.create_service(Trigger, '~/stand', self.stand_callback)
        self.crouch_srv = self.create_service(Trigger, '~/crouch', self.crouch_callback)

        self.get_logger().info('Starting unity stand & crouch ROS service')

    def stand_callback(self, request, response):
        try:
            command = {"action": "Stand"}
            command_json = json.dumps(command)
            self.send_command_to_unity(command_json)
            self.get_logger().info("Standing")
            response.success = True
            response.message = "Stand command executed successfully!"
        except Exception as e:
            self.get_logger().error(f'Failed to send stand command: {e}')
            response.success = False
            response.message = f'Error: {e}'
        
        return response

    def crouch_callback(self, request, response):
        try:
            command = {"action": "Crouch"}
            command_json = json.dumps(command)
            self.send_command_to_unity(command_json)
            self.get_logger().info("Crouching")
            response.success = True
            response.message = "Crouch command executed successfully!"
        except Exception as e:
            self.get_logger().error(f'Failed to send crouch command: {e}')
            response.success = False
            response.message = f'Error: {e}'
        
        return response

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
    node = UnityStandCrouchService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

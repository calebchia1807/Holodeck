#!/usr/bin/env python

import socket
import json
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class UnityNav(Node):
    def __init__(self):
        super().__init__('unity_nav')
    
        self.subscription = self.create_subscription(
            Twist, 
            '/cmd_vel', 
            self.cmd_vel_callback, 
            10)
        self.subscription

        self.get_logger().info('Starting Unity Nav Node')

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x     # left / right
        linear_y = msg.linear.y     # front / back
        linear_z = msg.linear.z     # up / down
        angular_x = msg.angular.x   # row
        angular_y = msg.angular.y   # pitch
        angular_z = msg.angular.z   # rotate left / right - yaw

        commands = []
        timestep = 0.02        

        # move forward & backward
        if linear_y != 0:
            distance_y = abs(linear_y)
            move_magnitude_y = distance_y / 1 * timestep
            steps_linear_y = int(distance_y / move_magnitude_y)
            
            for i in range(steps_linear_y):
                if linear_y > 0:
                    commands.append({"action": "MoveAhead", "moveMagnitude": move_magnitude_y})
                elif linear_y < 0:
                    commands.append({"action": "MoveBack", "moveMagnitude": move_magnitude_y})

        # move left & right
        if linear_x != 0:
            distance_x = abs(linear_x)  
            move_magnitude_x = distance_x / 1 * timestep
            steps_linear_x = int(distance_x / move_magnitude_x)

            for i in range(steps_linear_x):
                if linear_x > 0:
                    commands.append({"action": "MoveRight", "moveMagnitude": move_magnitude_x})
                elif linear_x < 0:
                    commands.append({"action": "MoveLeft", "moveMagnitude": move_magnitude_x})

        # rotate left & right (user to ros2 topic pub in degrees/s for angular_z)   --> CAN CHANGE TO RAD/S IF NEEDED!!!
        if angular_z != 0:
            angle_rotated_z = abs(angular_z)
            rotate_magnitude_z = angle_rotated_z / 1 * timestep 
            steps_angular_z = int(angle_rotated_z / rotate_magnitude_z)

            for i in range(steps_angular_z):
                if angular_z > 0:
                    commands.append({"action": "RotateLeft", "degrees": rotate_magnitude_z})
                elif angular_z < 0:
                    commands.append({"action": "RotateRight", "degrees": rotate_magnitude_z})

        if commands:
            for command in commands:
                command_json = json.dumps(command)
                self.send_command_to_unity(command_json)
                self.get_logger().info(command_json)

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
        node = UnityNav()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

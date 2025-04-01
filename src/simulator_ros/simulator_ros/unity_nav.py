#!/usr/bin/env python

import socket
import json
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

        # timestep = 0.02
        # move_magnitude = 0.01

        # commands = []

        timestep = 0.02
        distance_to_travel = abs(linear_y)
        move_magnitude = distance_to_travel / 1 * timestep

        commands = []
        if linear_y != 0:
            distance_travelled = abs(linear_y)
            steps = int(distance_travelled / move_magnitude)

            for i in range(steps):
                if linear_y > 0:
                    commands.append({"action": "MoveAhead", "moveMagnitude": move_magnitude})
                elif linear_y < 0:
                    commands.append({"action": "MoveBack", "moveMagnitude": move_magnitude})

        # if linear_y != 0:
        #     distance_travelled = abs(linear_y)*timestep
        #     steps = int(distance_travelled/move_magnitude)
        #     for i in range(steps):   
        #         if linear_y > 0:
        #             commands.append({"action": "MoveAhead", "moveMagnitude": move_magnitude})
        #         elif linear_y < 0:
        #             commands.append({"action": "MoveBack", "moveMagnitude": move_magnitude})

        # if linear_y > 0:
        #     commands.append({"action": "MoveAhead", "moveMagnitude": abs(linear_y)})
        # elif linear_y < 0:
        #     commands.append({"action": "MoveBack", "moveMagnitude": abs(linear_y)})

        # if linear_x > 0:
        #     commands.append({"action": "MoveRight", "moveMagnitude": abs(linear_x)})
        # elif linear_x < 0:
        #     commands.append({"action": "MoveLeft", "moveMagnitude": abs(linear_x)})

        # if angular_z > 0:
        #     commands.append({"action": "RotateLeft", "degrees": abs(angular_z)})
        # elif angular_z < 0:
        #     commands.append({"action": "RotateRight", "degrees": abs(angular_z)})

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

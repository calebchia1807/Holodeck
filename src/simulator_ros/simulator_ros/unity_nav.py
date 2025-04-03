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

        self.get_logger().info('Starting unity_nav ROS node')

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x     # front / back
        linear_y = msg.linear.y     # left / right
        linear_z = msg.linear.z     # NA
        angular_x = msg.angular.x   # look up / down - row
        angular_y = msg.angular.y   # NA - pitch
        angular_z = msg.angular.z   # rotate left / right - yaw

        commands = []
        timestep = 0.02        

        # move forward & backward
        if linear_x != 0:
            velocity_x = abs(linear_x)
            move_magnitude_x = velocity_x / 1 * timestep
            steps_linear_x = int(velocity_x / move_magnitude_x)

            if linear_x > 0:
                self.get_logger().info(f"Moving Ahead @ {velocity_x} m/s")
            elif linear_x < 0:
                self.get_logger().info(f"Moving Back @ {velocity_x} m/s")
    
            for i in range(steps_linear_x):
                if linear_x > 0:
                    commands.append({"action": "MoveAhead", "moveMagnitude": move_magnitude_x})
                elif linear_x < 0:
                    commands.append({"action": "MoveBack", "moveMagnitude": move_magnitude_x})

        # move left & right
        if linear_y != 0:
            velocity_y = abs(linear_y)
            move_magnitude_y = velocity_y / 1 * timestep
            steps_linear_y = int(velocity_y / move_magnitude_y)
            
            if linear_y > 0:
                self.get_logger().info(f"Moving Right @ {velocity_y} m/s")
            elif linear_y < 0:
                self.get_logger().info(f"Moving Left @ {velocity_y} m/s")
            
            for i in range(steps_linear_y):
                if linear_y > 0:
                    commands.append({"action": "MoveRight", "moveMagnitude": move_magnitude_y})
                elif linear_y < 0:
                    commands.append({"action": "MoveLeft", "moveMagnitude": move_magnitude_y})

        # look up & down
        if angular_x != 0:
            # angle_velocity_x = abs(math.degrees(angular_x))    # angular_x in radians - conversion needed as the command sent need to be in degrees!!!
            angle_velocity_x = abs(angular_x)                    # angular_x in degrees
            rotate_magnitude_x = angle_velocity_x / 1 * timestep 
            steps_angular_x = int(angle_velocity_x / rotate_magnitude_x)

            if angular_x > 0:
                self.get_logger().info(f"Looking up @ {angle_velocity_x} degrees/s")
                #self.get_logger().info(f"Looking up @ {abs(angular_x)} rad/s")
            elif angular_x < 0:
                self.get_logger().info(f"Looking down @ {angle_velocity_x} degrees/s")
                #self.get_logger().info(f"Looking down @ {abs(angular_x)} rad/s")


            for i in range(steps_angular_x):
                if angular_x > 0:
                    commands.append({"action": "LookUp", "degrees": rotate_magnitude_x})
                elif angular_x < 0:
                    commands.append({"action": "LookDown", "degrees": rotate_magnitude_x})

        # rotate left & right (range btw -30 degrees to 30 degrees only)
        if angular_z != 0:
            # angle_velocity_z = abs(math.degrees(angular_z))    # angular_z in radians
            angle_velocity_z = abs(angular_z)                    # angular_z in degrees
            rotate_magnitude_z = angle_velocity_z / 1 * timestep 
            steps_angular_z = int(angle_velocity_z / rotate_magnitude_z)

            if angular_z > 0:
                self.get_logger().info(f"Rotating Left @ {angle_velocity_z} degrees/s")
                #self.get_logger().info(f"Rotating Left @ {abs(angular_z)} rad/s")
            elif angular_z < 0:
                self.get_logger().info(f"Rotating Right @ {angle_velocity_z} degrees/s")
                #self.get_logger().info(f"Rotating Right @ {abs(angular_z)} rad/s")

            for i in range(steps_angular_z):
                if angular_z > 0:
                    commands.append({"action": "RotateLeft", "degrees": rotate_magnitude_z})
                elif angular_z < 0:
                    commands.append({"action": "RotateRight", "degrees": rotate_magnitude_z})

        if commands:
            for command in commands:
                command_json = json.dumps(command)
                self.send_command_to_unity(command_json)

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

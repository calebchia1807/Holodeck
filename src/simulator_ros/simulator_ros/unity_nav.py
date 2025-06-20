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

        '''
        FOR TELEOP_TWIST KEYBOARD
        -------------------------
        k/K: stop

        i/I: move forward
        ,/<: move back
        
        J: move left
        L: move right
        j: rotate left
        l: rotate right

        u: move forward & rotate left
        o: move forward & rotate right
        U: move forward & move left
        O: move forward & move right

        m: move back & rotate right
        .: move back & rotate left
        M: move back & move left
        >: move back & move right

        t: stand
        b: crouch

        
        FOR /cmd_vel PUBLICATION
        ------------------------
        ros2 topic pub -r 50 cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
        ** -r 50    >>> 50hz as timestep in unity is 0.02
        ** input values for linear xyz & angular xyz
        '''

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x     # front / back
        linear_y = msg.linear.y     # left / right
        linear_z = msg.linear.z     # stand / crouch
        angular_x = msg.angular.x   # NA - row
        angular_y = msg.angular.y   # look up / down - pitch
        angular_z = msg.angular.z   # rotate left / right - yaw
        
        timestep = 0.02
        
        # no motion
        if linear_x == 0 and linear_y == 0 and linear_z == 0 and angular_x == 0 and angular_y == 0 and angular_z == 0:
            self.send_command_to_unity(json.dumps({"action": "MoveAhead", "moveMagnitude": 0}))
            self.send_command_to_unity(json.dumps({"action": "MoveLeft", "moveMagnitude": 0}))
            self.send_command_to_unity(json.dumps({"action": "RotateLeft", "degrees": 0}))

        # move forward & backward
        if linear_x != 0:
            velocity_x = abs(linear_x)
            move_magnitude_x = velocity_x * timestep

            if linear_x > 0:
                self.get_logger().info(f"Moving Ahead @ {velocity_x} m/s")
                self.send_command_to_unity(json.dumps({"action": "MoveAhead", "moveMagnitude": move_magnitude_x}))
            elif linear_x < 0:
                self.get_logger().info(f"Moving Back @ {velocity_x} m/s")
                self.send_command_to_unity(json.dumps({"action": "MoveBack", "moveMagnitude": move_magnitude_x}))

        # move left & right
        if linear_y != 0:
            velocity_y = abs(linear_y)
            move_magnitude_y = velocity_y * timestep
            
            if linear_y > 0:
                self.get_logger().info(f"Moving Left @ {velocity_y} m/s")
                self.send_command_to_unity(json.dumps({"action": "MoveLeft", "moveMagnitude": move_magnitude_y}))
            elif linear_y < 0:
                self.get_logger().info(f"Moving Right @ {velocity_y} m/s")
                self.send_command_to_unity(json.dumps({"action": "MoveRight", "moveMagnitude": move_magnitude_y}))

        # stand & crouch
        if linear_z != 0:
            velocity_z = abs(linear_z)
            
            if linear_z > 0:
                self.get_logger().info(f"Standing")
                self.send_command_to_unity(json.dumps({"action": "Stand"}))
            elif linear_z < 0:
                self.get_logger().info(f"Crouching")
                self.send_command_to_unity(json.dumps({"action": "Crouch"}))

        # look up & down
        if angular_y != 0:
            # angle_velocity_x = abs(math.degrees(angular_x))    # angular_x in radians - conversion needed as the command sent need to be in degrees!!!
            angle_velocity_y = abs(angular_y)                    # angular_x in degrees
            rotate_magnitude_y = angle_velocity_y * timestep 

            if angular_y > 0:
                self.get_logger().info(f"Looking up @ {angle_velocity_y} degrees/s")
                #self.get_logger().info(f"Looking up @ {abs(angular_x)} rad/s")
                self.send_command_to_unity(json.dumps({"action": "LookUp", "degrees": rotate_magnitude_y}))
            elif angular_y < 0:
                self.get_logger().info(f"Looking down @ {angle_velocity_y} degrees/s")
                #self.get_logger().info(f"Looking down @ {abs(angular_x)} rad/s")
                self.send_command_to_unity(json.dumps({"action": "LookDown", "degrees": rotate_magnitude_y}))

        # rotate left & right (range btw -30 degrees to 30 degrees only)
        if angular_z != 0:
            # angle_velocity_z = abs(math.degrees(angular_z))    # angular_z in radians
            angle_velocity_z = abs(angular_z)                    # angular_z in degrees
            rotate_magnitude_z = angle_velocity_z * timestep 

            if angular_z > 0:
                self.get_logger().info(f"Rotating Left @ {angle_velocity_z} degrees/s")
                #self.get_logger().info(f"Rotating Left @ {abs(angular_z)} rad/s")
                self.send_command_to_unity(json.dumps({"action": "RotateLeft", "degrees": rotate_magnitude_z}))
            elif angular_z < 0:
                self.get_logger().info(f"Rotating Right @ {angle_velocity_z} degrees/s")
                #self.get_logger().info(f"Rotating Right @ {abs(angular_z)} rad/s")
                self.send_command_to_unity(json.dumps({"action": "RotateRight", "degrees": rotate_magnitude_z}))

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
    node = UnityNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
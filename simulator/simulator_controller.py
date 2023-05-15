#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from turtlesim.msg import Pose as TPose
from tf_transformations import euler_from_quaternion
import math
import csv 
from collections import deque

MAX_DIFF = 0.1

class MissionControl(deque):

    def __init__(self, csv_file="/home/giovanna/ros2_ws/src/simulator/simulator/csv_positions.csv"):
        super().__init__()
        with open(csv_file) as csv_file_var:
            csv_reading = csv.reader(csv_file_var, delimiter=',')
            for row in csv_reading:
                new_pose = Pose()
                new_pose.x, new_pose.y = [float(x) for x in row]
                self.enqueue(new_pose)

    def enqueue(self, x):
        super().append(x)

    def dequeue(self):
        return super().popleft()
    
class Pose(TPose):

    def __init__(self, x=0.0, y=0.0, theta=0.0):
        super().__init__(x=x, y=y, theta=theta)
        
    def __repr__(self):
        return f"(x={self.x}, y={self.y}, theta={self.theta})"
    
    def __add__(self, other):
        self.x += other.x
        self.y += other.y
        return self
    
    def __sub__(self, other):
        self.x -= other.x
        self.y -= other.y
        return self
    
    def __eq__(self, other):
        return abs(self.x - other.x) < MAX_DIFF and abs(self.y - other.y) < MAX_DIFF
        
class SimulatorControler(Node):
    def __init__(self, mission_control,control_period=0.02):
        super().__init__("simulator_controller")

        self.pose = Pose(x = -10.0)
        self.setpoint = Pose(x = -10.0)

        self.mission_control = mission_control

        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        self.subscription = self.create_subscription(Odometry, "/odom", self.pose_callback, 10)

        self.control_timer = self.create_timer(control_period, self.control_callback)

    def control_callback(self):
        if self.pose.x == -10.0:
            self.get_logger().info("Ainda não temos a primeira posição")

        msg = Twist()

        x_diff = self.setpoint.x - self.pose.x
        y_diff = self.setpoint.y - self.pose.y
        setpoint_ang = math.atan2(y_diff, x_diff)
        theta_diff = setpoint_ang - self.pose.theta

        if (abs(x_diff) < MAX_DIFF and abs(y_diff) < MAX_DIFF):
            self.update_setpoint()

        if abs(theta_diff) > (MAX_DIFF -0.05):
            msg.linear.x = 0.0
            msg.angular.z = 0.2 if theta_diff > 0 else -0.2

        elif abs(x_diff) > MAX_DIFF:
            msg.linear.x = 0.2

        if IndexError:
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.angular.z = 0.0
            self.get_logger().info("Terminamos a rota")
            exit()
        
        self.publisher.publish(msg)

    def update_setpoint(self):
        self.setpoint = self.pose + self.mission_control.dequeue()
        self.get_logger().info(f"A próxima posição é {self.setpoint}")

    def pose_callback(self, msg):

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        ang = msg.pose.pose.orientation

        __, __, theta = euler_from_quaternion([ang.x, ang.y, ang.z, ang.w])

        self.pose = Pose(x=x, y=y, theta=theta)

        if self.setpoint.x == -10.0:
            self.update_setpoint()

        self.get_logger().info(f"Estamos em x={msg.x}, y={msg.y}, theta={msg.theta}")

def main(args=None):
    rclpy.init(args=args)
    mc = MissionControl()
    node = SimulatorControler(mc)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    






#!usr/bin/env python

import rclpy
import json
from mapping import OccupancyGridMap
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import numpy as np
import math
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


# def quaternion_from_euler(roll, pitch, yaw):
#     """
#     Converts euler roll, pitch, yaw to quaternion
#     """
#     cy = math.cos(yaw * 0.5)
#     sy = math.sin(yaw * 0.5)
#     cp = math.cos(pitch * 0.5)
#     sp = math.sin(pitch * 0.5)
#     cr = math.cos(roll * 0.5)
#     sr = math.sin(roll * 0.5)

#     q = Quaternion()
#     q.w = cy * cp * cr + sy * sp * sr
#     q.x = cy * cp * sr - sy * sp * cr
#     q.y = sy * cp * sr + cy * sp * cr
#     q.z = sy * cp * cr - cy * sp * sr
#     return q 

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class Mapper:
    def __init__(self, map_size: list[list] = [[-10, 10], [-10, 10]], grid_size: float = 0.1, z_max: float = 3.4, 
                 sensor_offset: list = [0, 0], n_beams: int = 360, angle_range: list = [0, 2*np.pi],
                 p_occ: float = 0.9, p_free: float = 0.4) -> None:
                 
        self.map = OccupancyGridMap(xrange=map_size[0], yrange=map_size[1], grid_size=grid_size, 
                                    z_max=z_max, sensor_offset=sensor_offset, n_beams=n_beams,
                                     angle_range=angle_range, p_occ=p_occ, p_free=p_free)

        self.scan = None
        self.pose = {}

        plt.figure(figsize=(10, 10))
        plt.pause(0.1)

        self.update_node = SubscriberNode("mapper", self)
        self.map_node = MapPublisher("map_publisher")
        rclpy.spin(self.update_node)
        rclpy.spin(self.map_node)

    def __del__(self):
        self.map_node.destroy_node()

    def callback_scan(self, msg):
        self.scan = msg.ranges


    def callback_update(self, msg):
        # self.update_node.get_logger().info("update_node callback")
        self.pose["x"] = msg.pose.pose.position.x
        self.pose["y"] = msg.pose.pose.position.y
        quaternions = msg.pose.pose.orientation
        euler = euler_from_quaternion(quaternions)
        self.pose["theta"] = euler[2]
        if self.scan is None:
            return
        self.map.live_update_map(np.array([self.pose["x"], self.pose["y"], self.pose["theta"]]), np.array(self.scan))
        self.map_node.publish_callback(1.0 - 1./(1.+np.exp(self.map.odds_map)))
        plt.clf()
        plt.imshow(1.0 - 1./(1.+np.exp(self.map.odds_map)), 'Greys')
        plt.pause(0.01)


class ScanNode(Node):
    
    def __init__(self, name, parent):
        super().__init__(name)
        self.subscription_scan = self.create_subscription(LaserScan, "/scan", self.callback_scan, 10)
        self.parent = parent

    def callback_scan(self, msg):
        self.parent.callback_scan(msg)


class SubscriberNode(Node):
    
    def __init__(self, name, parent):
        super().__init__(name)
        self.subscription_scan = self.create_subscription(LaserScan, "/scan", self.callback_scan, 10)
        self.subscription_pose = self.create_subscription(Odometry, "/odom", self.callback_odom, 10)
        self.parent = parent

    def callback_scan(self, msg):
        self.parent.callback_scan(msg)

    def callback_odom(self, msg):
        self.parent.callback_update(msg)


class MapPublisher(Node):
    def __init__(self, name):
        super().__init__(name)
        self.publisher = self.create_publisher(Float64MultiArray, 'map', 10)
    
    def publish_callback(self, data):
        msg = Float64MultiArray()
        data = data.flatten()
        converted_data = getattr(data, "tolist", lambda: data)()
        print(type(converted_data))
        msg.data = (converted_data)
        self.publisher.publish(msg)


if __name__ == '__main__':
    rclpy.init()
    mapper = Mapper(map_size=[[-10, 10], [-10, 10]], grid_size=0.1, z_max=5, n_beams=360, angle_range=[0, 2*np.pi],
                    p_occ=0.9, p_free=0.4)
    
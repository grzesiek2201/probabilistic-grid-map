#!usr/bin/env python

import rclpy
import json
from .occupancy_grid import OccupancyGridMap ########################
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
import matplotlib.pyplot as plt
import numpy as np
import math
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from tf2_msgs.msg import TFMessage
import time


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
                 
        self.frame_id = "map"

        self.map = OccupancyGridMap(xrange=map_size[0], yrange=map_size[1], grid_size=grid_size, 
                                    z_max=z_max, sensor_offset=sensor_offset, n_beams=n_beams,
                                     angle_range=angle_range, p_occ=p_occ, p_free=p_free)

        self.xrange = map_size[0]
        self.yrange = map_size[1]
        self.grid_size = grid_size

        self.scan = None
        self.pose = {}

        # plt.figure(figsize=(10, 10))
        # plt.pause(0.1)

        self.update_node = SubscriberNode("mapper", self)
        self.map_node = MapPublisher("map_publisher")
        rclpy.spin(self.update_node)
        rclpy.spin(self.map_node)

    def __del__(self):
        self.map_node.destroy_node()

    def callback_scan(self, msg):
        self.scan = msg.ranges

        if len(self.pose) != 0:
            # self.map.live_update_map(np.array([self.pose["x"], self.pose["y"], self.pose["theta"]]), np.array(self.scan))
            self.map.update(np.array([self.pose["x"], self.pose["y"], self.pose["theta"]]), np.array(self.scan))
            width = (abs(self.xrange[1]) + abs(self.xrange[0])) / self.grid_size
            height = (abs(self.yrange[1]) + abs(self.yrange[0])) / self.grid_size
            self.map_node.publish_callback(1.0 - 1./(1.+np.exp(self.map.odds_map)), self.frame_id, width, height, self.grid_size)
            # plt.clf()
            # plt.imshow(1.0 - 1./(1.+np.exp(self.map.odds_map)), 'Greys')
            # plt.pause(0.01)

    def callback_update(self, msg):
        self.pose["x"] = msg.pose.pose.position.x
        self.pose["y"] = msg.pose.pose.position.y
        quaternions = msg.pose.pose.orientation
        euler = euler_from_quaternion(quaternions)
        self.pose["theta"] = euler[2] * 180.0 / np.pi


class ScanNode(Node):
    def __init__(self, name, parent):
        super().__init__(name)
        self.subscription_scan = self.create_subscription(LaserScan, "/scan", self.callback_scan, 1)
        self.parent = parent

    def callback_scan(self, msg):
        self.parent.callback_scan(msg)


class SubscriberNode(Node):
    def __init__(self, name, parent):
        super().__init__(name)
        self.subscription_scan = self.create_subscription(LaserScan, "/scan", self.callback_scan, 1)
        self.subscription_pose = self.create_subscription(Odometry, "/ground_truth_pos", self.callback_odom, 1)
        self.parent = parent
        self.time = time.time()
        self.prev_time = time.time()
        self.scan_time_rate = 0.3  # in seconds

    def callback_scan(self, msg):
        # self.time = time.time()
        # if self.time - self.prev_time > self.scan_time_rate:
        self.parent.callback_scan(msg)
            # self.prev_time = time.time()

    def callback_odom(self, msg):
        self.parent.callback_update(msg)


class MapPublisher(Node):
    def __init__(self, name):
        super().__init__(name)
        self.publisher = self.create_publisher(OccupancyGrid, 'map', 5)

    def publish_callback(self, data, frame_id, width=400, height=400, grid_size=0.05):
        msg = OccupancyGrid()
        lower_bound = data > 0.45
        upper_bound = data < 0.55
        data[lower_bound & upper_bound] = -2
        data = data.flatten()
        data = np.round(data) * 100
        data = data.astype(np.int8)
        data = getattr(data, 'tolist', lambda:data)()
        msg.data = data

        msg.header.frame_id = str(frame_id)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = float(grid_size)
        msg.info.width = int(width)
        msg.info.height = int(height)
        msg.info.origin.position.x = -(width / 2) * grid_size
        msg.info.origin.position.y = -(height / 2) * grid_size

        self.publisher.publish(msg)


def main():
    rclpy.init()
    mapper = Mapper(map_size=[[-3, 3], [-3, 3]], grid_size=0.1, z_max=3.4, n_beams=360, angle_range=[0, 2*np.pi],
                    p_occ=0.95, p_free=0.2)
    

if __name__ == '__main__':
    try:
        main()
    except Exception:
        rclpy.shutdown()
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

import rclpy


class FixedFrameBroadcaster(Node):

   def __init__(self, name):
       super().__init__(name)
       self.tf_broadcaster = TransformBroadcaster(self)
       self.timer = self.create_timer(0.01, self.broadcast_timer_callback)

   def broadcast_timer_callback(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = -0.707#0.0
        t.transform.rotation.y = -0.707#0.0
        t.transform.rotation.z = 0.0#-0.707
        t.transform.rotation.w = 0.0#-0.707

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    tf = FixedFrameBroadcaster('map_broadcaster')
    rclpy.spin(tf)

if __name__ == '__main__':
    try:
        main()
    except Exception:
        rclpy.shutdown()
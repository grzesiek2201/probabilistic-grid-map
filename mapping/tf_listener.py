import rclpy
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformException


class Listener(Node):
    def __init__(self):
        super().__init__('tf_robot_listener')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.timer = self.create_timer(0.1, self.on_timer)

    def on_timer(self):
        try:
            now = rclpy.time.Time()
            t = self.buffer.lookup_transform('odometry',
                                             'odometry',
                                             now)
            print(t.transform)
        except TransformException as e:
            self.get_logger().info(e)                                            



if __name__ == '__main__':
    rclpy.init()
    frame_listener_node = Listener()
    try:
        rclpy.spin(frame_listener_node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
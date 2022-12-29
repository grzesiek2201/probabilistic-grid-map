import rclpy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Vector3, Quaternion
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Odometry
from rclpy.node import Node


class ArrowPublisher(Node):
    def __init__(self, name):
        super().__init__(name)
        self.arrow_publisher = self.create_publisher(Marker, '/robot_marker', 5)

    def publish(self, msg):
        self.arrow_publisher.publish(msg)

class PoseSubscriber(Node):
    def __init__(self, name, publisher):
        super().__init__(name)
        self.pose_subscriber = self.create_subscription(Odometry, '/ground_truth_pos', self.pose_callback, 1)
        self.publisher = publisher
        self.marker = None
        self.create_timer(0.1, self.pose_publish)

    def pose_callback(self, msg):
        self.marker = create_marker(0, msg.pose.pose, 1.0, 0.1, 0.1, r=255.0, g=5.0, b=0.0, a=1.0)

    def pose_publish(self):
        # print("Publishing marker..")
        self.publisher.publish(self.marker)    


def create_marker(action, pose, length, width, height, r, g, b, a):
    """
    :param action: 0 = add/modify, 1 = (deprecated), 2 = delete, New in Indigo 3 = deleteall
    :param pose: position of the tail of the arrow
    """
    m = Marker()
    m.header.frame_id = 'odom'
    m.action = action
    m.type = 0
    m.pose = pose
    m.color.r = r
    m.color.g = g
    m.color.b = b
    m.color.a = a
    m.scale.x = length
    m.scale.y = width
    m.scale.z = height

    return m


def main():
    rclpy.init()
    publisher_node = ArrowPublisher('robot_arrow_publisher_node')
    subscriber_node = PoseSubscriber('robot_pose_subscriber_node', publisher=publisher_node)
    rclpy.spin(subscriber_node)


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
        print("shutting down...")
        rclpy.shutdown()
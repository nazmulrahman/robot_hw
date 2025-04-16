import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import Point
from ublox_msgs.msg import NavSTATUS, NavPVT
from geographiclib.geodesic import Geodesic


class RTKFixPublisher(Node):
    def __init__(self):
        super().__init__('rtk_fix_publisher')

        # Publishers
        self.pub_rear = self.create_publisher(Point, '/rtk_rear_fix', 10)
        self.pub_front = self.create_publisher(Point, '/rtk_front_fix', 10)

        # Subscribers
        self.create_subscription(NavSTATUS, '/ublox_rear/navStatus', self.callback_rear_status, 10)
        self.create_subscription(NavSTATUS, '/ublox_front/navStatus', self.callback_front_status, 10)
        self.create_subscription(NavPVT, '/ublox_rear/navpvt', self.callback_rear_navpvt, 10)
        self.create_subscription(NavPVT, '/ublox_front/navpvt', self.callback_front_navpvt, 10)

        # Constants
        self.lat_org = 90.35303250
        self.long_org = 23.76030833

        # Status placeholders
        self.status_rear = 0
        self.status_front = 0

    def callback_rear_status(self, msg):
        self.status_rear = msg.fix_stat
        self.get_logger().info(f"status_rear: {self.status_rear}")

    def callback_front_status(self, msg):
        self.status_front = msg.fix_stat
        self.get_logger().info(f"status_front: {self.status_front}")

    def calc_goal(self, origin_lat, origin_long, goal_lat, goal_long):
        geod = Geodesic.WGS84
        g = geod.Inverse(origin_lat, origin_long, goal_lat, goal_long)
        hypotenuse = g['s12']
        azimuth = math.radians(g['azi1'])
        x = -math.cos(azimuth) * hypotenuse
        y = math.sin(azimuth) * hypotenuse
        return x, y

    def callback_rear_navpvt(self, msg):
        rtk_msg = Point()
        rtk_msg.x, rtk_msg.y = self.calc_goal(self.lat_org, self.long_org, msg.lat * 1e-7, msg.lon * 1e-7)
        rtk_msg.z = 0.0
        self.pub_rear.publish(rtk_msg)

    def callback_front_navpvt(self, msg):
        rtk_msg = Point()
        rtk_msg.x, rtk_msg.y = self.calc_goal(self.lat_org, self.long_org, msg.lat * 1e-7, msg.lon * 1e-7)
        rtk_msg.z = 0.0
        self.pub_front.publish(rtk_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RTKFixPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

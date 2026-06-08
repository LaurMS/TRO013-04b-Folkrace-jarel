import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class FolkraceJuht(Node):

    MAX_KIIRUS = 0.35
    MIN_KIIRUS = 0.1
    MAX_POORDEIKIIRUS = 1.0

    OHUTU_KAUGUS = 1.0
    KRIITILINE_KAUGUS = 0.3

    RINGIDE_ARV = 3

    def __init__(self):
        super().__init__('folkrace_juht')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        self.praegune_x = 0.0
        self.praegune_y = 0.0

        self.prev_x = None
        self.prev_y = None

        self.kogu_labitud = 0.0
        self.labitud_ringid = 0

        self.RINGI_PIKKUS = 5.0

        self.get_logger().info("Folkrace juht käivitatud!")

    # ---------------- ODOMEETRIA ----------------
    def odom_callback(self, msg):
        self.praegune_x = msg.pose.pose.position.x
        self.praegune_y = msg.pose.pose.position.y

        if self.prev_x is not None:

            dx = self.praegune_x - self.prev_x
            dy = self.praegune_y - self.prev_y
            dist = math.sqrt(dx * dx + dy * dy)

            self.kogu_labitud += dist

            if self.kogu_labitud >= self.RINGI_PIKKUS * (self.labitud_ringid + 1):
                self.labitud_ringid += 1
                self.get_logger().info(
                    f"Ring {self.labitud_ringid}/{self.RINGIDE_ARV} läbitud!"
                )

        self.prev_x = self.praegune_x
        self.prev_y = self.praegune_y

    # ---------------- LIDAR ----------------
    def scan_callback(self, msg):

        # STOP kui ringid täis
        if self.labitud_ringid >= self.RINGIDE_ARV:
            self.cmd_pub.publish(Twist())
            return

        ranges = msg.ranges
        n = len(ranges)
        mid = n // 2

        def safe_min(slice_):
            vals = [r for r in slice_ if 0.12 < r < 8.0]
            return min(vals) if vals else 8.0

        front = safe_min(ranges[mid - 15: mid + 15])
        left = safe_min(ranges[mid + 70: mid + 110])
        right = safe_min(ranges[mid - 110: mid - 70])

        cmd = Twist()

        # ---------------- JUHTIMINE ----------------

        # 1. OHUTU KAUgel → max kiirus
        if front > self.OHUTU_KAUGUS:
            cmd.linear.x = self.MAX_KIIRUS
            cmd.angular.z = (left - right) * 0.3

        # 2. keskmine tsoon → aeglustus
        elif front > self.KRIITILINE_KAUGUS:

            ratio = (front - self.KRIITILINE_KAUGUS) / (
                self.OHUTU_KAUGUS - self.KRIITILINE_KAUGUS
            )

            cmd.linear.x = self.MIN_KIIRUS + ratio * (
                self.MAX_KIIRUS - self.MIN_KIIRUS
            )

            cmd.angular.z = (left - right) * 0.5

        # 3. liiga lähedal → stop + pööre
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = (
                self.MAX_POORDEIKIIRUS
                if left > right
                else -self.MAX_POORDEIKIIRUS
            )

        # piirame pöördekiiruse
        cmd.angular.z = max(
            -self.MAX_POORDEIKIIRUS,
            min(self.MAX_POORDEIKIIRUS, cmd.angular.z)
        )

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = FolkraceJuht()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

"""Moodul 04b: Iseseisev folkrace sõitmine (täiustatud).

Ülesanne:
  Ehita moodul 03 folkrace_driver'ile peale täiustatud versioon,
  mis kasutab raja_anduri sektorite analüüsi ja odomeetria andmeid.

Nõuded:
  1. Robot sõidab folkrace raja 3 TÄISRINGI
  2. Robot reguleerib kiirust vastavalt kaugusele (kaugel = kiire, lähedal = aeglane)
  3. Robot kasutab odomeetriat ringi lugemiseks
  4. Robot suudab üle silla minna
  5. Robot ei põrka seintesse

Täiustused võrreldes moodul 03 lihtsama versiooniga:
  - Kiiruse reguleerimine (mitte konstantne kiirus)
  - Sujuv pööramine (mitte järsk vasak/parem)
  - Ringi lugemine odomeetria abil
  - Silla tuvastamine ja ületamine

Käivita:
  Terminal 1: ros2 launch yahboom_webots webots.launch.py
  Terminal 2: ros2 run folkrace_juht folkrace_juht
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class FolkraceJuht(Node):

    # Häälestusparameetrid
    MAX_KIIRUS = 0.35       # m/s
    MIN_KIIRUS = 0.1        # m/s
    MAX_POORDEIKIIRUS = 1.0  # rad/s

    OHUTU_KAUGUS = 1.0      # m — alla selle aeglusta
    KRIITILINE_KAUGUS = 0.3  # m — alla selle peatu ja pöördu

    RINGIDE_ARV = 3          # mitu ringi sõita

    def __init__(self):
        super().__init__('folkrace_juht')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # Odomeetria andmed ringi lugemiseks
        self.algus_x = None
        self.algus_y = None
        self.praegune_x = 0.0
        self.praegune_y = 0.0
        self.labitud_ringid = 0
        self.kogu_labitud = 0.0
        self.prev_x = None
        self.prev_y = None

        # TODO: arvuta RINGI_PIKKUS (folkrace raja ligikaudne ümbermõõt meetrites)
        # Vihje: folkrace rada on umbes 4-6m pikk ring
        self.RINGI_PIKKUS = 5.0  # meetrit (häälesta vastavalt rajale)

        self.get_logger().info('Folkrace juht käivitatud! Sõidan 3 ringi.')

    def odom_callback(self, msg):
        self.praegune_x = msg.pose.pose.position.x
        self.praegune_y = msg.pose.pose.position.y

        # TODO: loe läbitud vahemaa kokku
        #
        # Vihje:
        #   if self.prev_x is not None:
        #       dx = self.praegune_x - self.prev_x
        #       dy = self.praegune_y - self.prev_y
        #       self.kogu_labitud += math.sqrt(dx*dx + dy*dy)
        #
        #       # Kontrolli kas ring on täis
        #       if self.kogu_labitud >= self.RINGI_PIKKUS * (self.labitud_ringid + 1):
        #           self.labitud_ringid += 1
        #           self.get_logger().info(f'Ring {self.labitud_ringid}/{self.RINGIDE_ARV} läbitud!')
        #
        #   self.prev_x = self.praegune_x
        #   self.prev_y = self.praegune_y
        pass

    def scan_callback(self, msg):
        # TODO: kontrolli kas kõik ringid on läbitud
        # if self.labitud_ringid >= self.RINGIDE_ARV:
        #     cmd = Twist()  # peatu
        #     self.cmd_pub.publish(cmd)
        #     return

        ranges = msg.ranges
        n = len(ranges)

        # TODO: loe lidari sektorid (kasuta moodul 04a raja_anduri teadmisi)
        #
        # Vihje:
        #   mid = n // 2  # otse ette
        #   front = min((r for r in ranges[mid-15:mid+15] if 0.12 < r < 8.0), default=8.0)
        #   left  = min((r for r in ranges[mid+70:mid+110] if 0.12 < r < 8.0), default=8.0)
        #   right = min((r for r in ranges[mid-110:mid-70] if 0.12 < r < 8.0), default=8.0)
        front = 8.0  # TODO
        left = 8.0   # TODO
        right = 8.0  # TODO

        cmd = Twist()

        # TODO: implementeeri täiustatud juhtimisloogika
        #
        # Kiiruse reguleerimine:
        #   - front > OHUTU_KAUGUS → MAX_KIIRUS
        #   - KRIITILINE < front < OHUTU → lineaarselt skaleeri MIN_KIIRUS..MAX_KIIRUS
        #   - front < KRIITILINE → peatu (0.0) ja pöördu
        #
        # Sujuv pööramine:
        #   - Arvuta suunaparandus: (left - right) * koefitsient
        #   - Piirangu sees: |angular.z| <= MAX_POORDEIKIIRUS
        #
        # Vihje:
        #   if front > self.OHUTU_KAUGUS:
        #       cmd.linear.x = self.MAX_KIIRUS
        #       cmd.angular.z = (left - right) * 0.3  # sujuv keskel hoidmine
        #   elif front > self.KRIITILINE_KAUGUS:
        #       ratio = (front - self.KRIITILINE_KAUGUS) / (self.OHUTU_KAUGUS - self.KRIITILINE_KAUGUS)
        #       cmd.linear.x = self.MIN_KIIRUS + ratio * (self.MAX_KIIRUS - self.MIN_KIIRUS)
        #       cmd.angular.z = (left - right) * 0.5
        #   else:
        #       cmd.linear.x = 0.0
        #       cmd.angular.z = self.MAX_POORDEIKIIRUS if left > right else -self.MAX_POORDEIKIIRUS

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = FolkraceJuht()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

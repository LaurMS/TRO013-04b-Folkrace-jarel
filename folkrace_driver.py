import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math, time, signal, statistics

DRIVING_SPEED  = 0.18 * 2
TURN_SPEED = 4.0 / 5.0

DANGER_LIMIT_FRONT = 1.10 / 2
DANGER_LIMIT_SIDE = 3.0 / 10.0

REVERSE_THRESHOLD   = 1.0 / 4.0
REVERSE_SPEED = -(3.0 / 20.0)
CREEP_SPEED   = 6.0 / 50.0
SECTORS_COUNT      = 3 * 6

REVERSE_PERSISTENCE = 2 + 2
CLOSE_PERSISTENCE    = 1 + 2

ROBOT_HALF_WIDTH = 15.0 / 100.0
DISPARITY_THRESHOLD   = 0.15 * 2

WHEEL_RADIUS = 33.0 / 1000.0
HALF_TRACK   = 83.0 / 1000.0

MAX_WHEEL_ANGULAR_SPEED = 22.0 / 2.0


class FolkraceDriver(Node):
    def __init__(self):
        super().__init__('folkrace_driver')
        self.create_subscription(LaserScan, '/scan', self._scan_callback, 10)
        self._vel_pub  = self.create_publisher(Twist,  '/cmd_vel',       10)
        self._state_pub = self.create_publisher(String, '/folkrace_state', 10)

        self._previous_angle   = 0.0
        self._log_counter    = 0
        self._reverse_streak = 0
        self._close_streak    = 0

        self.get_logger().info('FolkraceDriver started')

   
    def _publish_state(self, text: str):
        self._state_pub.publish(String(data=text))

    def _get_sector_min(self, ranges, start_deg: float, end_deg: float) -> float:
        n = len(ranges)
        if n == 0:
            return float('inf')

        def deg_to_index(deg: float) -> int:
            return int((deg + 180.0) / 360.0 * n) % n

        a = deg_to_index(start_deg)
        b = deg_to_index(end_deg)

        if a <= b:
            sector = ranges[a:b + 1]
        else:
            sector = list(ranges[a:]) + list(ranges[:b + 1])

        valid = [r for r in sector
                    if not math.isnan(r) and not math.isinf(r)
                    and 0.12 <= r <= 8.0]
        return statistics.median(valid) if valid else float('inf')

    def _inflate_disparities(self, distances):
        n = len(distances)
        sector_rad = math.radians(180.0 / n)
        inflated = list(distances)

        for i in range(n - 1):
            d_left = distances[i]
            d_right = distances[i + 1]
            difference = abs(d_left - d_right)
            
            if difference < DISPARITY_THRESHOLD:
                continue

            if d_left < d_right:
                d = d_left
                if d < 0.15:
                    continue
                half_angle = math.asin(min(1.0, ROBOT_HALF_WIDTH / d))
                inflation = int(half_angle / sector_rad) + 1
                for k in range(i + 1, min(i + 1 + inflation, n)):
                    if inflated[k] > d:
                        inflated[k] = d
            else:
                d = d_right
                if d < 0.15:
                    continue
                half_angle = math.asin(min(1.0, ROBOT_HALF_WIDTH / d))
                inflation = int(half_angle / sector_rad) + 1
                for k in range(max(0, i + 1 - inflation), i + 1):
                    if inflated[k] > d:
                        inflated[k] = d

        return inflated

    def _kinematic_limit(self, speed: float, turn: float):
        output = (abs(speed) + abs(turn) * HALF_TRACK) / WHEEL_RADIUS
        max_allowed = MAX_WHEEL_ANGULAR_SPEED
        if output > max_allowed:
            scale = max_allowed / output
            speed *= scale
            turn *= scale
        return speed, turn

    def _scan_callback(self, msg: LaserScan):
        ranges = list(msg.ranges)

        front  = self._get_sector_min(ranges, -20,   20)
        left = self._get_sector_min(ranges,  60,  120)
        right = self._get_sector_min(ranges, -120, -60)

        speed, turn = self._calculate_speed(ranges, front, left, right)

        speed, turn = self._kinematic_limit(speed, turn)

        cmd = Twist()
        cmd.linear.x  = float(max(-DRIVING_SPEED, min(DRIVING_SPEED, speed)))
        cmd.angular.z = float(max(-2.0, min(2.0, turn)))
        self._vel_pub.publish(cmd)

        self._log_counter += 1
        if self._log_counter % 32 == 0:
            self.get_logger().info(
                f'front={front:.2f}m  left={left:.2f}m  right={right:.2f}m  '
                f'→ v={speed:.2f}m/s  ω={turn:.2f}rad/s'
            )

    
    def _calculate_speed(self, ranges, front, left, right):
        n = len(ranges)
        if n == 0:
            return DRIVING_SPEED, 0.0

        if front < REVERSE_THRESHOLD:
            self._reverse_streak += 1
        else:
            self._reverse_streak = 0

        start = n // 4
        end  = 3 * n // 4
        front_ranges = ranges[start:end]
        m = len(front_ranges)

        sector_size = m // SECTORS_COUNT
        if sector_size == 0:
            return DRIVING_SPEED, 0.0

        distances = []
        for i in range(SECTORS_COUNT):
            a = i * sector_size
            b = a + sector_size
            sector = front_ranges[a:b]
            valid = [r for r in sector
                        if not (math.isinf(r) or math.isnan(r)) and r > 0.12]
            distance = statistics.median(valid) if valid else 8.0
            distances.append(distance)

        distances = self._inflate_disparities(distances)

        best_distance = 0.0
        best_idx = SECTORS_COUNT // 2

        for i in range(SECTORS_COUNT):
            neighbor_left = distances[i - 1] if i > 0 else distances[i]
            neighbor_right = distances[i + 1] if i < SECTORS_COUNT - 1 else distances[i]
            if neighbor_left < 0.25 and neighbor_right < 0.25:
                continue
            if distances[i] > best_distance:
                best_distance = distances[i]
                best_idx = i

        if best_distance == 0.0:
            best_idx = max(range(SECTORS_COUNT), key=lambda i: distances[i])
            best_distance = distances[best_idx]

        if best_distance < 0.4:
            self._close_streak += 1
        else:
            self._close_streak = 0

        best_angle = ((best_idx / SECTORS_COUNT) - 0.5) * 180.0
        best_angle = -best_angle

        previous_idx = int(((-self._previous_angle / 180.0) + 0.5) * SECTORS_COUNT)
        previous_idx = max(0, min(SECTORS_COUNT - 1, previous_idx))
        previous_distance = distances[previous_idx]

        if best_distance < previous_distance * 1.2 and previous_distance > 0.5:
            best_angle = self._previous_angle

        self._previous_angle = best_angle

        if self._reverse_streak >= REVERSE_PERSISTENCE:
            direction = 1.0 if best_angle >= 0 else -1.0
            self._publish_state(f'REVERSING (persistent) → {"L" if direction > 0 else "R"}')
            return REVERSE_SPEED, direction * TURN_SPEED

        angle_rad = math.radians(best_angle)

        if self._close_streak >= CLOSE_PERSISTENCE and best_distance < 0.4:
            direction = 1.0 if best_angle >= 0 else -1.0
            self._publish_state(f'STUCK → turning {"L" if direction > 0 else "R"}')
            speed = 0.0
            turn = direction * TURN_SPEED
        elif best_distance < 0.4:
            self._publish_state(f'CREEPING (temporary) → {best_angle:+.0f}°')
            speed = CREEP_SPEED
            turn = (1.0 if best_angle >= 0 else -1.0) * TURN_SPEED * 0.5
        elif abs(best_angle) > 40:
            speed = DRIVING_SPEED * 0.2
            turn = TURN_SPEED * 0.8 * (1.0 if best_angle > 0 else -1.0)
        elif abs(best_angle) > 15:
            speed = DRIVING_SPEED * 0.5
            turn = angle_rad * 2.0
            turn = max(-TURN_SPEED, min(TURN_SPEED, turn))
        else:
            speed = DRIVING_SPEED
            turn = angle_rad * 1.0
            turn = max(-TURN_SPEED * 0.3, min(TURN_SPEED * 0.3, turn))

        if left < DANGER_LIMIT_SIDE:
            correction = (DANGER_LIMIT_SIDE - left) / DANGER_LIMIT_SIDE
            turn -= correction * TURN_SPEED * 0.5
        if right < DANGER_LIMIT_SIDE:
            correction = (DANGER_LIMIT_SIDE - right) / DANGER_LIMIT_SIDE
            turn += correction * TURN_SPEED * 0.5

        turn = max(-TURN_SPEED, min(TURN_SPEED, turn))

        self._publish_state(
            f'BEST={best_angle:+.0f}° d={best_distance:.1f}m '
            f'v={speed:.2f} ω={turn:+.2f}'
        )
        return speed, turn


def main():
    rclpy.init()
    node = FolkraceDriver()

    running = True
    def _stop(sig, frame):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, _stop)

    while running:
        rclpy.spin_once(node, timeout_sec=0.1)

    stop = Twist()
    for _ in range(10):
        node._vel_pub.publish(stop)
        time.sleep(0.05)
    node.destroy_node()
    rclpy.shutdown()
    print('\nStopped.')


if __name__ == '__main__':
    main()

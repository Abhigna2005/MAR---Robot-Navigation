import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

SAFE_DIST   = 0.45   # metres — stop/turn if anything closer
LINEAR_SPD  = 0.22   # m/s forward speed
ANGULAR_SPD = 1.0    # rad/s turn speed

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.get_logger().info('Obstacle Avoider started — listening to /scan')

    def scan_cb(self, msg):
        ranges = msg.ranges
        n = len(ranges)

        # Divide scan into front (±30°), left, right sectors
        def sector_min(start_deg, end_deg):
            idxs = range(int(start_deg * n / 360), int(end_deg * n / 360))
            vals = [ranges[i] for i in idxs if msg.range_min < ranges[i] < msg.range_max]
            return min(vals) if vals else msg.range_max

        front = sector_min(330, 360)  # last 30°
        front2 = sector_min(0, 30)   # first 30°
        min_front = min(front, front2)
        left  = sector_min(30, 90)
        right = sector_min(270, 330)

        cmd = Twist()

        if min_front < SAFE_DIST:
            # Obstacle ahead — turn away from closer side
            cmd.linear.x  = 0.0
            cmd.angular.z = ANGULAR_SPD if right < left else -ANGULAR_SPD
            self.get_logger().info(f'OBSTACLE ahead={min_front:.2f}m — turning')
        else:
            # Path clear — go forward
            cmd.linear.x  = LINEAR_SPD
            cmd.angular.z = 0.0

        self.pub.publish(cmd)

def main():
    rclpy.init()
    node = ObstacleAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.pub.publish(Twist())
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

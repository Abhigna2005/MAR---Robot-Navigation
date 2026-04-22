import rclpy, math, heapq
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

GRID_SIZE = 40; CELL_SIZE = 0.5; ORIGIN_X = -10.0; ORIGIN_Y = -10.0

def make_grid():
    grid = [[0]*GRID_SIZE for _ in range(GRID_SIZE)]
    def mark(x0, x1, y0, y1):
        for gy in range(max(0, int((y0-ORIGIN_Y)/CELL_SIZE)), min(GRID_SIZE, int((y1-ORIGIN_Y)/CELL_SIZE)+1)):
            for gx in range(max(0, int((x0-ORIGIN_X)/CELL_SIZE)), min(GRID_SIZE, int((x1-ORIGIN_X)/CELL_SIZE)+1)):
                grid[gy][gx] = 1
    for i in range(GRID_SIZE):
        grid[0][i]=grid[-1][i]=grid[i][0]=grid[i][-1]=1
    # ShelfF
    mark(-7.0, -4.5, -3.0, 1.5)
    # ShelfE/D group
    mark(2.0, 7.0, -0.8, 2.0)
    mark(2.0, 7.0, -2.2, -0.6)
    mark(2.0, 7.0, -4.0, -2.0)
    mark(2.0, 7.0, -5.8, -3.8)
    mark(2.0, 7.0, -7.8, -5.6)
    mark(2.0, 7.0, -9.8, -7.5)
    return grid

GRID = make_grid()

def w2g(wx, wy): return (int((wx-ORIGIN_X)/CELL_SIZE), int((wy-ORIGIN_Y)/CELL_SIZE))
def g2w(gx, gy): return (gx*CELL_SIZE+ORIGIN_X+CELL_SIZE/2, gy*CELL_SIZE+ORIGIN_Y+CELL_SIZE/2)
def heur(a, b): return math.hypot(a[0]-b[0], a[1]-b[1])

def astar(s, g):
    open_l = [(0.0, s)]; came = {}; gs = {s: 0.0}
    while open_l:
        _, cur = heapq.heappop(open_l)
        if cur == g:
            path = []
            while cur in came:
                path.append(cur)
                cur = came[cur]
            path.append(s)
            path.reverse()
            return path
        for dx,dy in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
            nx,ny = cur[0]+dx, cur[1]+dy
            if 0<=nx<GRID_SIZE and 0<=ny<GRID_SIZE and GRID[ny][nx]==0:
                ng = gs[cur]+(1.4 if dx and dy else 1.0)
                nb = (nx, ny)
                if nb not in gs or ng < gs[nb]:
                    gs[nb] = ng
                    heapq.heappush(open_l, (ng+heur(nb,g), nb))
                    came[nb] = cur
    return []

SAFE_DIST = 0.40

class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        # ── Change these coordinates to test different paths ──
        start = (3.0, 3.0)
        goal  = (2.0, -2.0)

        sg, gg = w2g(*start), w2g(*goal)
        path = astar(sg, gg)
        self.wp = [g2w(*p) for p in path] if path else []
        self.idx = 0
        self.x, self.y, self.yaw = start[0], start[1], 0.0
        self.obstacle = False
        self.left_clear = True
        self.right_clear = True

        self.get_logger().info(f'Path found: {len(self.wp)} waypoints')
        self.create_timer(0.1, self.step)

    def scan_cb(self, msg):
        n = len(msg.ranges)
        def sec(a, b):
            vals = [msg.ranges[i] for i in range(int(a*n/360), int(b*n/360))
                    if msg.range_min < msg.ranges[i] < msg.range_max]
            return min(vals) if vals else msg.range_max
        front = min(sec(330, 360), sec(0, 30))
        self.obstacle = front < SAFE_DIST
        self.left_clear  = sec(30, 90)  > SAFE_DIST
        self.right_clear = sec(270, 330) > SAFE_DIST

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def step(self):
        if self.idx >= len(self.wp):
            self.pub.publish(Twist())
            self.get_logger().info('Goal reached!')
            return
        cmd = Twist()
        if self.obstacle:
            cmd.linear.x  = 0.0
            cmd.angular.z = 0.8 if self.left_clear else -0.8
            self.get_logger().warn('Obstacle! Avoiding...')
        else:
            tx, ty = self.wp[self.idx]
            dx, dy = tx - self.x, ty - self.y
            dist = math.hypot(dx, dy)
            if dist < 0.3:
                self.idx += 1
                return
            target_yaw = math.atan2(dy, dx)
            err = target_yaw - self.yaw
            while err >  math.pi: err -= 2*math.pi
            while err < -math.pi: err += 2*math.pi
            if abs(err) > 0.15:
                cmd.angular.z = max(-1.0, min(1.0, err))
            else:
                cmd.linear.x = min(0.22, dist)
        self.pub.publish(cmd)

def main():
    rclpy.init()
    node = Navigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.pub.publish(Twist())
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

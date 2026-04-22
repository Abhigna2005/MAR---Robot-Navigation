import rclpy
import math
import heapq
import time

from rclpy.node             import Node
from geometry_msgs.msg      import Twist, PoseStamped
from nav_msgs.msg           import Odometry, Path
from sensor_msgs.msg        import LaserScan
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Time as RosTime

# ---------------------------------------------------------------------------
# Grid  —  20 m x 16 m parking lot,  cell = 0.25 m  ->  80 x 64 cells
# ---------------------------------------------------------------------------
CELL_SIZE = 0.25
GRID_W    = 80
GRID_H    = 64
ORIGIN_X  = -10.0
ORIGIN_Y  = -8.0
INFLATE   = 2        # obstacle inflation in cells (= 0.5 m)


def make_grid():
    grid = [[0] * GRID_W for _ in range(GRID_H)]

    def mark(wx_min, wx_max, wy_min, wy_max, pad=INFLATE):
        gx0 = max(0,        int((wx_min - ORIGIN_X) / CELL_SIZE) - pad)
        gx1 = min(GRID_W-1, int((wx_max - ORIGIN_X) / CELL_SIZE) + pad)
        gy0 = max(0,        int((wy_min - ORIGIN_Y) / CELL_SIZE) - pad)
        gy1 = min(GRID_H-1, int((wy_max - ORIGIN_Y) / CELL_SIZE) + pad)
        for gy in range(gy0, gy1 + 1):
            for gx in range(gx0, gx1 + 1):
                grid[gy][gx] = 1

    # Boundary walls
    mark(-10.0,  10.0,   7.8,   8.1)
    mark(-10.0,  10.0,  -8.1,  -7.8)
    mark(-10.1,  -9.8,  -8.0,   8.0)
    mark(  9.8,  10.1,   2.0,   8.0)
    mark(  9.8,  10.1,  -8.0,  -2.0)

    # Centre dividers (thin)
    mark(-7.0,   2.0,  -0.075,  0.075, pad=0)
    mark( 3.0,   9.0,  -0.075,  0.075, pad=0)

    # North row parked cars (y ~ +4)
    for cx in [-8.0, -5.5, -3.0, -0.5, 4.0, 6.5]:
        mark(cx - 0.9, cx + 0.9,  2.2,  5.8)

    # South row parked cars (gap at x=+1.5 is the goal slot)
    for cx in [-8.0, -5.5, -3.0, 4.0, 6.5]:
        mark(cx - 0.9, cx + 0.9, -5.8, -2.2)

    # Speed bumps
    mark(-6.9, -5.1, -0.2,  0.2)
    mark( 1.1,  2.9, -0.2,  0.2)

    # Bollards at entrance
    mark(8.85, 9.15,  1.85,  2.15)
    mark(8.85, 9.15, -2.15, -1.85)

    return grid


BASE_GRID = make_grid()


def world_to_grid(wx, wy):
    gx = int((wx - ORIGIN_X) / CELL_SIZE)
    gy = int((wy - ORIGIN_Y) / CELL_SIZE)
    return (max(0, min(GRID_W - 1, gx)),
            max(0, min(GRID_H - 1, gy)))


def grid_to_world(gx, gy):
    wx = gx * CELL_SIZE + ORIGIN_X + CELL_SIZE / 2
    wy = gy * CELL_SIZE + ORIGIN_Y + CELL_SIZE / 2
    return (wx, wy)


def heuristic(a, b):
    # Euclidean distance heuristic (project plan requirement)
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def astar(start, goal, grid):
    """A* search. Returns list of grid-cell tuples or []."""
    open_list = []
    heapq.heappush(open_list, (0.0, start))
    came_from = {}
    g_score   = {start: 0.0}

    while open_list:
        _, current = heapq.heappop(open_list)
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1),
                       (-1, -1), (-1, 1), (1, -1), (1, 1)]:
            nx, ny = current[0] + dx, current[1] + dy
            if 0 <= nx < GRID_W and 0 <= ny < GRID_H and grid[ny][nx] == 0:
                cost  = 1.4 if (dx and dy) else 1.0
                new_g = g_score[current] + cost
                nb    = (nx, ny)
                if nb not in g_score or new_g < g_score[nb]:
                    g_score[nb] = new_g
                    heapq.heappush(open_list, (new_g + heuristic(nb, goal), nb))
                    came_from[nb] = current
    return []


def smooth_path(waypoints, angle_tol=0.10):
    """Remove collinear intermediate waypoints."""
    if len(waypoints) < 3:
        return waypoints
    smoothed = [waypoints[0]]
    for i in range(1, len(waypoints) - 1):
        px, py = smoothed[-1]
        cx, cy = waypoints[i]
        nx, ny = waypoints[i + 1]
        v1   = math.atan2(cy - py, cx - px)
        v2   = math.atan2(ny - cy, nx - cx)
        diff = abs(v2 - v1)
        if diff > math.pi:
            diff = 2 * math.pi - diff
        if diff > angle_tol:
            smoothed.append(waypoints[i])
    smoothed.append(waypoints[-1])
    return smoothed


def now_stamp():
    t = time.time()
    sec = int(t)
    ns  = int((t - sec) * 1e9)
    s = RosTime()
    s.sec     = sec
    s.nanosec = ns
    return s


# ---------------------------------------------------------------------------
# Speed constraint constants  (Phase 5)
# ---------------------------------------------------------------------------
MAX_LINEAR_VEL  = 0.7    # m/s  hard cap on forward speed
MAX_ANGULAR_VEL = 1.2    # rad/s hard cap on rotation
TURN_SPEED_MULT = 0.35   # fraction of MAX_LINEAR_VEL allowed while turning
RAMP_DIST       = 0.8    # m  distance over which to ramp speed up/down

TURN_THRESH     = 0.15   # rad  angle error triggering in-place rotation
WP_REACH        = 0.28   # m   waypoint capture radius

LIDAR_STOP      = 0.35   # m  emergency stop distance
LIDAR_SLOW      = 0.70   # m  begin slowing distance
REPLAN_TIMEOUT  = 2.0    # s  seconds stuck before replanning


# ---------------------------------------------------------------------------
# Navigator node
# ---------------------------------------------------------------------------
class Navigator(Node):

    def __init__(self):
        super().__init__('astar_navigator')

        # Publishers
        self.cmd_pub  = self.create_publisher(Twist,  '/cmd_vel',      10)
        self.path_pub = self.create_publisher(Path,   '/planned_path', 10)  # Phase 7
        self.traj_pub = self.create_publisher(Path,   '/trajectory',   10)  # Phase 7
        self.goal_pub = self.create_publisher(Marker, '/goal_marker',  10)  # Phase 7

        # Subscribers
        self.create_subscription(Odometry,  '/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        # Mission
        self.start_world = (0.0,  0.5)
        self.goal_world  = (1.5, -4.0)  # empty south-row parking slot

        # Odometry state
        self.x          = self.start_world[0]
        self.y          = self.start_world[1]
        self.yaw        = 0.0
        self.odom_ready = False

        # LiDAR state
        self.front_dist = float('inf')
        self.left_dist  = float('inf')
        self.right_dist = float('inf')

        # Navigation state
        self.wp          = []
        self.idx         = 0
        self.done        = False
        self.stuck_since = None
        self.last_log_t  = 0.0

        # Evaluation metrics
        self.start_time       = None
        self.dist_travelled   = 0.0
        self.prev_x           = self.x
        self.prev_y           = self.y
        self.max_speed_seen   = 0.0
        self.speed_violations = 0
        self.trajectory_msg   = Path()
        self.trajectory_msg.header.frame_id = 'odom'

        self._plan(self.start_world, self.goal_world)
        self._publish_goal_marker()
        self.create_timer(0.1, self.step)

    # -----------------------------------------------------------------------
    # Planning
    # -----------------------------------------------------------------------
    def _plan(self, from_world, to_world):
        sg = world_to_grid(*from_world)
        gg = world_to_grid(*to_world)

        if BASE_GRID[sg[1]][sg[0]] == 1:
            self.get_logger().error(f'Start cell {sg} is blocked!')
            self.wp = []
            return
        if BASE_GRID[gg[1]][gg[0]] == 1:
            self.get_logger().error(f'Goal cell {gg} is blocked!')
            self.wp = []
            return

        raw_cells = astar(sg, gg, BASE_GRID)
        if not raw_cells:
            self.get_logger().error('A* found no path!')
            self.wp = []
            return

        raw_wp  = [grid_to_world(gx, gy) for gx, gy in raw_cells]
        self.wp = smooth_path(raw_wp)
        self.idx = 0

        total_dist = sum(
            math.hypot(self.wp[i+1][0] - self.wp[i][0],
                       self.wp[i+1][1] - self.wp[i][1])
            for i in range(len(self.wp) - 1)
        )
        self.get_logger().info(
            f'A* path: {len(raw_wp)} raw -> {len(self.wp)} smoothed waypoints, '
            f'{total_dist:.2f} m planned'
        )
        self._print_path_table(total_dist)
        self._publish_planned_path()

    def _print_path_table(self, total_dist):
        print('\n' + '=' * 56)
        print('  A* PATH FOUND')
        print('=' * 56)
        print(f'  Waypoints  : {len(self.wp)}')
        print(f'  Distance   : {total_dist:.2f} m')
        print(f'  Est. time  : {total_dist / MAX_LINEAR_VEL:.1f} s')
        print(f'  Max speed  : {MAX_LINEAR_VEL:.2f} m/s')
        print('-' * 56)
        print(f'  {"#":>4}   {"X (m)":>8}   {"Y (m)":>8}')
        print('-' * 56)
        step = max(1, len(self.wp) // 15)
        for i, (wx, wy) in enumerate(self.wp):
            if i % step == 0 or i == len(self.wp) - 1:
                tag = ' <- START' if i == 0 else (' <- GOAL' if i == len(self.wp)-1 else '')
                print(f'  {i:>4}   {wx:>8.3f}   {wy:>8.3f}{tag}')
        print('=' * 56 + '\n')

    # -----------------------------------------------------------------------
    # Phase 7 — RViz publishers
    # -----------------------------------------------------------------------
    def _publish_planned_path(self):
        msg = Path()
        msg.header.frame_id = 'odom'
        msg.header.stamp    = now_stamp()
        for wx, wy in self.wp:
            ps = PoseStamped()
            ps.header             = msg.header
            ps.pose.position.x    = wx
            ps.pose.position.y    = wy
            ps.pose.position.z    = 0.0
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.path_pub.publish(msg)

    def _publish_goal_marker(self):
        m = Marker()
        m.header.frame_id    = 'odom'
        m.header.stamp       = now_stamp()
        m.ns                 = 'goal'
        m.id                 = 0
        m.type               = Marker.SPHERE
        m.action             = Marker.ADD
        m.pose.position.x    = self.goal_world[0]
        m.pose.position.y    = self.goal_world[1]
        m.pose.position.z    = 0.1
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.4
        m.color.r = 0.0
        m.color.g = 0.9
        m.color.b = 0.2
        m.color.a = 0.9
        self.goal_pub.publish(m)

    def _append_trajectory(self):
        ps = PoseStamped()
        ps.header.frame_id    = 'odom'
        ps.header.stamp       = now_stamp()
        ps.pose.position.x    = self.x
        ps.pose.position.y    = self.y
        ps.pose.position.z    = 0.0
        ps.pose.orientation.w = 1.0
        self.trajectory_msg.poses.append(ps)
        self.trajectory_msg.header.stamp = ps.header.stamp
        self.traj_pub.publish(self.trajectory_msg)

    # -----------------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------------
    def odom_cb(self, msg):
        new_x = msg.pose.pose.position.x
        new_y = msg.pose.pose.position.y

        step = math.hypot(new_x - self.prev_x, new_y - self.prev_y)
        if step < 0.5:
            self.dist_travelled += step
        self.prev_x = new_x
        self.prev_y = new_y

        self.x = new_x
        self.y = new_y
        q      = msg.pose.pose.orientation
        siny   = 2.0 * (q.w * q.z + q.x * q.y)
        cosy   = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)

        if not self.odom_ready:
            self.odom_ready = True
            self.start_time = time.time()

        if len(self.trajectory_msg.poses) == 0 or step > 0.05:
            self._append_trajectory()

    def scan_cb(self, msg):
        n = len(msg.ranges)

        def sec_min(start_deg, end_deg):
            vals = [
                msg.ranges[i]
                for i in range(int(start_deg * n / 360),
                               int(end_deg   * n / 360))
                if msg.range_min < msg.ranges[i] < msg.range_max
            ]
            return min(vals) if vals else msg.range_max

        self.front_dist = min(sec_min(330, 360), sec_min(0, 30))
        self.left_dist  = sec_min(30,  90)
        self.right_dist = sec_min(270, 330)

    # -----------------------------------------------------------------------
    # Control loop (10 Hz)
    # -----------------------------------------------------------------------
    def step(self):
        if self.done:
            return
        if not self.odom_ready:
            self.get_logger().info('Waiting for /odom ...', once=True)
            return

        now = time.time()
        if now - self.last_log_t > 2.0:
            self._publish_planned_path()
            self._publish_goal_marker()
            self._log_progress()
            self.last_log_t = now

        if self.idx >= len(self.wp):
            self.cmd_pub.publish(Twist())
            if not self.done:
                self.done = True
                self._print_metrics()
            return

        tx, ty = self.wp[self.idx]
        dx     = tx - self.x
        dy     = ty - self.y
        dist   = math.hypot(dx, dy)

        if dist < WP_REACH:
            self.idx        += 1
            self.stuck_since = None
            return

        # LiDAR obstacle handling
        lidar_factor = 1.0
        cmd = Twist()

        if self.front_dist < LIDAR_STOP:
            cmd.angular.z = MAX_ANGULAR_VEL * (1.0 if self.left_dist > self.right_dist else -1.0)
            self.cmd_pub.publish(cmd)
            if self.stuck_since is None:
                self.stuck_since = now
            elif now - self.stuck_since > REPLAN_TIMEOUT:
                self.get_logger().warn('Stuck — replanning ...')
                self._plan((self.x, self.y), self.goal_world)
                self.stuck_since = None
            return
        elif self.front_dist < LIDAR_SLOW:
            lidar_factor = max(0.2, (self.front_dist - LIDAR_STOP) / (LIDAR_SLOW - LIDAR_STOP))
        else:
            self.stuck_since = None

        # Heading error
        target_yaw = math.atan2(dy, dx)
        err = target_yaw - self.yaw
        while err >  math.pi: err -= 2 * math.pi
        while err < -math.pi: err += 2 * math.pi

        # Phase 5: speed constraints
        gx, gy    = self.goal_world
        goal_dist = math.hypot(gx - self.x, gy - self.y)
        ramp_up   = min(1.0, dist      / RAMP_DIST)
        ramp_down = min(1.0, goal_dist / RAMP_DIST)
        # Reduce speed proportionally to turn sharpness (Phase 5 requirement)
        turn_factor = max(TURN_SPEED_MULT, 1.0 - abs(err) / math.pi)
        speed_cap   = MAX_LINEAR_VEL * min(ramp_up, ramp_down) * lidar_factor * turn_factor
        speed_cap   = max(0.08, speed_cap)

        if abs(err) > TURN_THRESH:
            cmd.linear.x  = 0.0
            cmd.angular.z = max(-MAX_ANGULAR_VEL, min(MAX_ANGULAR_VEL, err * 1.8))
        else:
            cmd.linear.x  = min(speed_cap, dist * 0.8)
            cmd.angular.z = err * 0.6

        # Hard clamp and compliance tracking
        if cmd.linear.x > MAX_LINEAR_VEL:
            self.speed_violations += 1
            cmd.linear.x = MAX_LINEAR_VEL
        if abs(cmd.angular.z) > MAX_ANGULAR_VEL:
            cmd.angular.z = math.copysign(MAX_ANGULAR_VEL, cmd.angular.z)

        self.max_speed_seen = max(self.max_speed_seen, cmd.linear.x)
        self.cmd_pub.publish(cmd)

    # -----------------------------------------------------------------------
    # Evaluation metrics (PDF requirement)
    # -----------------------------------------------------------------------
    def _print_metrics(self):
        travel_time = time.time() - self.start_time if self.start_time else 0.0
        gx, gy      = self.goal_world
        final_error = math.hypot(gx - self.x, gy - self.y)
        planned_dist = sum(
            math.hypot(self.wp[i+1][0] - self.wp[i][0],
                       self.wp[i+1][1] - self.wp[i][1])
            for i in range(len(self.wp) - 1)
        ) if len(self.wp) > 1 else 0.0

        print('\n' + '=' * 56)
        print('  NAVIGATION METRICS REPORT')
        print('=' * 56)
        print(f'  1. Path length  (planned) : {planned_dist:.2f} m')
        print(f'     Path length  (actual)  : {self.dist_travelled:.2f} m')
        print(f'  2. Travel time            : {travel_time:.1f} s')
        print(f'  3. Path optimality ratio  : '
              f'{planned_dist / max(self.dist_travelled, 0.001):.3f} (1.0 = perfect)')
        print(f'  4. Navigation accuracy    : {final_error:.3f} m error at goal')
        print(f'  5. Speed compliance       :')
        print(f'     Max speed commanded    : {self.max_speed_seen:.3f} m/s  '
              f'(limit {MAX_LINEAR_VEL:.2f} m/s)')
        print(f'     Speed violations       : {self.speed_violations}')
        print(f'     Compliance status      : '
              f'{"PASS" if self.speed_violations == 0 else "FAIL"}')
        print('=' * 56 + '\n')
        self.get_logger().info('Goal reached! See metrics above.')

    def _log_progress(self):
        if not self.wp:
            return
        total   = len(self.wp)
        done    = min(self.idx, total)
        pct     = int(done / total * 28)
        bar     = 'X' * pct + '.' * (28 - pct)
        gx, gy  = self.goal_world
        gdist   = math.hypot(gx - self.x, gy - self.y)
        elapsed = time.time() - self.start_time if self.start_time else 0.0
        self.get_logger().info(
            f'[{bar}] {done}/{total} wp | {gdist:.2f} m to goal | '
            f'{elapsed:.0f} s elapsed | front={self.front_dist:.2f} m'
        )


def main():
    rclpy.init()
    node = Navigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.cmd_pub.publish(Twist())
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

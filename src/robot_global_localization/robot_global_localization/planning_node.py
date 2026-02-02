import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.srv import GetPlan

import heapq
import math

def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return (x, y, z, w)

class AStarPlanner(Node):

    def __init__(self):
        super().__init__('a_star_planner')

        self.map = None
        self.map_received = False
        self.current_pose = None

        self.robot_radius = 0.1
        self.inflated_map = None

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )

        self.path_pub = self.create_publisher(Path, '/planned_path', 10)

        self.create_service(
            GetPlan,
            '/make_plan',
            self.plan_callback
        )

    def map_callback(self, msg):
        self.map = msg
        self.map_received = True
        self.inflate_map()

    def inflate_map(self):
        w = self.map.info.width
        h = self.map.info.height
        r = int(    # inflated map radius
            math.ceil(self.robot_radius / self.map.info.resolution)
        )

        self.inflated_map = list(self.map.data)

        for y in range(h):
            for x in range(w):
                idx = y * w + x
                if self.map.data[idx] >= 50:
                    for dx in range(-r, r + 1):
                        for dy in range(-r, r + 1):
                            nx = x + dx
                            ny = y + dy
                            if 0 <= nx < w and 0 <= ny < h:
                                if dx*dx + dy*dy <= r*r:
                                    self.inflated_map[ny * w + nx] = 100

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose

    def world_to_grid(self, x, y):
        gx = int((x - self.map.info.origin.position.x) / self.map.info.resolution)
        gy = int((y - self.map.info.origin.position.y) / self.map.info.resolution)
        return gx, gy

    def grid_to_world(self, x, y):
        wx = self.map.info.origin.position.x + (x + 0.5) * self.map.info.resolution
        wy = self.map.info.origin.position.y + (y + 0.5) * self.map.info.resolution
        return wx, wy

    def is_free(self, x, y):
        index = y * self.map.info.width + x
        value = self.inflated_map[index]
        return value >= 0 and value < 50

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def plan_callback(self, request, response):        
        if not self.map_received:
            return response
        
        start = self.world_to_grid(
            self.current_pose.position.x,
            self.current_pose.position.y
        )

        goal = self.world_to_grid(
            request.goal.pose.position.x,
            request.goal.pose.position.y
        )
        
        if not self.is_free(start[0], start[1]):
            self.get_logger().warn("Start is in collision")
            return response

        if not self.is_free(goal[0], goal[1]):
            self.get_logger().warn("Goal is in collision")
            return response
        
        self.get_logger().warn(str((self.current_pose.position.x, self.current_pose.position.y)))
        self.get_logger().warn(str((request.start.pose.position.x, request.start.pose.position.y)))
        self.get_logger().warn(str((request.goal.pose.position.x, request.goal.pose.position.y)))

        open_list = []
        heapq.heappush(open_list, (0.0, start))

        came_from = {}
        cost_so_far = {}
        cost_so_far[start] = 0.0

        # theta_x = 0

        directions = [(1,0), (-1,0), (0,1), (0,-1)]

        while open_list:
            _, current = heapq.heappop(open_list)

            if current == goal:
                break

            for dx, dy in directions:
                nx = current[0] + dx
                ny = current[1] + dy

                if nx < 0 or ny < 0 or nx >= self.map.info.width or ny >= self.map.info.height:
                    continue

                if not self.is_free(nx, ny):
                    continue

                new_cost = cost_so_far[current] + 1.0
                next_cell = (nx, ny)

                if next_cell not in cost_so_far or new_cost < cost_so_far[next_cell]:
                    cost_so_far[next_cell] = new_cost
                    priority = new_cost + self.heuristic(goal, next_cell)
                    heapq.heappush(open_list, (priority, next_cell))
                    came_from[next_cell] = current

        if goal not in came_from:
            self.get_logger().warn("No path found")
            return response

        path = Path()
        path.header.frame_id = self.map.header.frame_id

        current = goal
        next = None
        while current != start:
            pose = PoseStamped()
            pose.header.frame_id = self.map.header.frame_id
            wx, wy = self.grid_to_world(current[0], current[1])
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            
            if next is None:
                yaw = 0
            else:
                dx = next[0] - current[0]
                dy = next[1] - current[1]
                yaw = math.atan2(dy, dx)

            q = quaternion_from_euler(0.0, 0.0, yaw)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]            
            
            path.poses.append(pose)
            
            next = current
            current = came_from[current]

        path.poses.reverse()
        
        self.path_pub.publish(path)

        response.plan = path
        return response


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

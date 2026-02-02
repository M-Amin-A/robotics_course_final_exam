import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from tf2_ros import Buffer, TransformListener

def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class PIDPathFollower(Node):

    def __init__(self):
        super().__init__('pid_path_follower')

        # Subscribers and publishers
        self.path_sub = self.create_subscription(
            Path,
            '/planned_path',
            self.path_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Control loop
        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        # Path storage
        self.path = []
        self.current_waypoint_index = 0

        # Robot pose (assumed known / simplified)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0  # radians

        # PID parameters
        self.kp = 2.0
        self.ki = 0.0
        self.kd = 0.2

        self.prev_error = 0.0
        self.integral = 0.0

        # Limits
        self.max_linear_speed = 0.3
        self.max_angular_speed = 1.0

        self.waypoint_tolerance = 0.5
        self.step_size = 10

        self.get_logger().info("PID Path Follower Node Started")


    def path_callback(self, msg: Path):
        self.path = msg.poses
        self.current_waypoint_index = 0
        self.get_logger().info(f"Received new path with {len(self.path)} waypoints")


    def update_pose_from_tf(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )

            self.x = t.transform.translation.x
            self.y = t.transform.translation.y

            q = t.transform.rotation
            self.yaw = yaw_from_quaternion(q)
        except(Exception) as e:
            pass
            # self.get_logger().warn(f"TF error: {e}", throttle_duration_sec=1.0)
            
    def control_loop(self):
        self.update_pose_from_tf()
            
        if not self.path or self.current_waypoint_index >= len(self.path):
            self.publish_stop()
            return

        target: PoseStamped = self.path[self.current_waypoint_index]
        target_x = target.pose.position.x
        target_y = target.pose.position.y

        # Position error
        dx = target_x - self.x
        dy = target_y - self.y
        distance_error = math.sqrt(dx * dx + dy * dy)

        # Desired heading
        desired_yaw = math.atan2(dy, dx)
        heading_error = self.normalize_angle(desired_yaw - self.yaw)

        # PID for angular velocity
        self.integral += heading_error
        derivative = heading_error - self.prev_error

        angular_z = (
            self.kp * heading_error +
            self.ki * self.integral +
            self.kd * derivative
        )

        self.prev_error = heading_error

        # Linear velocity (simple proportional control)
        linear_x = min(self.max_linear_speed, distance_error)

        # Slow down if heading error is large
        if abs(heading_error) > 0.7:
            linear_x = 0.0

        # Clamp angular velocity
        angular_z = max(
            -self.max_angular_speed,
            min(self.max_angular_speed, angular_z)
        )

        # Publish cmd_vel
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_pub.publish(cmd)

        # Waypoint reached?
        if distance_error < self.waypoint_tolerance:
            self.current_waypoint_index += self.step_size
            
        # debug
        # self.get_logger().info(f'------------------------------------------------------------')
        # self.get_logger().info(f'Current Pos: {(self.x, self.y, self.yaw)}')
        # self.get_logger().info(f'Waypoint Index: {self.current_waypoint_index}')   
        # self.get_logger().info(f'Waypoint Pos: {(target_x, target_y)}')   
        

    def publish_stop(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = PIDPathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

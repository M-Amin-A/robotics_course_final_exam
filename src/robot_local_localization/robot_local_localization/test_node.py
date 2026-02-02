import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
import math
import time


class TestNode(Node):
    def __init__(self):
        super().__init__("test_node")

        # -----------------------------
        # Declare parameters
        # -----------------------------
        self.declare_parameter("ekf_topic", "/ekf/odom")
        self.declare_parameter("meas_topic", "/measurement/odom")
        self.declare_parameter("pred_topic", "/prediction/odom")
        self.declare_parameter("real_topic", "/ground_truth_odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")

        ekf_topic = self.get_parameter("ekf_topic").value
        meas_topic = self.get_parameter("meas_topic").value
        pred_topic = self.get_parameter("pred_topic").value
        real_topic = self.get_parameter("real_topic").value
        cmd_topic = self.get_parameter("cmd_vel_topic").value

        # -----------------------------
        # Subscribers
        # -----------------------------
        self.create_subscription(Odometry, ekf_topic, self.ekf_callback, 10)
        self.create_subscription(Odometry, meas_topic, self.meas_callback, 10)
        self.create_subscription(Odometry, pred_topic, self.pred_callback, 10)
        self.create_subscription(Odometry, real_topic, self.real_callback, 10)
        

        # -----------------------------
        # Publishers
        # -----------------------------
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)

        self.real_path_pub = self.create_publisher(Path, "/path/real", 10)
        self.meas_path_pub = self.create_publisher(Path, "/path/measurement", 10)
        self.pred_path_pub = self.create_publisher(Path, "/path/prediction", 10)
        self.ekf_path_pub = self.create_publisher(Path, "/path/ekf", 10)

        # Path messages
        self.real_path = Path()
        self.real_path.header.frame_id = "odom"

        self.meas_path = Path()
        self.meas_path.header.frame_id = "odom"

        self.pred_path = Path()
        self.pred_path.header.frame_id = "odom"

        self.ekf_path = Path()
        self.ekf_path.header.frame_id = "odom"

        # -----------------------------
        # Control loop timer
        # -----------------------------
        self.timer = self.create_timer(0.05, self.control_loop)

        # Rectangle parameters
        self.state = 0
        self.state_start_time = self.get_clock().now().nanoseconds / 1e9
        self.linear_speed = 0.5
        self.angular_speed = 3.1415 / 8
        self.forward_duration = 4.0      # seconds
        self.turn_duration = 4.0         # seconds
        self.wait_duration = 1.0         # seconds

        self.get_logger().info("Test node started (rectangle path + RViz plotting).")

    # --------------------------------------------------------------
    # Callbacks for storing paths
    # --------------------------------------------------------------
    def ekf_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.ekf_path.poses.append(pose)
        self.ekf_path_pub.publish(self.ekf_path)

    def meas_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.meas_path.poses.append(pose)
        self.meas_path_pub.publish(self.meas_path)

    def pred_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.pred_path.poses.append(pose)
        self.pred_path_pub.publish(self.pred_path)
        
    def real_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        pose.pose.position.x += 4.0
        pose.pose.position.y += 2.0
        self.real_path.poses.append(pose)
        self.real_path_pub.publish(self.real_path)
        
    # --------------------------------------------------------------
    # Rectangle command generator
    # --------------------------------------------------------------
    def control_loop(self):
        now = self.get_clock().now().nanoseconds / 1e9
        t = now - self.state_start_time

        cmd = Twist()

        # STATE 0 → Move forward
        if self.state == 0:
            cmd.linear.x = self.linear_speed
            if t >= self.forward_duration:
                self.state = 1
                self.state_start_time = now
                
        # STATE 1 → Stop
        if self.state == 1:
            if t >= self.wait_duration:
                self.state = 2
                self.state_start_time = now

        # STATE 2 → Turn right 90 degrees
        elif self.state == 2:
            cmd.angular.z = self.angular_speed
            if t >= self.turn_duration:
                self.state = 3
                self.state_start_time = now
                
        # STATE 3 → Stop
        if self.state == 3:
            if t >= self.wait_duration:
                self.state = 0
                self.state_start_time = now

        # publish command
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.pos_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        self.goal_publisher_client = self.create_client(GetPlan, '/make_plan')
        
        self.timer = self.create_timer(1.0, self.publish_once)
        self.sent = False

    def publish_once(self):
        if self.sent:
            return
        
        self.publish_init_pos()        
        self.create_timer(5.0, self.publish_goal)
        
        self.sent = True

    def publish_init_pos(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.01

        self.pos_publisher.publish(msg)
        
    def publish_goal(self):
        while not self.goal_publisher_client.wait_for_service(timeout_sec=1.0):
            pass

        request = GetPlan.Request()

        request.start = PoseStamped()
        request.start.header.frame_id = 'map'
        request.start.pose.position.x = 0.0
        request.start.pose.position.y = 0.0
        request.start.pose.orientation.w = 1.0

        request.goal = PoseStamped()
        request.goal.header.frame_id = 'map'
        request.goal.pose.position.x = 15.0
        request.goal.pose.position.y = 3.0
        request.goal.pose.orientation.w = 1.0

        self.future = self.goal_publisher_client.call_async(request)
        self.future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        response = future.result()
        self.get_logger().info(f'Path size: {len(response.plan.poses)}')
        self.get_logger().info(str(response.plan.poses))
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

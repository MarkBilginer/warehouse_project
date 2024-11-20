import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from tf_transformations import quaternion_from_euler

class NavToPoseActionClient(Node):
    def __init__(self):
        super().__init__('nav_to_pose_action_client')

        # Action client for NavigateToPose
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # Subscriber to the /clicked_point topic
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.subscription = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.callback,
            qos_profile
        )
        self.get_logger().info('NavToPoseActionClient initialized and waiting for /clicked_point data.')

    def callback(self, msg):
        self.get_logger().info(f'Received Data: X={msg.point.x}, Y={msg.point.y}, Z={msg.point.z}')
        # Send navigation goal
        self.send_goal(msg.point.x, msg.point.y, 0.0)

    def send_goal(self, x, y, theta):
        self.get_logger().info('Sending goal to action server...')
        goal_pose = NavigateToPose.Goal()
        goal_pose.pose.header.frame_id = 'map'
        goal_pose.pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.pose.position.x = x
        goal_pose.pose.pose.position.y = y

        # Calculate quaternion from theta
        q = quaternion_from_euler(0, 0, theta)
        goal_pose.pose.pose.orientation.x = q[0]
        goal_pose.pose.pose.orientation.y = q[1]
        goal_pose.pose.pose.orientation.z = q[2]
        goal_pose.pose.pose.orientation.w = q[3]

        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available!')
            return
        self.get_logger().info('Action server detected.')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_pose,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        self.get_logger().info('Goal reached. Ready for a new goal.')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {feedback}')

def main(args=None):
    rclpy.init(args=args)
    action_client = NavToPoseActionClient()
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()

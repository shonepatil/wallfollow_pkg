import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile

from custom_interfaces.action import Move
from rclpy.action import ActionClient
import time


class MyActionClient(Node):

    def __init__(self):
        super().__init__('my_action_client')
        self._action_client = ActionClient(self, Move, 'turtlebot3_as')

    def send_goal(self, secs):
        goal_msg = Move.Goal()
        goal_msg.secs = secs

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

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
        self.get_logger().info('Result: {0}'.format(result.status))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            'Received feedback: {0}'.format(feedback.feedback))


def client_main(args=None):
    rclpy.init(args=args)

    action_client = MyActionClient()

    action_client.send_goal(5)

    rclpy.spin(action_client)


class WallFollow(Node):

    def __init__(self):
        # Here we have the class constructor
        # call the class constructor
        super().__init__('wallfollow')
        # create the publisher object
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # create the subscriber object
        self.subscriber = self.create_subscription(
            LaserScan, '/scan', self.move_turtlebot, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        # prevent unused variable warning
        self.subscriber
        # define the timer period for 0.5 seconds
        self.timer_period = 0.5
        # define the variable to save the received info
        self.laser_right = 0
        self.laser_front = 0
        # create a Twist message
        self.cmd = Twist()
        self.timer = self.create_timer(self.timer_period, self.motion)

    def move_turtlebot(self, msg):
        # Save the right laser scan info at 90°
        self.laser_right = msg.ranges[90]
        self.laser_front = msg.ranges[180]

    def motion(self):
        # print the data
        self.get_logger().info('right sensor: "%s"' % str(self.laser_right))
        self.get_logger().info('front sensor: "%s"' % str(self.laser_front))
        # Logic of move
        if self.laser_front < 0.5:
            self.cmd.linear.x = 0.04
            self.cmd.angular.z = 0.3
        elif self.laser_right > 0.3:
            self.cmd.linear.x = 0.1
            self.cmd.angular.z = -0.2
        elif self.laser_right < 0.3 and self.laser_right >= 0.2:
            self.cmd.linear.x = 0.2
            self.cmd.angular.z = 0.0
        else:
            self.cmd.linear.x = 0.1
            self.cmd.angular.z = 0.2
        # Publishing the cmd_vel values to topipc
        self.publisher_.publish(self.cmd)


def main(args=None):
    client_main()
    time.sleep(5)
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    wallfollow = WallFollow()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(wallfollow)
    # Explicity destroy the node
    wallfollow.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()

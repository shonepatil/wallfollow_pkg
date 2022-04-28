import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from custom_interfaces.action import Move

from geometry_msgs.msg import Twist
import time
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.executors import MultiThreadedExecutor


class MyActionServer(Node):

    def __init__(self):
        super().__init__('my_action_server')
        self._action_server = ActionServer(
            self, Move, 'turtlebot3_as', self.execute_callback)
        self.cmd = Twist()
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # create the subscriber object
        self.subscriber = self.create_subscription(
            LaserScan, '/scan', self.move_turtlebot, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        # prevent unused variable warning
        self.subscriber

        self.at_wall = False
        self.done = False
        # define the variable to save the received info
        self.laser_wall_dist = 0
        self.laser_front = 0
        self.laser_right = 0

    def move_turtlebot(self, msg):
        # Save the wall laser scan info at smallest
        self.laser_wall_dist = min(msg.ranges)
        self.laser_front = msg.ranges[180]
        self.laser_right = msg.ranges[90]

    def execute_callback(self, goal_handle):

        self.get_logger().info('Executing goal...')

        feedback_msg = Move.Feedback()
        feedback_msg.feedback = "Finding wall"

        while not self.at_wall:
            # goal_handle.publish_feedback(feedback_msg)
            # print the data
            self.get_logger().info('wall sensor dist: "%s"' % str(self.laser_wall_dist))
            self.get_logger().info('laser front dist: "%s"' % str(self.laser_front))
            if self.laser_front > self.laser_wall_dist + 0.05:
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.3
            elif self.laser_front > 0.3:
                self.cmd.linear.x = 0.2
                self.cmd.angular.z = 0.0
            else:
                if self.laser_front != 0:
                    self.cmd.linear.x = 0.0
                    self.cmd.angular.z = 0.0
                    self.publisher_.publish(self.cmd)
                    self.at_wall = True
            self.publisher_.publish(self.cmd)
            time.sleep(0.5)
        while not self.done:
            # goal_handle.publish_feedback(feedback_msg)
            # print the data
            self.get_logger().info('wall sensor dist: "%s"' % str(self.laser_wall_dist))
            self.get_logger().info('laser right dist: "%s"' % str(self.laser_right))
            if self.laser_right > self.laser_wall_dist + 0.02:
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.2
            else:
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.0
                self.publisher_.publish(self.cmd)
                self.get_logger().info('Found nearest wall.')
                self.done = True
            self.publisher_.publish(self.cmd)
            time.sleep(0.5)

        goal_handle.succeed()

        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0

        self.publisher_.publish(self.cmd)
        feedback_msg.feedback = "Finished action server. Robot moved to wall."
        result = Move.Result()
        result.status = feedback_msg.feedback
        return result


def main(args=None):
    rclpy.init(args=args)

    # my_action_server = MyActionServer()

    # rclpy.spin(my_action_server)
    try:
        my_action_server = MyActionServer()
        subscriber = my_action_server.subscriber
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(my_action_server)
        executor._take_subscription(subscriber)

        try:
            executor.spin()
        finally:
            executor.shutdown()

    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

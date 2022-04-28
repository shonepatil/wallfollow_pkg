import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile

from custom_interfaces.srv import FindWall
import sys
import time
import threading


class ClientSync(Node):

    def __init__(self):
        # Here we have the class constructor

        # call the class constructor to initialize the node as server_client
        super().__init__('find_wall_client')
        # create the service client object
        # defines the name and type of the service server we will work with.
        self.client = self.create_client(FindWall, 'find_wall')
        # checks once per second if a service matching the type and name of the client is available.
        while not self.client.wait_for_service(timeout_sec=1.0):
            # if it is not available, a message is displayed
            self.get_logger().info('service not available, waiting again...')

        # create a Empty request
        self.req = FindWall.Request()

    def send_request(self):

        # send the request
        self.req.move = sys.argv[1]
        # uses sys.argv to get access to command line input arguments for the request.
        self.future = self.client.call_async(self.req)
        # to print in the console


def client_main(args=None):
    # # initialize the ROS communication
    # rclpy.init(args=args)
    # # declare the node constructor
    # client = ClientAsync()
    # # run the send_request() method
    # client.send_request()

    # while rclpy.ok():
    #     # pause the program execution, waits for a request to kill the node (ctrl+c)
    #     rclpy.spin_once(client)
    #     if client.future.done():
    #         try:
    #             # checks the future for a response from the service
    #             # while the system is running.
    #             # If the service has sent a response, the result will be written
    #             # to a log message.
    #             response = client.future.result()
    #         except Exception as e:
    #             # Display the message on the console
    #             client.get_logger().info(
    #                 'Service call failed %r' % (e,))
    #         else:
    #             # Display the message on the console
    #             client.get_logger().info(
    #                 'Response state %r' % (response.wallfound,))
    #         break

    # client.destroy_node()
    # # shutdown the ROS communication
    # rclpy.shutdown()
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    client = ClientSync()
    # start the communication thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(client,))
    spin_thread.start()
    # run the send_request() method
    client.send_request()
    # Display the message on the console
    client.get_logger().info('Pretty message')

    client.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()


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
    return
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

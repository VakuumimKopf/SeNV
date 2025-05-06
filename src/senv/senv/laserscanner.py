import rclpy
import rclpy.node
import cv2
import numpy as np
import time

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from senv_interfaces.msg import Laser


class laserscanner(rclpy.node.Node):
    def __init__(self):
        super().__init__('laserturn')

        self.laser_distance = 0.0
        self.right_distance = 0.0
        self.left_distance = 0.0
        self.back_distance = 0.0
        self.front_distance = 0.0
        self.is_turning = False
        self.back_angle = 0
        self.front_angle = 0

        # definition of the parameters that can be changed at runtime
        self.declare_parameter('distance_to_turn', 0.45)
        self.declare_parameter('speed_drive', 0.15)
        self.declare_parameter('speed_turn', 0.5)
        self.declare_parameter('laser_front', 0)
        self.declare_parameter('turn_time', 2.0)  # must ideally equal to an integer when divided by timer_period

        self.img_row = np.random.randint(0, 256, 640, dtype=np.uint8)

        # definition of the QoS in order to receive data despite WiFi
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        # create subscribers for laser scan data with changed qos
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scanner_callback,
            qos_profile=qos_policy)
        self.subscription  # prevent unused variable warning

        # create publisher for driving commands
        self.publisher_laserturn = self.create_publisher(Laser, 'laser', qos_profile=qos_policy)

        # create timer to periodically invoke the driving logic
        self.timer_period = 0.1  # seconds
        self.my_timer = self.create_timer(self.timer_period, self.timer_callback)

    # handling received laser scan data
    def scanner_callback(self, msg):
        # self.get_logger().info("laserscanner callback" + str(len(msg.ranges)))
        # min_back = 0.0
        # min_front = 0.0
        '''
        rounded = [round(x, 3) for x in msg.ranges]
        # self.get_logger().info("laserscanner callback" + str(rounded))

        self.left_distance = min(rounded[170:190])
        self.right_distance = min(rounded[535:545])
        min_back = min(rounded[270:450])
        self.back_distance = min_back
        self.back_angle = rounded[270:450].index(min_back)
        min_front = min(min(rounded[0:90]), min(rounded[630:720]))
        self.front_distance = min_front
        if min_front == min(rounded[0:90]):
            self.front_angle = rounded[0:90].index(min_front)
        else:
            self.front_angle = rounded[630:720].index(min_front)
        # self.front_angle = rounded[0:90].index(min_front) or rounded[630:720].index(min_front)
        '''
        self.front_distance = msg.ranges[0]
        self.back_distance = msg.ranges[450]
        self.left_distance = msg.ranges[630]
        self.right_distance = msg.ranges[540]

    # driving logics
    def timer_callback(self):
        # self.get_logger().info("laserscannercallback")

        msg = Laser()
        msg.front_distance = self.front_distance
        msg.back_distance = self.back_distance
        msg.left_distance = self.left_distance
        msg.right_distance = self.right_distance
        msg.front_angle = self.front_angle
        msg.back_angle = self.back_angle
        """self.get_logger().info("front_distance: "
          + str(self.front_distance) + " front_angle: " + str(self.front_angle))
        self.get_logger().info("back_distance: " + str(self.back_distance) + " back_angle: " + str(self.back_angle))
        self.get_logger().info("left_distance: " + str(self.left_distance))
        self.get_logger().info("right_distance: " + str(self.right_distance))"""
        self.publisher_laserturn.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = laserscanner()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        print('Except in laserscanner')

    finally:
        node.destroy_node()
        print('shutting down laserscanner node')


if __name__ == '__main__':
    main()

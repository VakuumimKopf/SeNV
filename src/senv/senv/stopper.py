import rclpy
import rclpy.executors
import rclpy.node
import cv2
import numpy as np
import time

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

class Stopper(rclpy.node.Node):
    def __init__(self):
        super().__init__('stopper')
        self.publisher_stop = self.create_publisher(Twist, 'cmd_vel', 1)
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        time.sleep(0.5)
        print('Stop msg sent')
        self.get_logger().info('Stop msg sent')
        # send message
        self.publisher_stop.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    stop = Stopper()
    stop.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
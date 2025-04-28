import rclpy
import rclpy.node
import cv2
import numpy as np
import time

from std_msgs.msg import Int16
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
<<<<<<< HEAD
=======
from senv_interfaces.msg import Laser
>>>>>>> c16411254d4dbc1762a597ba784d2dfd2d3f7ad0

class laserscanner(rclpy.node.Node):
    def __init__(self):
        super().__init__('laserturn')

<<<<<<< HEAD
=======
        self.front_distance = 0.0
        self.is_turning = False

>>>>>>> c16411254d4dbc1762a597ba784d2dfd2d3f7ad0
        # definition of the parameters that can be changed at runtime
        self.declare_parameter('distance_to_turn', 0.45)
        self.declare_parameter('speed_drive', 0.15)
        self.declare_parameter('speed_turn', 0.5)
        self.declare_parameter('laser_front', 0)
        self.declare_parameter('turn_time', 2.0) # must ideally equal to an integer when divided by timer_period 


        #self.img_row = np.array([0, 64, 128, 192, 255], dtype=np.uint8) # Beispiel
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
<<<<<<< HEAD
        self.front_distance = 0.0
        # create publisher for driving commands
        self.publisher_laserturn = self.create_publisher(Int16 , 'laser', 1)
        self.is_turning = False
        # create timer to periodically invoke the driving logic
        self.timer_period = 0.5  # seconds
        self.counter = 0
        self.turn_time = self.get_parameter('turn_time').get_parameter_value().double_value
=======

        # create publisher for driving commands
        self.publisher_laserturn = self.create_publisher(Laser , 'laser', 1)

        # create timer to periodically invoke the driving logic
        self.timer_period = 0.5  # seconds
>>>>>>> c16411254d4dbc1762a597ba784d2dfd2d3f7ad0
        self.my_timer = self.create_timer(self.timer_period, self.timer_callback)
    
    # handling received laser scan data
    def scanner_callback(self, msg):
        #self.get_logger().info(f'Hallo in LaserscannerCallback: {self.front_distance}')
        # saving the required sensor value, no further processing at this point
        self.front_distance = msg.ranges[self.get_parameter('laser_front').get_parameter_value().integer_value]
        
    # driving logics
    def timer_callback(self):
        self.get_logger().info("laserscannercallback")
<<<<<<< HEAD
        if ((self.front_distance <= 0.2) & (self.front_distance != 0.0 )) :
            msg = Int16()
            msg.data = 0
        else:
            msg = Int16()
            msg.data = 1
        # send message
        #self.get_logger().info("Msg", msg)
=======
        
        msg = Laser()
        msg.distance = self.front_distance
        msg.angle = 0
>>>>>>> c16411254d4dbc1762a597ba784d2dfd2d3f7ad0
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
<<<<<<< HEAD
    main()
=======
    main()
>>>>>>> c16411254d4dbc1762a597ba784d2dfd2d3f7ad0

import rclpy
import rclpy.node
import numpy as np
from senv.stopper import Stopper
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from senv_interfaces.msg import Laser
from senv.description import float_desc, int_desc, bool_desc, light_int_desc
import array


class laserscanner(rclpy.node.Node):
    def __init__(self):
        super().__init__('laserturn')

        self.front_distance = 0.0
        self.front_left_distance = 0.0
        self.left_distance = 0.0
        self.back_left_distance = 0.0
        self.back_distance = 0.0
        self.back_right_distance = 0.0
        self.right_distance = 0.0
        self.front_right_distance = 0.0
        self.raw = []

        # definition of the parameters that can be changed at runtime
        self.declare_parameter('distance_to_turn', 0.45)
        self.declare_parameter('speed_drive', 0.15)
        self.declare_parameter('speed_turn', 0.5)
        self.declare_parameter('laser_front', 0)
        self.declare_parameter('turn_time', 2.0)

        # must ideally equal to an integer when divided by timer_period
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
        self.front_left_distance = msg.ranges[90]
        self.left_distance = msg.ranges[180]
        self.back_left_distance = msg.ranges[270]
        self.back_distance = msg.ranges[360]
        self.back_right_distance = msg.ranges[450]
        self.right_distance = msg.ranges[540]
        self.front_right_distance = msg.ranges[630]
        self.raw = array.array('d', msg.ranges)
        # saving the required sensor value, no further processing at this point
        self.front_distance = msg.ranges[self.get_parameter('laser_front')
                                         .get_parameter_value().integer_value]

    # driving logics
    def timer_callback(self):
        # self.get_logger().info("laserscannercallback")

        msg = Laser()
        msg.front_distance = self.front_distance
        msg.front_left_distance = self.front_left_distance
        msg.left_distance = self.left_distance
        msg.back_left_distance = self.back_left_distance
        msg.back_distance = self.back_distance
        msg._back_right_distance = self.back_right_distance
        msg.right_distance = self.right_distance
        msg.front_right_distance = self.front_right_distance
        msg.raw = self.raw
        self.publisher_laserturn.publish(msg)
        #  self.get_logger().info("laserscanner : " + str(msg.raw))


def main(args=None):
    rclpy.init(args=args)
    node = laserscanner()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:

        node.destroy_node()

    finally:
        node.destroy_node()
        print('Shutting Down LaserScanner')


if __name__ == '__main__':
    main()

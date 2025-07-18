import rclpy
import rclpy.node
import numpy as np
from sensor_msgs.msg import LaserScan
from senv_interfaces.msg import Laser
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

        if msg is not None:
            # self.get_logger().info("Laserscan exists")
            self.front_distance = msg.ranges[0]
            self.front_left_distance = msg.ranges[90]
            self.left_distance = msg.ranges[180]
            self.back_left_distance = msg.ranges[270]
            self.back_distance = msg.ranges[360]
            self.back_right_distance = msg.ranges[450]
            self.right_distance = msg.ranges[540]
            self.front_right_distance = msg.ranges[630]
            self.raw = array.array('d', msg.ranges)

    # driving logics
    def timer_callback(self):
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

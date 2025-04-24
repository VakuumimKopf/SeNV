import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class driver(Node):
    def __init__(self):
        super().__init__('driver')
        self.get_logger().info('driver node has been started.')
        
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,depth=1)

        self.subscriber = self.create_subscription(
            Twist,  # Replace with the actual message type
            'driving',
            self.driving_callback, 
            qos_profile=qos_policy
        )
        
        self.subscriber  # prevent unused variable warning


        #self.publisher = self.create_publisher(Twist, 'cmd_vel', 1)


        #timer
        # self.timer = self.create_timer(0.1, self.callback)
        
    def driving_callback(self, msg: Twist):
        # Process the incoming message
        self.get_logger().info("Driver recieved data" + str(msg.linear.x))
        # define direction based on the sign in msg

    def callback(self):
        self.get_logger().info('Executing callback...')

        msg = Twist()
        self.publisher.publish(msg)
        


def main(args=None):
    rclpy.init(args=args)
    node = driver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
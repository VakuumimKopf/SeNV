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

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 1)

    def driving_callback(self, msg: Twist):
        # Process the incoming message
        self.get_logger().info("Driver recieved data: " + str(msg.linear.x) + " : " + str(msg.angular.z))
        
        # Debuga algorithm here here 

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = driver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
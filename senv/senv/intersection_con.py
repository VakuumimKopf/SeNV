import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class intersection_con(Node):
    def __init__(self):
        super().__init__('intersection_con')
        self.get_logger().info('intersection_con node has been started.')
        

        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,depth=1)

        self.subscriber = self.create_subscription(
            any,  # Replace with the actual message type
            'crossing_intersection',
            self.crossing_intersection_callback, 

            qos_profile=qos_policy
        )
        
        self.subscriber  # prevent unused variable warning

        self.publisher = self.create_publisher(
            Twist,  # Replace with the actual message type
            'driving',
            qos_profile=qos_policy
        )
        self.publishers

        self.publishers = self.create_publisher(
            any,  # Replace with the actual message type
            'finished',
            qos_profile=qos_policy
        )
        self.publishers

        #timer
        self.timer = self.create_timer(0.1, self.callback)
        
    def crossing_intersection_callback(self, msg):
        # Process the incoming message
        self.get_logger().info('Received message: %s' % msg.data)
        # define direction based on the sign in msg

    def callback(self):
        # Define your callback function here
        self.get_logger().info('Executing callback...')
        # driving logic

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = intersection_con()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
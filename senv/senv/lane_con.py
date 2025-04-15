import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist



class lane_con(Node):
    def __init__(self):
        super().__init__('lane_con')
        self.get_logger().info('lane_con node has been started.')
        

        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,depth=1)

        self.subscriber = self.create_subscription(
            any,  # Replace with the actual message type
            'driving_lane_obstacle',
            self.line_driving_callback, 

            qos_profile=qos_policy
        )
        
        self.subscriber  # prevent unused variable warning

        self.publisher = self.create_publisher(
            Twist,  # Replace with the actual message type
            'driving',
            qos_profile=qos_policy
        )
        self.publisher

        self.publisher = self.create_publisher(
            any,  # Replace with the actual message type
            'finished',
            qos_profile=qos_policy
        )
        self.publisher

        #timer
        self.timer = self.create_timer(0.1, self.callback)
        
    def line_driving_callback(self, msg):
        # Process the incoming message
        self.get_logger().info('Received message: %s' % msg.data)
        # Add driving logic here

    def callback(self):
        # Define your callback function here
        self.get_logger().info('Executing callback...')
        # obstacle avoidance


def main(args=None):
    rclpy.init(args=args)
    node = lane_con()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from senv_interfaces.msg import Pic,Laser


class lane_con(Node):
    def __init__(self):
        super().__init__('lane_con')
        self.get_logger().info('lane_con node has been started.')
        
        # Qos policy setting
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,depth=1)

        self.subscriber_pic = self.create_subscription(
            Pic,  # Replace with the actual message type
            'pic',
            self.pic_callback,
            qos_profile=qos_policy
        )
        self.subscriber_pic  # prevent unused variable warning

        self.subscriber_laser = self.create_subscription(
            Laser,  # Replace with the actual message type
            'laser',
            self.laser_callback,
            qos_profile=qos_policy
        )
        self.subscriber_laser  # prevent unused variable warning
        
    def pic_callback(self, msg):
        # Process the incoming message
        self.get_logger().info('Received message pic')
        # Add driving logic here

    def laser_callback(self, msg):
        # Define your callback function here
        self.get_logger().info('Received message laser')
        # obstacle avoidance


def main(args=None):
    rclpy.init(args=args)
    node = lane_con()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
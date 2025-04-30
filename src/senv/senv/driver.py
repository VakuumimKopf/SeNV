import rclpy
import rclpy.executors
import rclpy.node
from std_msgs.msg import Int16
from senv.stopper import Stopper
from geometry_msgs.msg import Twist
from enum import Enum

class State(Enum):
    FollowLine = 1
    TurnOnObject = 2
    Error = 3

class Driver(rclpy.node.Node):
    def __init__(self):
        super().__init__('driver')

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

    print('Hi from Driver')
    rclpy.init(args=args, signal_handler_options=rclpy.SignalHandlerOptions.NO)
    driver_node = Driver()
    
    try:
        rclpy.spin(driver_node)

    except KeyboardInterrupt:
        driver_node.destroy_node()
        stop = Stopper()

    finally:
        driver_node.destroy_node()
        stop.destroy_node()
        rclpy.shutdown()
        print('Shutting Down Driver')


if __name__ == '__main__':
    main()

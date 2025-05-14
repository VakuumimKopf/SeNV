import rclpy
import rclpy.action
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from geometry_msgs.msg import Twist
from senv_interfaces.msg import Pic, Laser
from senv_interfaces.action import ConTask
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, IntegerRange, FloatingPointRange

class park_con(Node):
    def int_desc(desc):
        min_val=0
        max_val=255
        step=1
        return ParameterDescriptor(type= ParameterType.PARAMETER_INTEGER, description=desc, 
                                   integer_range=[IntegerRange(from_value=min_val, to_value=max_val, step=step)])
    def float_desc(desc):
        min_val=0.0 
        max_val=2.0
        step=0.001
        return ParameterDescriptor(type= ParameterType.PARAMETER_DOUBLE, description=desc, 
                                   floating_point_range=[FloatingPointRange(from_value=min_val, to_value=max_val, step=step)])
    def bool_desc(desc):
        return ParameterDescriptor(type=ParameterType.PARAMETER_BOOL, description = desc)
    
    def __init__(self):
        super().__init__('park_con')
        self.get_logger().info('park_con node has been started.')

        # Parameters
        self.turned_on = False

        # QOS Policy Setting
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)

        # Subscribing to camera and laser topics
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

        self.intersection_task_server_ = ActionServer(
            self,
            ConTask,
            "park_task",
            self.execute_callback
        )

    def execute_callback(self, goal_handle: ServerGoalHandle):

        # Get request from goal
        target = goal_handle.request.start_working
        self.get_logger().info("starting park server")

        # Execute action
        self.turned_on = target
        self.datahandler()

        # Final Goal State
        goal_handle.succeed()

        # Result
        result = ConTask.Result()
        result.finished = True
        return result

    def pic_callback(self, msg):
        if self.turned_on is False:
            return
        # Process the incoming message
        self.get_logger().info('Received message pic')
        # Add driving logic here

    def laser_callback(self, msg):
        if self.turned_on is False:
            return
        # Define your callback function here
        self.get_logger().info('Received message laser')
        # obstacle avoidance

    def datahandler(self):
        self.get_logger().info("Handling park data")


def main(args=None):
    rclpy.init(args=args)
    node = park_con()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:

        node.destroy_node()

    finally:
        #stop = Stopper()
        node.destroy_node()
        #stop.destroy_node()
        #rclpy.shutdown()
        print('Shutting Down Park_Con')


if __name__ == '__main__':
    main()

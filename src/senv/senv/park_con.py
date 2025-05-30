import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from senv_interfaces.msg import Pic, Laser
from senv_interfaces.action import ConTask
from senv.description import float_desc, int_desc, bool_desc, light_int_desc


class park_con(Node):
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
        node.destroy_node()
        print('Shutting Down Park_Con')


if __name__ == '__main__':
    main()

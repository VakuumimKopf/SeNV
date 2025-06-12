import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.action.server import ServerGoalHandle
from senv_interfaces.msg import Pic, Laser, Move
from senv_interfaces.action import ConTask
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup


class Intersection_con(Node):
    def __init__(self):
        super().__init__('intersection_con')
        self.get_logger().info('intersection_con node has been started.')

        # Parameters
        self.turned_on = False

        # QOS Policy Setting
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)

        # Subscribing to camera and laser topics
        self.subscriber_pic = self.create_subscription(
            Pic,
            'pic',
            self.pic_callback,
            qos_profile=qos_policy
        )
        self.subscriber_pic  # prevent unused variable warning

        self.subscriber_laser = self.create_subscription(
            Laser,
            'laser',
            self.laser_callback,
            qos_profile=qos_policy
        )
        self.subscriber_laser  # prevent unused variable warning

        self.intersection_task_server_ = ActionServer(
            self,
            ConTask,
            "intersection_task",
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self.publisher_driver = self.create_publisher(Move, "drive", qos_policy)

    def execute_callback(self, goal_handle: ServerGoalHandle):

        # Get request from goal
        target = goal_handle.request.start_working
        info = goal_handle.request.info
        self.get_logger().info("starting intersection server")

        # Turn in action server
        self.turned_on = target

        # Execute code as long as node is turned on
        while self.turned_on is True:
            self.datahandler(info)

        # Final Goal State
        goal_handle.succeed()

        # Result
        result = ConTask.Result()
        result.finished = True
        return result

    def cancel_callback(self, goal_handle):
        self.turned_on = False
        return CancelResponse.ACCEPT

    def pic_callback(self, msg):
        if self.turned_on is False:
            return

    def laser_callback(self, msg):
        if self.turned_on is False:
            return

    def datahandler(self, info):
        self.get_logger().info("Handling intersection data")
        self.get_logger().info(str(info))


def main(args=None):
    rclpy.init(args=args)
    intersection_con = Intersection_con()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(intersection_con, executor=executor)
    except KeyboardInterrupt:
        intersection_con.destroy_node()
    finally:
        intersection_con.destroy_node()
        print('Shutting Down intersection_con')


if __name__ == '__main__':
    main()

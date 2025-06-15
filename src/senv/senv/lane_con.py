import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.action.server import ServerGoalHandle
from senv_interfaces.msg import Pic, Laser, Move
from senv_interfaces.action import ConTask
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup


class Lane_con(Node):
    def __init__(self):
        super().__init__('lane_con')
        self.get_logger().info('lane_con node has been started.')

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

        self.lane_task_server_ = ActionServer(
            self,
            ConTask,
            "lane_task",
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self.publisher_driver = self.create_publisher(Move, "drive", 1)

    def execute_callback(self, goal_handle: ServerGoalHandle):

        # Get request from goal
        target = goal_handle.request.start_working
        info = goal_handle.request.info
        self.get_logger().info("starting lane server")

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
        msg = Move()
        msg.follow = True
        msg.speed = 1
        msg.turn = 0
        self.publisher_driver.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    lane_con = Lane_con()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(lane_con, executor=executor)
    except KeyboardInterrupt:
        lane_con.destroy_node()
    finally:
        lane_con.destroy_node()
        print('Shutting Down lane_con')


if __name__ == '__main__':
    main()

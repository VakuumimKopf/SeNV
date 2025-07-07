import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.action.server import ServerGoalHandle
from senv_interfaces.msg import Laser, Move
from senv_interfaces.action import ConTask
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import time


class Obstacle_con(Node):
    def __init__(self):
        super().__init__('obstacle_con')
        self.get_logger().info('obstacle_con node has been started.')

        # Parameters
        self.obstacle_state = 0
        self.right_distance = 0.0
        self.front_distance = 0.0
        self.front_right_distance = 0.0
        self.turned_on = False
        self.right_range = []
        self.raw = []
        self.right_closest = 0.0
        self.right_check = 0.0

        # QOS Policy Setting
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)

        self.subscriber_laser = self.create_subscription(
            Laser,
            'laser',
            self.laser_callback,
            qos_profile=qos_policy
        )
        self.subscriber_laser  # prevent unused variable warning

        self.obstacle_task_server_ = ActionServer(
            self,
            ConTask,
            "obstacle_task",
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self.publisher_driver = self.create_publisher(Move, "drive", 1)

    def execute_callback(self, goal_handle: ServerGoalHandle):

        # Get request from goal
        target = goal_handle.request.start_working
        self.get_logger().info("Starting obstacle server")

        # Turn on flag
        self.turned_on = target

        # Wait for process to finish
        while self.turned_on is True:
            self.datahandler()
            self.wait_ros2(0.4)

        self.get_logger().info("Obstacle handling finished, turning off")

        self.obstacle_state = 0

        # Final Goal State
        goal_handle.succeed()
        result = ConTask.Result()
        result.finished = True
        return result

    def cancel_callback(self):
        if self.turned_on is True:
            return CancelResponse.REJECT
        else:
            return CancelResponse.ACCEPT

    def laser_callback(self, msg):
        if self.turned_on is False:
            return
        self.raw = msg.raw
        self.front_distance = msg.front_distance
        self.front_right_distance = msg.front_right_distance
        self.right_check = self.raw[674]
        self.right_range = self.raw[540:719]
        if len(self.right_range) == 0:
            self.right_range = [0.0]
        self.right_closest = min(min(self.right_range), self.front_distance)

    # Handling the data for the obstacle avoidance
    def datahandler(self):
        msg = Move()
        msg.follow = True
        if self.obstacle_state == 0:
            if self.right_closest < 0.3:
                msg.speed = 0.0
                msg.turn = 0
                self.publisher_driver.publish(msg)
                self.wait_ros2(2)
                if self.right_check < 0.35 or self.front_distance < 0.35:
                    self.obstacle_state = 1
                else:
                    self.obstacle_state = 4
            else:
                self.wait_ros2(1)
                if self.right_closest < 0.3:
                    msg.speed = 0.0
                    msg.turn = 0
                    self.publisher_driver.publish(msg)
                    self.wait_ros2(2)
                    if self.right_check < 0.35 or self.front_distance < 0.35:
                        self.obstacle_state = 1
                    else:
                        self.obstacle_state = 4
                else:
                    self.obstacle_state = 4

        elif self.obstacle_state == 1:
            self.turn90(1.0)
            self.drive_length(1.3)
            self.turn90(-1.0)
            self.follow_length(1.0)
            self.obstacle_state = 2
        elif self.obstacle_state == 2:
            self.drive_along_obstacle()
        elif self.obstacle_state == 3:
            self.turn90(-1.0)
            self.drive_length(1.3)
            self.turn90(1.0)
            self.obstacle_state = 4
        elif self.obstacle_state == 4:
            msg = Move()
            msg.follow = True
            msg.speed = 0.7
            msg.turn = 0
            self.publisher_driver.publish(msg)
            self.turned_on = False

    # Wait for a given duration in seconds
    def wait_ros2(self, duration: float):
        i = duration * 100
        while i > 0:
            time.sleep(0.01)
            i -= 1

    # Turn the robot 90 degrees in a given direction
    # direction: 1 for left, -1 for right
    def turn90(self, direction):
        msg = Move()
        msg.follow = False
        msg.speed = 1.0
        msg.override = True
        msg.speed_o = 0.0
        msg.turn_o = 0.5 * direction
        self.publisher_driver.publish(msg)
        self.wait_ros2(3)
        msg.turn_o = 0.0
        self.publisher_driver.publish(msg)

    # Drive the robot for a given duration
    def drive_length(self, duration):
        msg = Move()
        msg.follow = False
        msg.override = True
        msg.speed = 1.0
        msg.speed_o = 0.2
        msg.turn_o = 0.0
        self.publisher_driver.publish(msg)
        self.wait_ros2(duration)
        msg.speed_o = 0.0
        self.publisher_driver.publish(msg)

    # line holding for a given duration
    def follow_length(self, duration):
        msg = Move()
        msg.follow = True
        msg.override = False
        msg.speed = 0.5
        self.publisher_driver.publish(msg)
        self.wait_ros2(duration)
        msg.speed_o = 0.0
        self.publisher_driver.publish(msg)

    # Drive along the obstacle if it is too close
    def drive_along_obstacle(self):
        msg = Move()
        msg.follow = True
        msg.turn = 0
        if self.right_closest < 0.37:
            msg.speed = 0.7
            self.publisher_driver.publish(msg)
        else:
            self.wait_ros2(0.5)
            if self.right_closest < 0.37:
                msg.speed = 0.7
                self.publisher_driver.publish(msg)
            self.obstacle_state = 3    # Continue with your state machine


def main(args=None):
    rclpy.init(args=args)
    obstacle_con = Obstacle_con()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(obstacle_con, executor=executor)
    except KeyboardInterrupt:
        obstacle_con.destroy_node()
    finally:
        obstacle_con.destroy_node()
        print('Shutting Down obstacle_con')


if __name__ == '__main__':
    main()

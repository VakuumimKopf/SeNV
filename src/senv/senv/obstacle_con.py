import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.action.server import ServerGoalHandle
from senv_interfaces.msg import Pic, Laser, Move
from senv_interfaces.action import ConTask
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2


class Obstacle_con(Node):
    def __init__(self):
        super().__init__('obstacle_con')
        self.get_logger().info('obstacle_con node has been started.')

        # Parameters
        self.obstacle_state = 0
        self.right_distance = 0.0
        self.front_distance = 0.0
        self.turned_on = True
        self.right_range = []
        self.raw = []
        self.right_closest = 0.0
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
            callback_groups=ReentrantCallbackGroup(),
        )

        self.publisher_driver = self.create_publisher(Move, "drive", 1)

    def execute_callback(self, goal_handle: ServerGoalHandle):

        # Get request from goal
        target = goal_handle.request.start_working
        info = goal_handle.request.info
        self.get_logger().info("Starting obstacle server")

        # Turn on flag
        self.turned_on = target

        # Start main logic via a timer to avoid blocking
        self.main_timer = self.create_timer(0.1, lambda: self.datahandler(info))

        # Wait for process to finish
        while self.turned_on is True:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info("Obstacle handling finished, turning off")
        # Stop the timer when finished
        # self.main_timer.cancel()

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
        self.right_range = self.raw[540:719]
        if len(self.right_range) == 0:
            self.right_range = [0.0]
        self.right_closest = min(self.right_range)
        self.front_distance = msg.front_distance

    def datahandler(self, info):
        self.get_logger().info("shortest obstacle" + str(self.right_closest))
        if self.obstacle_state == 0:
            if self.front_distance < 0.4:
                self.obstacle_state = 1
                self.get_logger().info("Obstacle detected in front, changing state to 1")
            else:
                self.wait_ros2(1)
                if self.right_distance < 0.4:
                    self.obstacle_state = 1
                else:
                    self.get_logger().info("No obstacle detected, waiting for next scan")
                    return
        elif self.obstacle_state == 1:
            self.get_logger().info("Turn 90 degress to the left")
            self.turn90(1.0)
            self.obstacle_state = 2
        elif self.obstacle_state == 2:
            self.get_logger().info("Driving along obstacle")
            self.drive_length(1.3)
            self.obstacle_state = 3
        elif self.obstacle_state == 3:
            self.get_logger().info("Turning 90 degrees to the right")
            self.turn90(-1.0)
            self.obstacle_state = 4
        elif self.obstacle_state == 4:
            self.drive_timer = self.create_timer(0.1, self.drive_along_obstacle)
        elif self.obstacle_state == 5:
            self.turn90(-1.0)
            self.obstacle_state = 6
        elif self.obstacle_state == 6:
            self.drive_length(1.3)
            self.obstacle_state = 7
        elif self.obstacle_state == 7:
            self.turn90(1.0)
            self.get_logger().info("Obstacle handling finished, turning off")
            msg = Move()
            msg.follow = True
            msg.speed = 0.7
            msg.turn = 0
            self.publisher_driver.publish(msg)
            self.turned_on = False
            self.main_timer.cancel()  # Stop the main timer

    def wait_ros2(self, duration):
        """ROS 2-kompatibles Warten, ohne Callbacks zu blockieren."""
        start_time = self.get_clock().now().nanoseconds
        while (self.get_clock().now().nanoseconds - start_time) / 1e9 < duration:
            rclpy.spin_once(self, timeout_sec=0.1)

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

    def drive_along_obstacle(self):
        msg = Move()
        msg.follow = True
        msg.turn = 0
        if self.right_closest < 0.6:
            msg.speed = 0.7
            self.publisher_driver.publish(msg)
            # self.get_logger().info("distance: " + str(self.right_closest))
        else:
            msg.speed = 0.0
            self.publisher_driver.publish(msg)
            self.get_logger().info("Finished driving along obstacle")
            self.drive_timer.cancel()  # Stop the timer
            self.obstacle_state = 5    # Continue with your state machine


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

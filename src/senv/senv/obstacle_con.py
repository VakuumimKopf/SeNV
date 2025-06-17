import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist
from rclpy.action.server import ServerGoalHandle
from senv_interfaces.msg import Pic, Laser
from senv_interfaces.action import ConTask
from std_msgs.msg import String
from senv.description import float_desc, int_desc, light_int_desc, bool_desc


class obstacle_con(Node):
    def __init__(self):
        super().__init__('obstacle_con')
        self.get_logger().info("obstacle_con node started")
        # Parameters
        self.declare_parameter('distance_to_obstacle', 0.6, float_desc(
            "Gewünschter Abstand zum Objekt"))

        self.turned_on = False
        self.state_obstacle = "Unknown"
        self.right_distance = 0.0

        # QOS Policy Setting
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)

        # Subscriptions
        self.subscriber_pic = self.create_subscription(
            Pic, 'pic', self.pic_callback, qos_profile=qos_policy
            )
        self.subscriber_laser = self.create_subscription(
            Laser, 'laser', self.laser_callback, qos_profile=qos_policy
            )

        # Publishers
        self.publisher_driver = self.create_publisher(Twist, 'driving', qos_profile=qos_policy)
        self.publisher_state = self.create_publisher(String, 'state', qos_profile=qos_policy)

        # Action Server
        self.obstacle_task_server = ActionServer(
            self, ConTask, "obstacle_task", self.execute_callback
            )

    def execute_callback(self, goal_handle):
        target = goal_handle.request.start_working
        self.turned_on = target
        self.datahandler()
        goal_handle.succeed()
        result = ConTask.Result()
        result.finished = True
        return result

    def pic_callback(self, msg):
        if not self.turned_on:
            return

    def laser_callback(self, msg: Laser):
        if not self.turned_on:
            return

        self.right_distance = msg.right_distance
        if self.right_distance == "inf":
            self.right_distance = 0.0
        if msg.front_distance <= 0.5 and msg.front_distance != 0:
            self.state_obstacle = "Infront"

        self.get_logger().info(f"right distance: {self.right_distance}")

    def datahandler(self):
        if self.state_obstacle == "Infront":
            self.turn90(1.0, 3)
            self.drive_length(2)
            self.turn90(-1.0, 3)
            self.drive_length(0)
            while self.right_distance > 0.6:
                self.get_logger().info("waiting till obstacle is right of me")
                self.wait_ros2(0.1)

            while self.right_distance < 0.6:
                self.drive_along()
                self.get_logger().info("Driving along obstacle")
                self.wait_ros2(0.1)

            self.drive_length(0.3)
            self.turn90(-1.0, 3)
            if self.right_distance > 0.8:
                self.drive_length(0.2)
            while self.right_distance > 0.8:
                # self.get_logger().info("waiting till obstacle is right of me")
                self.wait_ros2(0)

            self.turn90(1.0, 3)
            self.get_logger().info("Obstacle avoidance finished")

    def turn90(self, direction, duration):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.5 * direction
        self.publisher_driver.publish(msg)

        # ROS 2-kompatibles Warten
        self.wait_ros2(duration)

        msg.angular.z = 0.0
        self.publisher_driver.publish(msg)
        self.get_logger().info("Turned 90 degrees")

    def drive_length(self, duration):
        msg = Twist()

        msg.linear.x = 0.175
        msg.angular.z = 0.0
        self.publisher_driver.publish(msg)
        self.wait_ros2(duration)

        # ROS 2-kompatibles Warten

        # msg.linear.x = 0.0
        # self.publisher_driver.publish(msg)
        self.get_logger().info("Drove length")

    def drive_along(self):
        distance_to_obstacle = self.get_parameter(
            'distance_to_obstacle').get_parameter_value().double_value
        msg = Twist()
        msg.linear.x = 0.1

        if self.right_distance < distance_to_obstacle:
            msg.angular.z = 0.25
        else:
            msg.angular.z = - 0.25

        self.publisher_driver.publish(msg)

    def back_turn90(self):
        msg = Twist()
        while self.right_distance < 0.8:
            msg.linear.x = 0.0
            msg.angular.z = 0.2
            self.publisher_driver.publish(msg)

        msg.angular.z = 0.0
        self.publisher_driver.publish(msg)
        self.get_logger().info("Back turn 90 degrees")

    def wait_ros2(self, duration):
        """ROS 2-kompatibles Warten, ohne Callbacks zu blockieren."""
        start_time = self.get_clock().now().nanoseconds
        while (self.get_clock().now().nanoseconds - start_time) / 1e9 < duration:
            rclpy.spin_once(self, timeout_sec=0.1)


def main(args=None):
    rclpy.init(args=args)
    node = obstacle_con()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.destroy_node()

    finally:
        node.destroy_node()
        print('Shutting Down Obstacle_con')


if __name__ == '__main__':
    main()

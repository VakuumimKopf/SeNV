import rclpy
import rclpy.action
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from geometry_msgs.msg import Twist
from senv_interfaces.msg import Pic, Laser
from senv_interfaces.action import ConTask
import time
from std_msgs.msg import String


class obstacle_con(Node):
    def __init__(self):
        super().__init__('obstacle_con')
        self.get_logger().info('obstacle_con node has been started.')

        # Parameters
        self.turned_on = False
        self.state_obstacle = "Unknown"
        self.closest_obstacle = 0.0
        self.closest_obstacle_angle = 0.0
        self.last_state_obstacle = "Unknown"

        self.declare_parameter('distance_min', 0.15)
        self.declare_parameter('distance_max', 0.25)
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
            Laser,
            'laser',
            self.laser_callback,
            qos_profile=qos_policy
        )
        self.subscriber_laser  # prevent unused variable warning

        self.obstacle_task_server = ActionServer(
            self,
            ConTask,
            "obstacle_task",
            self.execute_callback
        )
        self.publisher_driver = self.create_publisher(
            Twist,
            'driver',
            qos_profile=qos_policy
        )
        self.publisher_driver  # prevent unused variable warning

        self.publisher_state = self.create_publisher(
            String,
            'state',
            qos_profile=qos_policy
        )
        self.publisher_state  # prevent unused variable warning

    def execute_callback(self, goal_handle: ServerGoalHandle):

        # Get request from goal
        target = goal_handle.request.start_working
        self.get_logger().info(f"starting obstacle server {target}")

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
        # self.get_logger().info('Received message pic')
        # Add driving logic here

    def laser_callback(self, msg):
        if self.turned_on is False:
            return
        # Define your callback function here
        # self.get_logger().info('Received message laser')
        # Process the incoming message
        self.closest_obstacle = min(msg.front_distance, msg.back_distance, msg.left_distance, msg.right_distance)
        if self.closest_obstacle == msg.front_distance:
            self.closest_obstacle_angle = msg.front_angle
            self.state_obstacle = "Infront"
        elif self.closest_obstacle == msg.back_distance:
            self.closest_obstacle_angle = msg.back_angle
            self.state_obstacle = "Behind"
        elif self.closest_obstacle == msg.left_distance:
            self.closest_obstacle_angle = None
            self.state_obstacle = "Left"
        elif self.closest_obstacle == msg.right_distance:
            self.closest_obstacle_angle = None
            self.state_obstacle = "Right"
        else:
            self.closest_obstacle_angle = None
            self.state_obstacle = "Unknown"

        self.get_logger().info(f"Obstacle state: {self.state_obstacle}")

    def datahandler(self):

        distance_min = self.get_parameter('distance_min').get_parameter_value().double_value
        distance_max = self.get_parameter('distance_max').get_parameter_value().double_value
        round_count = 0
        self.get_logger().info(f"laser_distance: {self.closest_obstacle}")
        self.get_logger().info(f"laser angle: {self.closest_obstacle_angle}")

        msg_state = String()
        msg_state.data = "Obstacle_avoidance"
        self.publisher_state.publish(msg_state)
        while self.state_obstacle != "Done" and self.turned_on:

            # Create a Twist message
            msg = Twist()

            # Set linear and angular velocities based on the obstacle state
            if self.state_obstacle == "Infront":
                msg.linear.x = 0.0
                msg.angular.z = 0.5
            elif self.state_obstacle == "Behind":
                msg.linear.x = 0.0
                msg.angular.z = -0.5
            # elif self.state_obstacle == "Left" or round_count >= 2:
            #     msg.linear.x = 0.5
            #     if self.closest_obstacle < distance_min:
            #         msg.angular.z = 0.5
            #     elif self.closest_obstacle > distance_max:
            #         msg.angular.z = -0.5
            #     else:
            #         msg.angular.z = 0.0
            elif self.state_obstacle == "Right":
                msg.linear.x = 0.5
                msg.angular.z = 0.0
                if self.closest_obstacle < distance_min:
                    msg.angular.z = 0.5
                elif self.closest_obstacle > distance_max:
                    msg.angular.z = -0.5
                else:
                    msg.angular.z = 0.0
                if self.last_state_obstacle != "Right":
                    round_count += 1

            else:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
            self.last_state_obstacle = self.state_obstacle
            # Publish the Twist message
            self.publisher_driver.publish(msg)
            if round_count >= 2:
                self.state_obstacle = "Done"
                break
            # Sleep for a short duration to control the rate of publishing
            time.sleep(0.1)

        msg_state = String()
        msg_state = "Obstacle_avoidance_finished"
        self.publisher_state.publish(msg_state)


def main(args=None):
    rclpy.init(args=args)
    node = obstacle_con()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

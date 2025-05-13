import rclpy
import rclpy.action
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from geometry_msgs.msg import Twist
from senv_interfaces.msg import Pic, Laser
from senv_interfaces.action import ConTask
import time
from senv.stopper import Stopper
from std_msgs.msg import String
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, IntegerRange, FloatingPointRange
def light_int_desc(desc):
    min_val=0 
    max_val=255 
    step=1
    return ParameterDescriptor(type= ParameterType.PARAMETER_INTEGER, description=desc, 
                                integer_range=[IntegerRange(from_value=min_val, to_value=max_val, step=step)])
def int_desc(desc):
    min_val=0
    max_val=1000
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

class obstacle_con(Node):
    def __init__(self):
        super().__init__('obstacle_con')
        self.get_logger().info("Obstacle avoidance node started")

        # Parameters
        self.declare_parameter('distance_to_obstacle', 0.4, float_desc("Gewünschter Abstand zum Objekt"))
        
        self.turned_on = False
        self.state_obstacle = "Unknown"
        self.right_distance = 0.0

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
        self.get_logger().info("subscribed to laser")
        self.subscriber_laser  # prevent unused variable warning

        self.obstacle_task_server = ActionServer(
            self,
            ConTask,
            "obstacle_task",
            self.execute_callback
        )
        self.publisher_driver = self.create_publisher(
            Twist,
            'driving',
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
        # self.get_logger().info(f"starting obstacle server {target}")

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

    def laser_callback(self, msg: Laser):
        if self.turned_on is False:
            return

        # Define your callback function here
        self.get_logger().info('Received message laser')

        # Process the incoming message
        self.right_distance = msg.right_distance
        if msg.front_distance <= 0.5 and msg.front_distance != 0:
            self.state_obstacle = "Infront"
        self.get_logger().info(f"Obstacle state: {self.state_obstacle}")

    def datahandler(self):
        distance_to_obstacle = self.get_parameter('distance_to_obstacle').get_parameter_value().double_value
        # driving logic
        if self.state_obstacle == "Infront":

            self.turn90(1.0)
            self.get_logger().info("First Turn done")

            self.drive_length(1.75)
            self.get_logger().info("Lane Changed")

            self.turn90(-1.0)
            self.get_logger().info("Second Turn done")

            self.drive_length(2.5)

            if self.right_distance == "inf":
                self.right_distance = 0.0

            self.get_logger().debug("Right_distance: " + str(self.right_distance))

            while self.right_distance >= distance_to_obstacle:
                self.get_logger().debug("Right_distance: " + str(self.right_distance))
                time.sleep(0.1)

            while self.right_distance < distance_to_obstacle:
                self.drive_along(1, distance_to_obstacle)
                self.get_logger().info("Driving along obstacle")
                time.sleep(1)

            self.drive_length(1)
            self.get_logger().info("Drove by obstacle")
            self.turn90(-1.0)
            self.get_logger().info("Third Turn done")
            self.drive_length(1.75)
            self.get_logger().info("Lane Changed back")
            self.turn90(1.0)
            self.get_logger().info("Obstacle avoidance finished")

        # finish the task

    def turn90(self, direction):
        # Create a Twist message
        msg = Twist()
        # Set linear and angular velocities based on the obstacle state
        msg.linear.x = 0.0
        msg.angular.z = 0.5 * direction
        # Publish the Twist message
        self.publisher_driver.publish(msg)
        time.sleep(3)
        msg.angular.z = 0.0
        self.publisher_driver.publish(msg)
        self.get_logger().info("Turned 90 degrees")
        # Sleep for a short duration to control the rate of publishing

    def drive_length(self, length):
        # Create a Twist message
        msg = Twist()
        # Set linear and angular velocities based on the obstacle state
        msg.linear.x = 0.175
        msg.angular.z = 0.0
        # Publish the Twist message
        self.publisher_driver.publish(msg)
        time.sleep(length)
        msg.linear.x = 0.0
        self.publisher_driver.publish(msg)
        self.get_logger().info("Drove length")
        # Sleep for a short duration to control the rate of publishing

    def drive_along(self, distance_to_obstacle):
        # keeping self.right_distance at 0.3 while driving along the obstacle
        # Create a Twist message
        msg = Twist()
        # Set linear and angular velocities based on the obstacle state
        msg.linear.x = 0.2
        if self.right_distance < distance_to_obstacle:
            msg.angular.z = 0.2
        else:
            msg.angular.z = -0.2
        # Publish the Twist message
        self.publisher_driver.publish(msg)
        time.sleep(0.1)

        self.publisher_driver.publish(msg)
        # self.get_logger().info("Drove length")


def main(args=None):
    rclpy.init(args=args)
    node = obstacle_con()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.destroy_node()

    finally:
        #stop = Stopper()
        node.destroy_node()
        #stop.destroy_node()
        #rclpy.shutdown()
        print('Shutting Down Obstacle_con')

if __name__ == '__main__':
    main()

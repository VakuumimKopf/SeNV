import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from geometry_msgs.msg import Twist
from senv_interfaces.msg import Laser, Move
from senv_interfaces.action import ConTask
from senv.description import float_desc, int_desc, bool_desc, light_int_desc
import time
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup


class Park_con(Node):
    def __init__(self):
        super().__init__('park_con')
        self.get_logger().info('park_con node has been started.')

        # Parameters
        self.declare_parameter('speed_drive', 0.085, float_desc('Fahr Geschwindigkeit Park_Con'))
        self.declare_parameter('speed_turn', 0.115, float_desc("Drehgeschwindigkeit Park_Con"))

        self.turned_on = False
        self.lane_msg = Twist()
        self.right_distance = 0
        self.check_parking = False
        self.sign_detected = False
        self.park_spot = "none"
        self.last_spin = False
        self.front_spot = "empty"
        self.left_spot = "empty"
        self.right_spot = "empty"
        self.driveby = 0
        self.lane_hold_direction = 1  # or -1

        # QOS Policy Setting
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)

        # Subscribing to camera and laser topics
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
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup
        )

        self.publisher_driver = self.create_publisher(Move, 'drive', qos_profile=qos_policy)

    def execute_callback(self, goal_handle: ServerGoalHandle):

        # Get request from goal
        target = goal_handle.request.start_working
        self.get_logger().info("starting park server")

        # Execute action
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

    def laser_callback(self, msg):
        if self.turned_on is False:
            return

        # self.get_logger().info('Received message laser')
        self.right_distance = msg.right_distance
        if msg.right_distance < 0.2 and msg.right_distance != 0.0:
            self.get_logger().info("Schild rechts erkannt")
            self.sign_detected = True

        if msg.right_distance < 0.37 and msg.right_distance != 0.0:
            self.get_logger().info("etwas rechts erkannt")
            if self.driveby == 1:
                self.right_spot = "taken"
            if self.driveby == 2:
                self.front_spot = "taken"
            if self.driveby == 3:
                self.left_spot = "taken"

    def datahandler(self, info):
        self.get_logger().info("Handling park data")
        # self.laneholding_for_duration(5, 0.1, -1.0)

        self.parkingdist = 1.6  # Länge der Parklücke
        stopmsg = Twist()
        stopmsg.angular.z = 0.0
        stopmsg.linear.x = 0.0
        shrugmsg = Twist()  # shrugmessage
        shrugmsg.angular.z = 0.3
        shrugmsg.linear.x = 0.0
        # Begin Laneholding until sign is detected by laser
        self.laneholding_until_sign(0.1)  # 1publish/0.1s
        self.publisher_driver.publish(stopmsg)  # Stop for testing
        self.wait_ros2(3)  # wait next to Parksign for testing
        self.last_spin = True

        # lil shrug (very clean)
        self.get_logger().info("Shrugging")
        self.publisher_driver.publish(shrugmsg)
        self.wait_ros2(0.5)

        self.publisher_driver.publish(stopmsg)  # Stop for testing
        self.wait_ros2(3)
        self.laneholding_for_duration(3.5, 0.1, 1.0)  # drive unitl first spot

        self.publisher_driver.publish(stopmsg)  # Stop for testing
        self.wait_ros2(3)

        self.get_logger().info("Stopping infront first spot")
        self.driveby = 1  # drive along first spot
        self.laneholding_for_duration(4, 0.1, 1.0)
        self.publisher_driver.publish(stopmsg)
        self.wait_ros2(1)  # Stop for testings
        if self.right_spot == "empty":
            # self.turn90(-1.0)
            # self.turn90(-1.0)
            # take right parkingspot
            self.get_logger().info("In rechter Lücke parken")
            # self.laneholding_for_duration(2.0, 0.1, -1.0)
            self.drive_length(1.2, -1.0)
            # Distance between parkingspaces
            self.turn90(-1.0)
            self.drive_length(self.parkingdist)
            self.get_logger().info("Warten in Lücke für 5 Sekunden")
            self.wait_ros2(5)
            self.depark("backwards")
            return

        self.driveby = 2  # drive along second spot
        self.laneholding_for_duration(4, 0.1, 1.0)
        self.publisher_driver.publish(stopmsg)
        self.wait_ros2(1)  # Stop for testing
        if self.front_spot == "empty":
            # self.turn90(-1.0)
            # self.turn90(-1.0)
            # in parklücke gerade aus fahren (Anpassen von dauer notw.)
            self.get_logger().info("In mittlerer Lücke parken")
            # self.laneholding_for_duration(2.0, 0.1, -1.0)
            self.drive_length(1.2, -1.0)
            self.turn90(-1.0)
            self.drive_length(self.parkingdist)
            self.get_logger().info("Warten in Lücke für 5 Sekunden")
            self.wait_ros2(5)
            self.depark("backwards")
            return

        self.driveby = 3  # drive along third spot
        self.laneholding_for_duration(4, 0.1, 1.0)
        self.publisher_driver.publish(stopmsg)
        self.wait_ros2(1)
        if self.left_spot == "empty":
            # takeleft parkingspot
            # self.turn90(-1.0)
            # self.turn90(-1.0)
            self.get_logger().info("In linker Lücke parken")
            # self.laneholding_for_duration(2.0, 0.1, -1.0)
            self.drive_length(1.2, -1.0)
            self.turn90(-1.0)
            self.drive_length(self.parkingdist)
            self.get_logger().info("Warten in Lücke für 5 Sekunden")
            self.wait_ros2(5)
            self.depark("backwards")
            return
        
        
        '''
        self.laneholding_for_duration(9.7, 0.1)
        self.get_logger().info("Done with Laneholding")
        # self.publisher_driver.publish(stopmsg)  # Stop for testing
        # self.wait_ros2(3)  # wait next to middle spot for testing
        self.turn90(-1.0)
        self.wait_ros2(0.3)
        self.check_parking = True
        self.sign_detected = False
        self.wait_ros2(0.3)  # wait for proper laserscannercallback

        self.check_parking = False
        '''

    def depark(self, direction):
        if direction == "forwards":
            self.turn90(1.0)
            self.turn90(1.0)
            self.drive_length(self.parkingdist)
            self.turn90(-1.0)
            self.get_logger().info("Forwärts Ausgeparkt")
        elif direction == "backwards":
            self.drive_length(self.parkingdist, -1.0)
            self.turn90(1.0)
            self.get_logger().info("Backwärts Ausgeparkt")

    def turn90(self, direction):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.5 * direction
        self.publisher_driver.publish(msg)
        duration = 3

        # ROS 2-kompatibles Warten
        self.wait_ros2(duration)

        msg.angular.z = 0.0
        self.publisher_driver.publish(msg)
        self.get_logger().info("Turned 90 degrees")

    def drive_length(self, duration, direction=1.0):
        msg = Twist()

        msg.linear.x = 0.175 * direction
        msg.angular.z = 0.0
        self.publisher_driver.publish(msg)
        self.wait_ros2(duration)
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_driver.publish(msg)

    def wait_ros2(self, duration):
        """ROS 2-kompatibles Warten, ohne Callbacks zu blockieren."""
        start_time = self.get_clock().now().nanoseconds
        while (self.get_clock().now().nanoseconds - start_time) / 1e9 < duration:
            rclpy.spin_once(self, timeout_sec=0.1)

    def laneholding_for_duration(self, duration_sec, interval_sec, direction):
        self.lane_hold_direction = direction
        start_time = time.time()
        while time.time() - start_time < duration_sec:
            msg = self.lane_msg
            self.publisher_driver.publish(msg)
            self.wait_ros2(interval_sec)
        # self.lane_hold_direction = 1
        self.get_logger().info("Laneholding finished")

    def laneholding_until_sign(self, interval_sec):
        while self.sign_detected is False:
            self.get_logger().info("Lanholding until sign in Park_con")
            msg = self.lane_msg
            self.publisher_driver.publish(msg)
            self.wait_ros2(interval_sec)


def main(args=None):
    rclpy.init(args=args)
    park_con = Park_con()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(park_con, executor=executor)
    except KeyboardInterrupt:
        park_con.destroy_node()
    finally:
        park_con.destroy_node()
        print('Shutting Down park_con')


if __name__ == '__main__':
    main()
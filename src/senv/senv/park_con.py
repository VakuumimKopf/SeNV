import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from geometry_msgs.msg import Twist
from senv_interfaces.msg import Pic, Laser
from senv_interfaces.action import ConTask
from senv.description import float_desc, int_desc, bool_desc, light_int_desc
import time


class park_con(Node):
    def __init__(self):
        super().__init__('park_con')
        self.get_logger().info('park_con node has been started.')
        # Parameters
        self.declare_parameter('speed_drive', 0.085, float_desc('Fahr Geschwindigkeit Park_Con'))
        self.declare_parameter('speed_turn', 0.115, float_desc("Drehgeschwindigkeit Park_Con"))
        self.declare_parameter('light_lim', 100, light_int_desc("Helligkeits Grenzwert"))
        self.declare_parameter('middle_pix', 550, int_desc("Ausrichtungs Faktor (gering links hoch,"
                                                           "rechts Maximal Kamera Auflösung)"))
        self.declare_parameter('offset_scale', 23, int_desc("Offset Scaling"))
        self.declare_parameter('front_parkingspot_distance', 0.3, float_desc(
                                'Erkennungsdistanz Parklücke vorne'))
        self.declare_parameter('left_parkingspot_distance', 0.5, float_desc(
                                'Erkennungsdistanz Parklücke links'))
        self.declare_parameter('right_parkingspot_distance', 0.5, float_desc(
                                'Erkennungsdistanz Parklücke rechts'))
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
        self.publisher_driver = self.create_publisher(Twist, 'driving', qos_profile=qos_policy)
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
        # self.get_logger().info('Received message pic')
        speed_drive = self.get_parameter('speed_drive').get_parameter_value().double_value
        speed_turn = self.get_parameter('speed_turn').get_parameter_value().double_value
        middle_pix_ref = self.get_parameter('middle_pix').get_parameter_value().integer_value
        offset_scaling = self.get_parameter('offset_scale').get_parameter_value().integer_value
        last_spin = self.last_spin
        direction = self.lane_hold_direction
        line_pos = msg.line
        """
        reverse_offset = 20
        if direction == -1:
            middle_pix = middle_pix_ref + reverse_offset
        else:
            middle_pix = middle_pix_ref
            """
        middle_pix = middle_pix_ref
        offset = abs(line_pos-middle_pix)
        speed = 0.0
        turn = 0.0
        # neg turn speed = rechts
        # pos turn speed = links
        if (line_pos == 0 and last_spin is False):
            self.get_logger().info("turning left")
            # no white in bottom line of image
            speed = 0.0
            turn = -speed_turn/2 * direction

        elif (line_pos == 0 and last_spin is True):
            self.get_logger().info("turning right")
            speed = 0.0
            turn = speed_turn/2 * direction

        else:
            speed = speed_drive
            if direction == 1:
                # white pixel in image line / and bright_pos > boundary_left)
                if line_pos < (middle_pix):

                    # left side is brightest: turn left '+' (Drehrichtung Roboter 30)
                    turn = speed_turn * (offset/offset_scaling)
                    self.last_spin = False

                # and bright_pos < boundary_right)
                elif line_pos > (middle_pix):

                    # right side right turn '-'
                    turn = -speed_turn * (offset/offset_scaling)
                    self.last_spin = True

                else:
                    # bright pixel is in the middle
                    turn = 0.0
            else:
                # white pixel in image line / and bright_pos > boundary_left)
                if line_pos < (middle_pix):

                    turn = -speed_turn * (offset/offset_scaling)
                    self.last_spin = True

                # and bright_pos < boundary_right)
                elif line_pos > (middle_pix):

                    turn = speed_turn * (offset/offset_scaling)
                    self.last_spin = False

                else:
                    # bright pixel is in the middle
                    turn = 0.0

        self.lane_msg.linear.x = speed * direction
        self.lane_msg.angular.z = turn

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
        '''
        if self.left_spot == "empty":
            self.park_spot = "left"
        elif self.front_spot == "empty":
            self.park_spot = "front"
        elif self.right_spot == "empty":
            self.park_spot = "right"
        else:
            self.park_spot = "none"
            
            front_spot = "empty"
            left_spot = "empty"
            right_spot = "empty"

            if msg.front_distance < front_spot_dist:
                self.front_spot = "taken"
            if msg.front_left_distance < left_spot_dist:
                self.left_spot = "taken"
            if msg.front_right_distance < right_spot_dist:
                self.right_spot = "taken"
            if self.front_spot == "empty":
                self.park_spot = "front"
            elif self.left_spot == "empty":
                self.park_spot = "left"
            elif self.right_spot == "empty":
                self.park_spot = "right"
            else:
                self.park_spot = "none"
            '''

    def datahandler(self):
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

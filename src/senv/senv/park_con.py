import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.action.server import ServerGoalHandle
from senv_interfaces.msg import Pic, Laser, Move
from senv_interfaces.action import ConTask
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import time
import numpy as np
import math


class Park_con(Node):
    def __init__(self):
        super().__init__('crosswalk_con')
        self.get_logger().info('crosswalk_con node has been started.')
        # Parameters
        self.turned_on = False
        self.last_pic_msg = None
        self.right_distance = 0
        self.check_parking = False
        self.sign_detected = False
        self.park_spot = "none"
        self.last_spin = False

        self.front_spot = "empty"
        self.left_spot = "empty"
        self.right_spot = "empty"
        self.last_spot = "empty"

        self.driveby = 0
        self.lane_hold_direction = 1  # or -1

        self.parkingdist = 1.62  # width of the parking space
        self.parkinglength = 0.285  # length of thd parking space

        self.scan_area = "far"
        self.scanning = False

        self.followmsg = Move()
        self.followmsg.follow = True
        self.followmsg.speed = 0.5
        self.stopmsg = Move()
        self.stopmsg.follow = False
        self.stopmsg.speed = 0.0
        self.xposition = 0.0
        self.startxposition = 0.0
        self.yposition = 0.0
        self.startyposition = 0.0
        self.start_odom = False
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
        # msg.pose.pose.position.x
        self.subscriber_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile=qos_policy
        )

        self.crosswalk_task_server_ = ActionServer(
            self,
            ConTask,
            "park_task",
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self.publisher_driver = self.create_publisher(Move, "drive", 1)

    def execute_callback(self, goal_handle: ServerGoalHandle):

        # Get request from goal
        target = goal_handle.request.start_working
        info = goal_handle.request.info
        self.get_logger().info("starting crosswalk server")

        # Turn in action server
        self.turned_on = target

        # Execute code as long as node is turned on
        self.datahandler(info)

        self.last_pic_msg = None
        self.right_distance = 0
        self.check_parking = False
        self.sign_detected = False
        self.park_spot = "none"
        self.last_spin = False
        self.front_spot = "empty"
        self.left_spot = "empty"
        self.right_spot = "empty"
        self.last_spot = "empty"
        self.driveby = 0
        self.scan_area = ""
        self.scanning = False
        self.get_logger().info("Hand back")
        msg = Move()
        msg.follow = True
        msg.speed = 0.5
        self.followmsg = msg

        # Final Goal State
        goal_handle.succeed()

        # Result
        result = ConTask.Result()
        result.finished = True
        return result

    def cancel_callback(self, goal_handle):
        if self.turned_on is True:
            return CancelResponse.REJECT
        else:
            return CancelResponse.ACCEPT

    def pic_callback(self, msg):
        if self.turned_on is False:
            return

        self.last_pic_msg = msg

    def odom_callback(self, msg):
        if self.turned_on is False:
            return
        if self.start_odom is True:
            self.startxposition = msg.pose.pose.position.x
            self.startyposition = msg.pose.pose.position.y
            self.start_odom = False
            self.get_logger().info("Set Startpos")
        self.xposition = msg.pose.pose.position.x
        self.yposition = msg.pose.pose.position.y

        self.odomdist = np.sqrt(
            ((self.xposition - self.startxposition) ** 2) +
            ((self.yposition - self.startyposition) ** 2))

    def laser_callback(self, msg):
        if self.turned_on is False:
            return
        # self.get_logger().info('Received message laser')
        close_values = []
        taken = False
        self.right_distance = msg.right_distance
        # self.get_logger().info(f"{msg.right_distance}")
        ranges = msg.raw
        right_sector = []
        right_sector = ranges[536:544]

        # Filter: Werte unter 0.2 und ungleich 0.0
        close_values = [r for r in right_sector if 0.0 < r < 0.2]

        if len(close_values) > 0:
            self.sign_detected = True
            self.get_logger().info("Hallo in sign_detect right")

        # 70 110, 63 123, 58 138
        i = 0
        far_dist = 0.37
        middle_dist = 0.3
        close_dist = 0.22
        corner_dist = 0.41
        far_start_angle = 720 - 146
        middle_start_angle = 720 - 124
        close_start_angle = 720 - 114

        if self.scanning:
            self.get_logger().info(f"Scanning Parkingspot {self.driveby}, {self.scan_area}")
            if ranges[580] < corner_dist or ranges[500] < corner_dist:
                taken = True
                self.get_logger().info("Object in the Corner")
            while i < 68:
                if ranges[far_start_angle-i] < far_dist:
                    taken = True
                    self.get_logger().info(f"Taken Parkingspot {self.driveby}, {self.scan_area}, angle:{-i+far_start_angle} , distance {ranges[-i+far_start_angle]}")
                i += 1
            self.scan_area = "middle"
            i = 0
            self.get_logger().info(f"Scanning Parkingspot {self.driveby}, {self.scan_area}")
            while i < 96:
                if ranges[middle_start_angle-i] < middle_dist:
                    taken = True
                    self.get_logger().info(f"Taken Parkingspot {self.driveby}, {self.scan_area}, angle:{-i+far_start_angle} , distance {ranges[-i+far_start_angle]}")
                i += 1
            self.scan_area = "close"
            i = 0
            self.get_logger().info(f"Scanning Parkingspot {self.driveby}, {self.scan_area}")
            while i < 116:
                if ranges[close_start_angle-i] < close_dist:
                    taken = True
                    self.get_logger().info(f"Taken Parkingspot {self.driveby}, {self.scan_area}, angle:{-i+far_start_angle} , distance {ranges[-i+far_start_angle]}")
                i += 1
            self.scan = "far"
            i = 0

        if taken is True:
            self.get_logger().info(f"Etwas in parklücke {self.driveby} erkannt")
            if self.driveby == 1:
                self.right_spot = "taken"
            if self.driveby == 2:
                self.front_spot = "taken"
            if self.driveby == 3:
                self.left_spot = "taken"
            if self.driveby == 4:
                self.last_spot = "taken"
        self.scanning = False
        # self.get_logger().info("End of Laserscan in Park")

    def datahandler(self, info):
        self.get_logger().info("Handling park data")
        # self.laneholding_for_duration(5, 0.1, -1.0)
        # Begin Laneholding until sign is detected by laser
        self.laneholding_until_sign(0.1)  # 1publish/0.1s
        self.publisher_driver.publish(self.stopmsg)  # Stop for testing
        self.wait_ros2(1)  # wait next to Parksign for testing
        # self.last_spin = True

        # self.laneholding_for_duration(5.0, 0.1, 1.0)  # drive unitl middle of first spot
        self.drive_length_odom(0.39)

        self.publisher_driver.publish(self.stopmsg)  # Stop for testing
        self.get_logger().info("Stopping at first spot")
        # self.wait_ros2(3)

        # self.laneholding_for_duration(4.0, 0.1, 1.0)
        # self.publisher_driver.publish(self.stopmsg)

        self.driveby = 1  # at first spot
        self.scanning = True  # start scanning
        self.wait_ros2(1)  # Stop for testings

        if self.right_spot == "empty":
            # self.turn90(-1.0)
            # self.turn90(-1.0)
            # take right parkingspot
            self.get_logger().info("In rechter Lücke parken")
            # self.laneholding_for_duration(2.0, 0.1, -1.0)
            # self.drive_length(1.2, -1.0)
            # Distance between parkingspaces
            self.turn90(-1.0)
            # self.wait_ros2(1)
            self.drive_length(self.parkingdist)
            self.get_logger().info("Warten in Lücke für 5 Sekunden")
            self.wait_ros2(5)
            self.depark("backwards")
            return

        self.drive_length_odom(self.parkinglength)
        self.publisher_driver.publish(self.stopmsg)
        self.driveby = 2  # drive along second spot
        self.scanning = True
        self.wait_ros2(1)  # Stop for testing
        if self.front_spot == "empty":
            # self.turn90(-1.0)
            # self.turn90(-1.0)
            # in parklücke gerade aus fahren (Anpassen von dauer notw.)
            self.get_logger().info("In mittlerer Lücke parken")
            # self.laneholding_for_duration(2.0, 0.1, -1.0)
            # self.drive_length(1.2, -1.0)
            self.turn90(-1.0)
            self.drive_length(self.parkingdist)
            self.get_logger().info("Warten in Lücke für 5 Sekunden")
            self.wait_ros2(5)
            self.depark("backwards")
            return

        self.drive_length_odom(self.parkinglength)
        self.publisher_driver.publish(self.stopmsg)
        self.driveby = 3  # drive along third spot
        self.scanning = True
        self.wait_ros2(1)
        if self.left_spot == "empty":
            # takeleft parkingspot
            # self.turn90(-1.0)
            # self.turn90(-1.0)
            self.get_logger().info("In linker Lücke parken")
            # self.laneholding_for_duration(2.0, 0.1, -1.0)
            # self.drive_length(1.2, -1.0)
            self.turn90(-1.0)
            self.drive_length(self.parkingdist)
            self.get_logger().info("Warten in Lücke für 5 Sekunden")
            self.wait_ros2(5)
            self.depark("backwards")
            return
        
        self.drive_length_odom(self.parkinglength)
        self.publisher_driver.publish(self.stopmsg)
        self.driveby = 4  # drive along third spot
        self.scanning = True
        self.wait_ros2(1)
        if self.last_spot == "empty":
            # takeleft parkingspot
            # self.turn90(-1.0)
            # self.turn90(-1.0)
            self.get_logger().info("In letzter (4) Lücke parken")
            # self.laneholding_for_duration(2.0, 0.1, -1.0)
            # self.drive_length(1.2, -1.0)
            self.turn90(-1.0)
            self.drive_length(self.parkingdist)
            self.get_logger().info("Warten in Lücke für 5 Sekunden")
            self.wait_ros2(5)
            self.depark("backwards")
            return

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
        self.publisher_driver.publish(self.stopmsg)
        msg = self.followmsg
        msg.speed = 1.0
        msg.turn = 0
        msg.override = True
        msg.speed_o = 0.0
        msg.turn_o = 0.5 * direction
        self.publisher_driver.publish(msg)
        duration = 3.2

        # ROS 2-kompatibles Warten
        self.wait_ros2(duration)

        self.publisher_driver.publish(self.stopmsg)
        self.get_logger().info("Turned 90 degrees")

    def drive_length(self, duration, direction=1.0):
        self.publisher_driver.publish(self.stopmsg)
        msg = self.followmsg
        msg.speed = 1.0
        msg.override = True
        msg.speed_o = 0.175 * direction
        msg.turn_o = 0.0
        self.publisher_driver.publish(msg)
        self.wait_ros2(duration)
        self.publisher_driver.publish(self.stopmsg)

    def drive_length_odom(self, distance):
        self.get_logger().info("Driving distance with Odometry")
        self.publisher_driver.publish(self.stopmsg)
        self.start_odom = True
        msg = self.followmsg
        self.wait_ros2(2)
        while self.odomdist < distance:
            self.publisher_driver.publish(msg)
            self.wait_ros2(0.1)
            self.get_logger().info(f"Waiting for odom {distance - self.odomdist} ")

    def laneholding_for_duration(self, duration_sec, interval_sec, direction):
        self.lane_hold_direction = direction
        start_time = time.time()
        while time.time() - start_time < duration_sec:
            msg = self.followmsg
            self.publisher_driver.publish(msg)
            self.wait_ros2(interval_sec)
        # self.lane_hold_direction = 1
        self.get_logger().info("Laneholding finished")

    def quaternion_to_yaw(q):
        """
        Wandelt ein Quaternion (geometry_msgs.msg.Quaternion) in einen Yaw-Winkel (in Radiant) um.
        """
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def normalize_angle(angle):
        # Normalisiere Winkel auf [-pi, pi]
        return math.atan2(math.sin(angle), math.cos(angle))

    def turn90_odom(self, direction):
        self.turn_direction = direction
        self.start_odom_turn = True
        self.yaw_start = None
        self.yaw_current = None
        self.turned_angle = 0.0

        msg = self.followmsg
        msg.speed = 0.0
        msg.turn_o = 0.5 * direction
        msg.override = True
        self.publisher_driver.publish(msg)

        # Starte Timer, der alle 100 ms prüft
        self.create_timer(0.1, self.check_turn90)

    def check_turn90(self):
        if not self.start_odom_turn or self.yaw_start is None:
            return

        delta = self.normalize_angle(self.yaw_current - self.yaw_start)
        self.turned_angle = abs(delta)

        self.get_logger().info(f"Turning: {math.degrees(self.turned_angle):.1f}°")

        if self.turned_angle >= math.radians(90):
            self.publisher_driver.publish(self.stopmsg)
            self.start_odom_turn = False
            self.get_logger().info("Turned 90 degrees (odom)")
            
    '''
    def turn90_odom(self, direction):
        """
        Dreht den Roboter um 90° basierend auf Odometrie.
        direction = +1 für links, -1 für rechts
        """
        self.publisher_driver.publish(self.stopmsg)

        msg = self.followmsg
        msg.speed = 0.0
        msg.turn = 0
        msg.override = True
        msg.speed_o = 0.0
        msg.turn_o = 0.5 * direction  # je nach Drehrate ggf. anpassen

        # Starte Drehung
        self.start_odom_turn = True
        self.yaw_start = None
        self.yaw_current = None
        self.turned_angle = 0.0

        self.publisher_driver.publish(msg)

        while abs(self.turned_angle) < math.radians(90):
            self.wait_ros2(0.05)
            if self.yaw_start is not None:
                delta = self.normalize_angle(self.yaw_current - self.yaw_start)
                self.turned_angle = abs(delta)
                self.get_logger().info(f"Turning: {math.degrees(self.turned_angle):.1f}°")

        self.publisher_driver.publish(self.stopmsg)
        self.get_logger().info("Turned 90 degrees (odom)")
        '''
    
    def laneholding_until_sign(self, interval_sec):
        self.get_logger().info("Lanholding until sign in Park_con")
        while self.sign_detected is False:
            msg = self.followmsg
            self.publisher_driver.publish(msg)
            self.wait_ros2(interval_sec)
            self.get_logger().info("Waiting for sign to the right")

    def wait_ros2(self, duration: float):
        i = duration * 100
        while i > 0:
            time.sleep(0.01)
            i -= 1


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

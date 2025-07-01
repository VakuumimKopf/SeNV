import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import signal
import sys
import threading
import time
from senv_interfaces.msg import Move, Lane


class Driver(Node):
    def __init__(self):
        super().__init__('driver')

        self.get_logger().info("Driver gestartet")
        self.get_logger().info('Driver was initialized')

        self.last_drive_msg = None
        self.last_lane_msg = None
        self.turned_on = True

        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)

        # Topic subscription
        self.drive_subscriber = self.create_subscription(
            Move,
            'drive',
            self.driving_callback,
            qos_profile=qos_policy
        )
        self.drive_subscriber  # prevent unused variable warning

        self.lane_subscriber = self.create_subscription(
            Lane,
            'lane',
            self.lane_callback,
            qos_profile=qos_policy
        )
        self.lane_subscriber  # prevent unused variable warning

        # Stop-Flag
        self.stop_requested = False

        #  Publisher
        self.publisher_driver = self.create_publisher(Twist, 'cmd_vel', 1)

        # Signal-Handler korrekt registrieren
        signal.signal(signal.SIGINT, self.handle_shutdown_signal)
        signal.signal(signal.SIGTERM, self.handle_shutdown_signal)

        # create timers for data handling
        self.build_drive_timer_period = 0.1
        self.build_drive_timer = self.create_timer(
            self.build_drive_timer_period, self.build_drive_msg)

    def driving_callback(self, msg):

        self.last_drive_msg = msg

    def lane_callback(self, msg):

        self.last_lane_msg = msg

    # Translate the Move msg into a Twist msg
    def build_drive_msg(self):

        if self.last_drive_msg is None or self.last_lane_msg is None:
            return
        
        if self.turned_on is False:
            return

        last_drive_msg = self.last_drive_msg
        last_lane_msg = self.last_lane_msg

        speed = 0.0
        turn = 0.0

        # Check if the con_node wants to use custom speed and turn values
        if last_drive_msg.override is True:
            turn = last_drive_msg.turn_o
            speed = last_drive_msg.speed_o

        # Check if the con_node wants to use the lane_detect funcionality
        elif last_drive_msg.follow is True:

            middle = 320  # dont change
            area_short = 15
            area_long = 35

            lines = [last_lane_msg.gline_a, last_lane_msg.gline_b, last_lane_msg.gline_c]

            if all(v == 0 for v in lines[0]):
                return

            #  Calculate middle of the line
            x1 = lines[0][0]
            x2 = lines[0][2]
            x_d = (x1 + x2) / 2

            #  Determine Turn Value
            if x_d < middle - area_short:
                if x_d < middle - area_long:
                    turn = 0.35
                else:
                    turn = 0.30
            elif x_d > middle + area_short:
                if x_d > middle + area_long:
                    turn = -0.35
                else:
                    turn = -0.30
            else:
                turn = 0.0

            speed = 0.2*(1-abs(turn))

        else:
            if last_drive_msg.turn == 1:
                speed = 0.0
                turn = 0.1

        #  Build Twist msg with calculated turn and speed
        msg = Twist()
        #  Add the speed requested
        msg.angular.z = turn * self.last_drive_msg.speed
        msg.linear.x = speed * self.last_drive_msg.speed

        # self.get_logger().info(str(msg.angular.z))
        # self.get_logger().info(str(msg.linear.x))

        self.publisher_driver.publish(msg)

    def publish_stop(self):
        if rclpy.ok():
            self.turned_on = False
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher_driver.publish(msg)
            self.get_logger().info('Stop-Befehl gesendet')
        else:
            self.get_logger().warn('Konnte Stop nicht senden – rclpy nicht mehr OK')

    def handle_shutdown_signal(self, signum, frame):
        # Nur einmal reagieren
        if self.stop_requested:
            return
        self.stop_requested = True

        self.get_logger().warn(f"Signal {signum} empfangen – Stoppe Roboter...")

        try:
            self.publish_stop()
            # ROS braucht Zeit, um das Publishing zu verarbeiten
            rclpy.spin_once(self, timeout_sec=0.2)
            time.sleep(0.2)
        except Exception as e:
            self.get_logger().error(f'Fehler beim Stoppen: {e}')
        finally:
            # Erst jetzt: Node zerstören und ROS beenden
            self.get_logger().info('Beende Node...')
            self.destroy_node()
            rclpy.shutdown()
            sys.exit(0)


def main():
    rclpy.init()
    node = Driver()

    # spin() in Thread, damit Signale verarbeitet werden können
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
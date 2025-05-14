'''import rclpy
import rclpy.executors
import rclpy.node
from std_msgs.msg import Int16
from senv.stopper import Stopper
from geometry_msgs.msg import Twist
from enum import Enum
import time
import signal
from rclpy.executors import ExternalShutdownException


class Driver(rclpy.node.Node):
    def __init__(self):
        super().__init__('driver')
        self.get_logger().info('Driver was initialized')
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)

        # Topic subscription
        self.subscriber = self.create_subscription(
            Twist,
            'driving',
            self.driving_callback,
            qos_profile=qos_policy
        )
        self.subscriber  # prevent unused variable warning

        # Topic to publish
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.stoplisher = self.create_publisher(Twist, 'cmd_vel', 10)
        #rclpy.get_default_context().on_shutdown(self.send_stop_driver)
        #rclpy.add_on_shutdown(self.send_stop)
        signal.signal(signal.SIGINT, self.send_stop_driver)


    def driving_callback(self, msg: Twist):
        # Process the incoming message
        self.get_logger().info(
            "Driver recieved data: " + str(msg.linear.x) + " : " + str(msg.angular.z)
            )

        # Debuga algorithm here

        # Send final msg here
        self.publisher.publish(msg)

    def publish_stop (self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.get_logger().info('Sending Stop in Driver')
        self.stoplisher.publish(msg)
        print('Sending Stop in Driver')
    
    def send_stop_driver(self, signum, frame):
        #node.get_logger().info('Driver Received Stop SIGINT')
        print('Driver Received Stop SIGINT')
        #if rclpy.ok() and node is not None:
        try:
            self.publish_stop()
        except Exception as e:
            #node.get_logger().info('Driver Received Stop SIGINT')
            print(f'Exception in Stop: {e}')
        self.destroy_node()
        rclpy.shutdown


def send_stop(node):
    #node.get_logger().info('Driver Received Stop SIGINT')
    print('Driver Received Stop SIGINT')
    #if rclpy.ok() and node is not None:
    try:
        node.publish_stop()
    except Exception as e:
        #node.get_logger().info('Driver Received Stop SIGINT')
        print(f'Exception in Stop: {e}')
    node.destroy_node()
    rclpy.shutdown

def main(args=None):

    
    rclpy.init(args=args)
    node = Driver()
    #stop = None
    #rclpy.get_default_context().on_shutdown(node.send_stop)
    #signal.signal(signal.SIGINT, lambda signum, frame: send_stop(node, signum, frame))
    rclpy.get_default_context().on_shutdown(lambda: send_stop(node))
    try:
        rclpy.spin(node)
    
    except KeyboardInterrupt:
        #node.publish_stop()
        print('Driver Node Shut down in Except')
        #stop = Stopper()
        
    finally:
        #if stop is not None:
        #    stop.destroy_node()
        #node.publish_stop()
        node.destroy_node()
        #rclpy.shutdown()
        #print('Shutting Down Driver')


if __name__ == '__main__':
    main()
'''

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import signal
import sys
import threading
import time


class Driver(Node):
    def __init__(self):
        super().__init__('driver')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.get_logger().info("Driver gestartet")
        self.get_logger().info('Driver was initialized')
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)

        # Topic subscription
        self.subscriber = self.create_subscription(
            Twist,
            'driving',
            self.driving_callback,
            qos_profile=qos_policy
        )
        self.subscriber  # prevent unused variable warning
        # Stop-Flag
        self.stop_requested = False

        # Signal-Handler korrekt registrieren
        signal.signal(signal.SIGINT, self.handle_shutdown_signal)
        signal.signal(signal.SIGTERM, self.handle_shutdown_signal)

    def driving_callback(self, msg: Twist):
        # Process the incoming message
        self.get_logger().info(
            "Driver recieved data: " + str(msg.linear.x) + " : " + str(msg.angular.z)
            )

        # Debuga algorithm here

        # Send final msg here
        self.publisher.publish(msg)

    def publish_stop(self):
        if rclpy.ok():
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher.publish(msg)
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
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
from senv_interfaces import Pic

class camera(Node):
    def __init__(self):
        super().__init__('trafficlight_detector')

        #set Parameters 
        self.bridge = CvBridge()
        self.status = 'unknown'

        #self.img_row = np.array([0, 64, 128, 192, 255], dtype=np.uint8) # Beispiel
        self.img_row = np.random.randint(0, 256, 640, dtype=np.uint8)

        # definition of the QoS in order to receive data despite WiFi
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        
        # create subscribers for image data with changed qos
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            qos_profile=qos_policy)
        self.subscription  # prevent unused variable warning

        #create topic to publish data
        self.publisher_ = self.create_publisher(Pic, 'pic', 1)

        #create timers for data handling 
        self.line_timer_period = 2.0
        self.line_timer = self.create_timer(self.line_timer_period, self.line_detection)

        self.sign_timer_period = 2.0
        self.sign_timer = self.create_timer(self.sign_timer_period, self.sign_detection)

    def image_callback(self, data):

        try:
            # Bild von ROS zu OpenCV umwandeln
            img_cv = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Fehler beim Konvertieren: {e}')
            return

        # Bild auf kleinere Größe skalieren
        img_resized = cv2.resize(img_cv, (320, 240))
        hsv = cv2.cvtColor(img_resized, cv2.COLOR_BGR2HSV)



        # Farbgrenzen für rot
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([179, 255, 255])

        # Farbgrenzen für grün
        lower_green = np.array([40, 70, 70])
        upper_green = np.array([90, 255, 255])

        # Masken erstellen
        red_mask = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)
        green_mask = cv2.inRange(hsv, lower_green, upper_green)

        red_count = cv2.countNonZero(red_mask)
        green_count = cv2.countNonZero(green_mask)

        self.status = 'unknown'
        if red_count > green_count and red_count > 200:
            self.status = 'red'
        elif green_count > red_count and green_count > 200:
            self.status = 'green'

        # Nachricht veröffentlichen
        msg = String()
        msg.data = self.status
        self.publisher_.publish(msg)
        self.get_logger().info(f'Ampelstatus: {self.status}')
    
    def line_detection(self):
        self.get_logger().info("line_detection gestartet")

    def sign_detection(self):
        self.get_logger().info("sign_detection gestartet")


def main(args=None):
    rclpy.init(args=args)
    node = camera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()
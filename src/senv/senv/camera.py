import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
from senv_interfaces.msg import Pic

class camera(Node):
    def __init__(self):
        super().__init__('camera')

        #set Parameters 
        self.bridge = CvBridge()
        self.status = 'unknown'
        self.hsv = 0

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
        self.line_timer_period = 0.5
        self.line_timer = self.create_timer(self.line_timer_period, self.line_detection)

        self.status_sign = ""
        self.sign_timer_period = 1.0
        self.sign_timer = self.create_timer(self.sign_timer_period, self.sign_detection)

    # raw data formating routine 
    def image_callback(self, data):

        try:
            # Bild von ROS zu OpenCV umwandeln
            img_cv = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Fehler beim Konvertieren: {e}')
            return
        img_resized = cv2.resize(img_cv, (320, 240))
        height, width, _ = img_resized.shape
        x_start = width * 2 // 3
        x_end = width
        y_start = height // 3
        y_end = height * 2 // 3

        roi = img_resized[y_start:y_end, x_start:x_end]
        # Bild auf kleinere Größe skalieren
        self.hsv = cv2.cvtColor(img_resized, cv2.COLOR_BGR2HSV)
    
    # line detection in formated data
    def line_detection(self):
        self.get_logger().info("line_detection gestartet")


        # Nachricht veröffentlichen
        msg = Pic()
        msg.sign = self.status
        #self.get_logger().info(msg.sign)
        msg.line = 0
        #self.get_logger().info(msg.line)

        self.publisher_.publish(msg)

    # sign detection in fomated data
    def sign_detection(self):
        self.get_logger().info("sign_detection gestartet")
        min_area = 50 # Minimale fläche für rotes licht
        if self.hsv == 0:
            self.status = "unknown"

        else:
            # Farbgrenzen für rot
            lower_red1 = np.array([0, 100, 100])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([160, 100, 100])
            upper_red2 = np.array([179, 255, 255])
            hsv = self.hsv

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(mask1, mask2)

        # Optionale Rauschunterdrückung
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

        # Konturen finden
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Überprüfen, ob ein Fleck groß genug ist
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > min_area:
                self.status = 'red'

            
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

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
        
        self.last_spin = False # False == gegen UHrzeigersinn True==mit Uhrzeigersinn
        
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

        # Eingang roh daten werden gefiltert und es wird der ein Int Array 
        # gespeichert unter self.img_row abgelegt um dann in line_detection gepublisht zu werden  

        img_cv = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding = 'passthrough')

        # convert image to grayscale
        img_gray = cv2.cvtColor(img_cv, cv2.COLOR_BGR2GRAY)

        # get image size
        height, width = img_gray.shape[:2]
        #self.get_logger().info(f"Image Höhe: {width} ")
        # get the lowest row from image
        img_row = img_gray[height-9,:]
        self.img_row = img_row 

        # show image
        #cv2.imshow("IMG", img_gray)
        #cv2.imshow("IMG_ROW", img_row)
        #cv2.waitKey(1)
    
    # line detection in formated data
    def line_detection(self):
        self.get_logger().info("line_detection gestartet")

        # Nachricht veröffentlichen
        msg = Pic()
        msg.sign = self.status
        #self.get_logger().info(msg.sign)
        msg.line = self.img_row
        #self.get_logger().info(msg.line)

        self.publisher_.publish(msg)

    # sign detection in fomated data
    def sign_detection(self):
        self.get_logger().info("sign_detection gestartet")

        #Function for signs - string for Outputs("")

        #Function for trafficlights - string for Output("red light", "green light")


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

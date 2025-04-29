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
        self.declare_parameter('boundary_left', 100) # 200 für 640px, 100 für 320
        self.declare_parameter('boundary_right', 630) # 440 für 640px, 220 für 320px
        self.declare_parameter('light_lim', 100)
        self.bridge = CvBridge()
        self.status = ""
        self.line_pos = 0
        self.hsv = np.array([])
        self.waitingforgreen = False
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

        self.status = ""
        self.sign_timer_period = 0.5
        self.sign_timer = self.create_timer(self.sign_timer_period, self.sign_detection)

    # raw data formating routine 
    def image_callback(self, data):
        boundary_left = self.get_parameter('boundary_left').get_parameter_value().integer_value
        boundary_right = self.get_parameter('boundary_right').get_parameter_value().integer_value
        light_lim = self.get_parameter('light_lim').get_parameter_value().integer_value

        # Eingang roh daten werden gefiltert und es wird der ein Int Array 
        # gespeichert unter self.img_row abgelegt um dann in line_detection gepublisht zu werden  

        img_cv = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding = 'passthrough')

        # convert image to grayscale
        height, width, _ = img_cv.shape
        img_gray = cv2.cvtColor(img_cv, cv2.COLOR_BGR2GRAY)
        img_row = img_gray[height-9,:]

        # Speichere HSV für farbanalyse in sign detection


        # Bereich: rechtes Drittel, mittleres Drittel vertikal
        x_start = width * 2 // 3
        x_end = width
        y_start = height // 3
        y_end = height * 2 // 3

        roi = img_cv[y_start:y_end, x_start:x_end]

        if roi.size == 0:
            self.hsv = None
        else:
            self.hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Formating data 
        img_row = img_row[boundary_left:boundary_right]
        
        # 20 greatest items sorted
        img_row_sorted = np.argsort(img_row)[-20:]

        # Values of 20 greatest items
        largest_values = img_row[img_row_sorted]

        # Index des mittleren Wertes der 20 größten im Teil-Array
        middle_index_in_subset = np.argsort(largest_values)[len(largest_values) // 2]

        # Index des mittleren Wertes im Original-Array
        middle_index_in_original = img_row_sorted[middle_index_in_subset]
        self.line_pos = 0
        # Hellstes Element 
        brightest = max(img_row)
        if brightest > light_lim:
            self.line_pos = middle_index_in_original + boundary_left
        # show image
        #cv2.imshow("IMG_ROW", img_row)
        cv2.imshow("IMG", img_cv)
        cv2.waitKey(1)
    
    # line detection in formated data
    def line_detection(self):
        #self.get_logger().info("line_detection gestartet")

        # Nachricht veröffentlichen
        msg = Pic()
        if self.waitingforgreen == True:
            self.status = "red light"
        msg.sign = self.status
        #self.get_logger().info(msg.sign)
        msg.line = int(self.line_pos)
        #self.get_logger().info(msg.line)

        self.publisher_.publish(msg)

    # sign detection in fomated data
    def sign_detection(self):
        #self.get_logger().info("sign_detection gestartet")
        min_area = 30    # Minimale Fläche (z. B. Ampellicht)
        max_area = 200 
        hsv = self.hsv
        self.status = ""
        if (hsv.size == 0):
            self.status = ""
            return
        #Function for signs - string for Outputs("")
        min_area = 30 # Minimale fläche für rotes licht
        # Farbgrenzen für rot
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([179, 255, 255])

        # Farbgrenzen für Grün (Ampelgrün)
        lower_green = np.array([40, 50, 50])
        upper_green = np.array([90, 255, 255])


        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(mask1, mask2)
        # Optionale Rauschunterdrückung
        '''#red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        #red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

            # Konturen finden
        #contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > min_area:
                self.status = 'red light'
                # Grüne Fläche suchen'''
        
        green_mask = cv2.inRange(hsv, lower_green, upper_green)

        # Rauschunterdrückung
        kernel = np.ones((5, 5), np.uint8)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)

        # Konturen analysieren
        contours_red, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_green, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        max_red_area = 0
        max_green_area = 0

        # Rote Fläche suchen
        for contour in contours_red:
            area = cv2.contourArea(contour)
            if min_area < area < max_area:
                max_red_area = max(max_red_area, area)
                # Überprüfen, ob ein Fleck groß genug ist
            

        for contour in contours_green:
            area = cv2.contourArea(contour)
            if min_area < area < max_area:
                max_green_area = max(max_green_area, area)

        # Statuslogik
        if max_red_area > 0:
            self.status = 'red light'
            self.waitingforgreen = True
        if max_green_area >= max_red_area and max_green_area > 0:
            self.status = 'green light'
            self.waitingforgreen = False

        cv2.imshow("IMG_red", red_mask)
        cv2.waitKey(1)
        
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
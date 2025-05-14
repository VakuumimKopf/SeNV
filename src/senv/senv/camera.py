import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2 as cv2
import numpy as np

from senv_interfaces.msg import Pic

# Import ultralytics for sign detection with "pip install ultralytics"
from ultralytics import YOLO
# load model
# change the path to your model found in senv as best.pt
model = YOLO("/home/lennart/ros2_ws/src/senv/best.pt")  # z. B. "./yolo_model/best.pt"


class camera(Node):
    def __init__(self):
        super().__init__('camera')

        # set Parameters
        self.declare_parameter('boundary_left', 100)
        # 200 für 640px, 100 für 320
        self.declare_parameter('boundary_right', 630)
        # 440 für 640px, 220 für 320px
        self.declare_parameter('light_lim', 100)

        self.bridge = CvBridge()
        self.status = ""

        self.line_pos = 0
        self.hsv = np.array([])
        self.waitingforgreen = False
        self.img_row = np.random.randint(0, 256, 640, dtype=np.uint8)

        # definition of the QoS in order to receive data despite WiFi
        qos_policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)

        # False is gegen UHrzeigersinn, True is mit Uhrzeigersinn
        self.last_spin = False

        # create subscribers for image data with changed qos
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            qos_profile=qos_policy)
        self.subscription  # prevent unused variable warning

        # create topic to publish data
        self.publisher_ = self.create_publisher(Pic, 'pic', 1)

        # create timers for data handling
        self.line_timer_period = 0.1
        self.line_timer = self.create_timer(
            self.line_timer_period, self.line_detection)

        self.status = ""
        self.sign_timer_period = 0.5
        self.sign_timer = self.create_timer(
            self.sign_timer_period, self.sign_handler)

        # create variables for sign detection
        self.img = []
        # self.start_time = time.time()  # Start time for image saving (timestamp for easier differentiation)

    # raw data formating routine
    def image_callback(self, data):

        # Get needed parameters
        boundary_left = self.get_parameter('boundary_left').get_parameter_value().integer_value
        boundary_right = self.get_parameter('boundary_right').get_parameter_value().integer_value
        light_lim = self.get_parameter('light_lim').get_parameter_value().integer_value

        # Eingang roh daten werden gefiltert und es wird der ein Int Array
        # gespeichert unter self.img_row abgelegt um dann in line_detection gepublisht zu werden

        img_cv = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding='passthrough')

        # just the image for later use (in sign_identification)
        self.img = data

        # convert image to grayscale
        height, width, _ = img_cv.shape
        img_gray = cv2.cvtColor(img_cv, cv2.COLOR_BGR2GRAY)
        img_row = img_gray[height-9, :]

        # Speichere HSV für farbanalyse in sign detection

        # Bereich: rechtes Drittel, mittleres Drittel vertikal
        x_start = width * 2 // 3
        x_end = width
        y_start = height // 4
        y_end = height // 2

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
        # cv2.imshow("IMG_ROW", img_row)
        cv2.imshow("IMG", img_cv)
        cv2.waitKey(1)

    # line detection in formated data
    def line_detection(self):
        # self.get_logger().info("line_detection gestartet")

        # Nachricht veröffentlichen
        msg = Pic()
        if self.waitingforgreen is True:
            self.status = "red light"
        msg.sign = self.status
        # self.get_logger().info(msg.sign)
        msg.line = int(self.line_pos)
        # self.get_logger().info(msg.line)

        self.publisher_.publish(msg)

    # sign detection in fomated data
    def sign_handler(self):

        light_status = self.light_detection()

        # return the first value of the detected_labels array in sign_identification
        sign_status = self.sign_identification()

        if (sign_status != ""):
            self.status = sign_status

        elif (light_status != ""):
            self.status = light_status

        else:
            self.status = ""

    def light_detection(self):
        # Versioncheck for cv2
        # print(cv2.__version__)

        min_area = 30   # Minimale Fläche (z. B. Ampellicht)
        max_area = 2000
        hsv = self.hsv
        status = ""
        if (hsv.size == 0):
            status = ""
            return
        # Function for signs - string for Outputs("")
        min_area = 30  # Minimale fläche für rotes licht
        # Farbgrenzen für rot
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([179, 255, 255])

        # Farbgrenzen für Grün (Ampelgrün)
        lower_green = np.array([50, 150, 50])
        upper_green = np.array([85, 255, 255])

        # Red mask creation
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(mask1, mask2)

        # Green mask creation
        green_mask = cv2.inRange(hsv, lower_green, upper_green)

        # Rauschunterdrückung
        kernel = np.ones((5, 5), np.uint8)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)

        # Konturen analysieren
        # added _ to ignore the second return value
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
            status = 'red light'
            self.waitingforgreen = True
        if max_green_area >= max_red_area and max_green_area > 0:
            status = 'green light'
            self.waitingforgreen = False

        cv2.imshow("IMG_red", red_mask)
        cv2.imshow("IMG_green", green_mask)
        cv2.waitKey(1)

        return status

    # Use following function to create dataset of raw images
    """
    def image_saving(self):
        # Save the image to a file.

        # Change to directory of your choice
        # Change the directory and sign as needed
        # 0 output
        # directory = "/home/lennart/ros2_ws/src/senv/Images/left"
        # sign = "left"
        # 1 output
        # directory = "/home/lennart/ros2_ws/src/senv/Images/right"
        # sign = "right"
        # 2 output
        # directory = "/home/lennart/ros2_ws/src/senv/Images/straight"
        # sign = "straight"
        # 3 output
        # directory = "/home/lennart/ros2_ws/src/senv/Images/crosswalk"
        # sign = "crosswalk"
        # 4 output
        directory = "/home/lennart/ros2_ws/src/senv/Images/park"
        sign = "park"

        # Get the current time
        current_time = self.get_clock().now().to_msg()

        # Format the time as a string
        time_str = f"{current_time.sec}_{current_time.nanosec}"

        # Create the filename
        filename = f"image_{sign}_{time_str}.jpg"

        # Convert the image to a format that can be saved
        img_cv = self.bridge.compressed_imgmsg_to_cv2(self.img, desired_encoding='passthrough')
        os.chdir(directory)
        # Save the image
        cv2.imwrite(filename, img_cv)

        print("After saving image:")

        print(f'Successfully saved {time_str}')

        elapsed_time = time.time() - self.start_time
        if elapsed_time > 32:
            self.get_logger().info("32 Sekunden erreicht – Programm wird beendet.")
            rclpy.shutdown()
        return ""
    """


# sign identification

    def sign_identification(self):
        """
        Predict the class of an image using a pre-trained model.
        """
        # get the image from the camera as np.array
        image = self.bridge.compressed_imgmsg_to_cv2(self.img, desired_encoding='passthrough')
        # Prediction (Inference)
        results = model(image, verbose=False)  # kann auch save=True sein

        # classlist from model (the names must match your `data.yaml`)
        class_names = model.names

        # IDs of the detected classes (e.g. 0, 1, 2 …)
        class_ids = results[0].boxes.cls.cpu().numpy().astype(int)

        # Change labels to names
        detected_labels = [class_names[i] for i in class_ids]

        # when there is no sign detected, the list is empty
        if detected_labels == []:
            # self.get_logger().info("No sign detected")
            return ""

        """
        # DEBUG printet alle erkannten Schilder und zeigt Rahmen um diese

        print("Erkannte Schilder:", detected_labels)
        # Annotiertes Bild aus dem Ergebnis holen
        annotated_image = results[0].plot()  # Gibt ein NumPy-Array zurück

        # Bild anzeigen mit OpenCV
        cv2.imshow("YOLOv8-Erkennung", annotated_image)
        cv2.waitKey(1)
        """

        # return first detected label
        return detected_labels[0]


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

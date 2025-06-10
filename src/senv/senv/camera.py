import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2 as cv2
import numpy as np
from senv_interfaces.msg import Pic
from senv.description import light_int_desc

# Import ultralytics for sign detection with "pip install ultralytics"
from ultralytics import YOLO

# load model
model = YOLO("./yolo_model/best.pt")


class camera(Node):
    def __init__(self):
        super().__init__('camera')

        # Dynmaische Paramter für RotFarbgrenzen 1
        self.declare_parameter('low1red_color', 0, light_int_desc("Rot Farbwert Untere Grenze 1"))
        self.declare_parameter('low1red_sat', 100, light_int_desc("Rot Saturation Untere Grenze 1"))
        self.declare_parameter('low1red_alpha', 100, light_int_desc("Rot Helligkeit Untere Grenze 1"))
        self.declare_parameter('up1red_color', 10, light_int_desc("Rot Farbwert Obere Grenze 1"))
        self.declare_parameter('up1red_sat', 255, light_int_desc("Rot Saturation Obere Grenze 1"))
        self.declare_parameter('up1red_alpha', 255, light_int_desc("Rot Helligkeit Obere Grenze 1"))

        # Dynmaische Paramter für RotFarbgrenzen 2
        self.declare_parameter('low2red_color', 160, light_int_desc("Rot Farbwert Untere Grenze 2"))
        self.declare_parameter('low2red_sat', 100, light_int_desc("Rot Saturation Untere Grenze 2"))
        self.declare_parameter('low2red_alpha', 100, light_int_desc("Rot Helligkeit Untere Grenze 2"))
        self.declare_parameter('up2red_color', 179, light_int_desc("Rot Farbwert Obere Grenze 2"))
        self.declare_parameter('up2red_sat', 255, light_int_desc("Rot Saturation Obere Grenze 2"))
        self.declare_parameter('up2red_alpha', 255, light_int_desc("Rot Helligkeit Obere Grenze 2"))

        # Dynmaische Paramter für Grüne Farbgrenzen
        self.declare_parameter('lowgreen_color', 50, light_int_desc("Grün Farbwert Untere Grenze"))
        self.declare_parameter('lowgreen_sat', 150, light_int_desc("Grün Saturation Untere Grenze"))
        self.declare_parameter('lowgreen_alpha', 50, light_int_desc("Grün Helligkeit Untere Grenze"))
        self.declare_parameter('upgreen_color', 85, light_int_desc("Grün Farbwert Obere Grenze"))
        self.declare_parameter('upgreen_sat', 255, light_int_desc("Grün Saturation Obere Grenze"))
        self.declare_parameter('upgreen_alpha', 255, light_int_desc("Grün Helligkeit Obere Grenze"))

        # Variables
        self.raw_image
        self.bridge = CvBridge()
        self.hsv = np.array([])
        self.waitingforgreen = False
        self.img_row = np.random.randint(0, 256, 640, dtype=np.uint8)

        # definition of the QoS in order to receive data despite WiFi
        qos_policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)

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
        self.sign_timer_period = 0.5
        self.sign_timer = self.create_timer(
            self.sign_timer_period, self.sign_handler)

    # raw data formating routine
    def image_callback(self, data):

        #  Converting raw data into np.array

        img_cv = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding='passthrough')

        # just the image for later use (in sign_identification)
        self.raw_image = img_cv

        # convert image to grayscale
        height, width, _ = img_cv.shape

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

    # starts sign and light detection and publishes result
    def sign_handler(self):

        light_status = self.light_detection()
        sign_status = self.sign_identification()

        # If light and sign are detected prefer sign
        if (sign_status != ""):
            status = sign_status

        elif (light_status != ""):
            status = light_status

        else:
            status = ""

        msg = Pic()
        msg.status = status
        self.publisher_.publish(msg)

    def light_detection(self):

        #  Get Parameters
        min_area = self.get_parameter('min_area').get_parameter_value().integer_value
        max_area = self.get_parameter('max_area').get_parameter_value().integer_value
        low1red_color = self.get_parameter('low1red_color').get_parameter_value().integer_value
        low1red_sat = self.get_parameter('low1red_sat').get_parameter_value().integer_value
        low1red_alpha = self.get_parameter('low1red_alpha').get_parameter_value().integer_value
        up1red_color = self.get_parameter('up1red_color').get_parameter_value().integer_value
        up1red_sat = self.get_parameter('up1red_sat').get_parameter_value().integer_value
        up1red_alpha = self.get_parameter('up1red_alpha').get_parameter_value().integer_value

        low2red_color = self.get_parameter('low2red_color').get_parameter_value().integer_value
        low2red_sat = self.get_parameter('low2red_sat').get_parameter_value().integer_value
        low2red_alpha = self.get_parameter('low2red_alpha').get_parameter_value().integer_value
        up2red_color = self.get_parameter('up2red_color').get_parameter_value().integer_value
        up2red_sat = self.get_parameter('up2red_sat').get_parameter_value().integer_value
        up2red_alpha = self.get_parameter('up2red_alpha').get_parameter_value().integer_value

        lowgreen_color = self.get_parameter('lowgreen_color').get_parameter_value().integer_value
        lowgreen_sat = self.get_parameter('lowgreen_sat').get_parameter_value().integer_value
        lowgreen_alpha = self.get_parameter('lowgreen_alpha').get_parameter_value().integer_value
        upgreen_color = self.get_parameter('upgreen_color').get_parameter_value().integer_value
        upgreen_sat = self.get_parameter('upgreen_sat').get_parameter_value().integer_value
        upgreen_alpha = self.get_parameter('upgreen_alpha').get_parameter_value().integer_value

        # Minimale Fläche (z. B. Ampellicht)
        hsv = self.hsv
        status = ""
        if (hsv.size == 0):
            status = ""
            return

        # Function for signs - string for Outputs("")
        # min_area = 30  # Minimale fläche für rotes licht
        # Farbgrenzen für rot
        lower_red1 = np.array([low1red_color, low1red_sat, low1red_alpha])
        upper_red1 = np.array([up1red_color, up1red_sat, up1red_alpha])
        lower_red2 = np.array([low2red_color, low2red_sat, low2red_alpha])
        upper_red2 = np.array([up2red_color, up2red_sat, up2red_alpha])

        # Farbgrenzen für Grün (Ampelgrün)
        lower_green = np.array([lowgreen_color, lowgreen_sat, lowgreen_alpha])
        upper_green = np.array([upgreen_color, upgreen_sat, upgreen_alpha])

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

    # Predict the class of an image using a pre-trained model
    def sign_identification(self):

        image = self.raw_image

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
        node.destroy_node()

    finally:
        node.destroy_node()
        print('Shutting Down Camera Node')


if __name__ == '__main__':

    main()

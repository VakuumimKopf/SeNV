import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2 as cv2
import numpy as np
from senv_interfaces.msg import Pic
from senv.description import light_int_desc, int_desc

# Import ultralytics for sign detection with "pip install ultralytics"
from ultralytics import YOLO

# Import Pytorch for Feature Map Generation
import torch
# load model
model = YOLO("src/senv/senv/best3.0.pt")


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
        self.declare_parameter('min_area', 30, int_desc("Minimum Area for Light"))
        self.declare_parameter('max_area', 300, int_desc("Maximum Area for Light"))

        # Variables
        self.raw_image = None
        self.bridge = CvBridge()
        self.hsv = np.array([])
        self.waitingforgreen = False
        self.kill_light = False

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
        self.sign_timer_period = 0.016
        self.sign_timer = self.create_timer(
            self.sign_timer_period, self.sign_handler)

    # raw data formating routine
    def image_callback(self, data):

        #  Converting raw data into np.array

        img_cv = cv2.blur(self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding='passthrough'), (5, 5))

        # just the image for later use (in sign_identification)
        self.raw_image = img_cv

        # convert image to grayscale
        height, width, _ = img_cv.shape

        # Speichere HSV für farbanalyse in sign detection
        # Bereich: rechtes Drittel, mittleres Drittel vertikal
        x_start = width // 2
        x_end = width
        y_start = height // 4
        y_end = height // 2

        roi = img_cv[y_start:y_end, x_start:x_end]
        cv2.imshow("YOLOv8-Erkennung (Confidence > 0.8)", img_cv)
        cv2.waitKey(1)
        if roi.size == 0:
            self.hsv = None
        else:
            self.hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    # starts sign and light detection and publishes result
    def sign_handler(self):

        msg = Pic()

        # msg.light = self.light_detection()
        if self.kill_light is False:
            msg.light = self.light_detection()
        msg.sign = self.sign_identification()
        # self.get_logger().info(str(msg.sign))

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
            return status

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
            status = "red light"
            self.waitingforgreen = True
        if max_green_area >= max_red_area and max_green_area > 0:
            status = "green light"
            self.waitingforgreen = False
            self.kill_light = True

        # cv2.imshow("IMG_red", red_mask)
        # cv2.imshow("IMG_green", green_mask)
        # cv2.waitKey(1)
        self.get_logger().info(status)
        return status

    # Predict the class of an image using a pre-trained model
    def sign_identification(self):

        if self.raw_image is None:
            return ""

        image = self.raw_image
        threshhold = 0.85
        results = model(image, verbose=False)  # kann auch save=True sein
        class_names = model.names

        # IDs of the detected classes (e.g. 0, 1, 2 …)
        class_ids = results[0].boxes.cls.cpu().numpy().astype(int)
        # Bounding boxes: [x1, y1, x2, y2], neccesary for filtered sign display in the image
        xyxy = results[0].boxes.xyxy.cpu().numpy()

        # folter for signs with a probability of at least 0.8
        high_conf_indices = [i for i, conf in enumerate(results[0].boxes.conf.cpu().numpy())
                             if conf > threshhold]

        # if there is no sign with at least 80% recognition
        if not high_conf_indices:
            return ""
        # Display signs with over 80% in another image

        # manual display of filtered signs
        for i in high_conf_indices:
            x1, y1, x2, y2 = map(int, xyxy[i])
            # filter signs that are too small/too far away
            if abs(y1 - y2) < 40:
                del high_conf_indices[i]
            label = f"{class_names[class_ids[i]]}: {results[0].boxes.conf.cpu().numpy()[i]:.2f}"

            # Zeichne Rechteck
            cv2.rectangle(image, (x1, y1), (x2, y2), (255, 0, 0), 2)
            # Zeichne Label
            cv2.putText(image, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        # Zeige das bearbeitete Bild mit nur den gefilterten Erkennungen
        # cv2.imshow("YOLOv8-Erkennung (Confidence > 0.8)", image)
        # cv2.waitKey(1)

        filtered_class_ids = [class_ids[i] for i in high_conf_indices]
        detected_labels = [class_names[i] for i in filtered_class_ids]
        # when there is no sign detected, the list is empty
        if detected_labels == []:
            return ""
        # self.get_logger().info(f"Detected Sign is {detected_labels}")
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

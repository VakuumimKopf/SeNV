import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from copy import copy
from senv_interfaces.msg import Pic
from cv_bridge import CvBridge


class lane_detect(Node):
    def __init__(self):
        super().__init__('lane_detect')

        #  Parameters

        #  Variables
        self.edges = None
        self.raw_image = None
        self.bridge = CvBridge()

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

        # create timers for lane detection
        self.lane_timer_period = 0.1
        self.lane_timer = self.create_timer(
            self.lane_timer_period, self.lane_detection)

    # raw data formating routine
    def image_callback(self, data):

        img_cv = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding='passthrough')

        #  Convert RGB image to greyscale
        greyscale = cv2.cvtColor(img_cv, cv2.COLOR_BGR2GRAY)

        #  Applying gaussian blur for noise reduction
        #  Parameter
        kernel_size = 7
        blur = cv2.GaussianBlur(greyscale, (kernel_size, kernel_size), 0)

        #  Applying canny edge detection of the whole image
        low_t = 120  # first threshhold for hysteresis
        high_t = 250  # secound treshhold
        edges = cv2.Canny(blur, low_t, high_t)

        self.contours(copy(edges))

        # Saving edges in self.edges and data in self.raw_image
        self.edges = edges
        self.raw_image = img_cv

    # line detection in formated data
    def lane_detection(self):

        #  Parameter
        number_parts = 5

        #  copy current version of self.edges and self.raw_image
        edges = copy(self.edges)
        raw_image = copy(self.raw_image)

        if edges is None or raw_image is None:
            return

        #  Calculating the y points of the wanted segments
        segment_info = self.segment_info(raw_image, number_parts)

        #  Appling polygon mask to filter only for region of interest
        regions = self.spilter(edges, segment_info)

        result = None
        lastres = raw_image
        num = 0
        for region in regions:

            self.get_logger().info(str(num))
            #  Appling hough transform returning set of found lines
            hough = self.hough_transform(region, raw_image, num)

            #  Catch if no hough lines are detected
            if hough is None:
                continue

            # Appling hough lines on input image
            result = self.draw_lane_lines(lastres, self.lane_lines(hough, segment_info[num]))
            lastres = result
            num = num + 1

        if result is None:
            return
        cv2.imshow("Result", result)
        cv2.waitKey(1)

    def contours(self, image):
        cnts = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]

        # Iterate thorugh contours and draw rectangles around contours
        for c in cnts:
            x, y, w, h = cv2.boundingRect(c)
            cv2.rectangle(image, (x, y), (x + w, y + h), (36, 255, 12), 2)

        cv2.imshow("contours", image)

    def segment_info(self, image, n):

        segments = []
        rows, cols = image.shape[:2]
        for i in range(0, n):
            bottom = (rows - (i/n * rows * 0.4))
            top = rows * 0.6 + ((n-i-1) / n * rows * 0.4)
            segments.append([bottom, top])

        return segments

    def spilter(self, image, segment_info):

        ignore_mask_color = 255
        rows, cols = image.shape[:2]
        masked_images = []
        n = len(segment_info)

        for i in range(0, n):
            mask = np.zeros_like(image)
            bottom_left = [0.0, (segment_info[i][0])]
            top_left = [0.0, (segment_info[i][1])]
            bottom_right = [cols, (segment_info[i][0])]
            top_right = [cols, (segment_info[i][1])]

            #  Building polygon
            vertices = np.array([[bottom_left, top_left, top_right, bottom_right]], dtype=np.int32)
            cv2.fillPoly(mask, vertices, ignore_mask_color)

            masked_images.append(cv2.bitwise_and(image, mask))

        num = 0
        for masked_image in masked_images:
            cv2.imshow("masked" + str(num), masked_image)
            num = num + 1

        return masked_images

    #  Determine and cut region of interest
    def hough_transform(self, image, raw_image, num):

        #  Parameters
        rho = 1
        theta = np.pi/180
        threshold = 10
        minLineLength = 10
        maxLineGap = 50

        lines = cv2.HoughLinesP(image, rho=rho, theta=theta, threshold=threshold,
                                minLineLength=minLineLength, maxLineGap=maxLineGap)

        cdstP = copy(raw_image)

        if lines is not None:
            for i in range(0, len(lines)):
                line = lines[i][0]
                cv2.line(cdstP, (line[0], line[1]), (line[2], line[3]), (0, 0, 255), 3, cv2.LINE_AA)

        cv2.imshow("hough" + str(num), cdstP)
        return lines

    def draw_lane_lines(self, image, lines, color=[0, 0, 255], thickness=10):

        line_image = np.zeros_like(image)
        for line in lines:
            if line is not None:
                cv2.line(line_image, *line,  color, thickness)

        cv2.imshow("Lines", line_image)
        return cv2.addWeighted(image, 1.0, line_image, 1.0, 0.0)

    def lane_lines(self, lines, segment_info):
        left_lane, right_lane = self.average_slope_intercept(lines)
        y1 = segment_info[0]
        y2 = segment_info[1]
        left_line = self.pixel_points(y1, y2, left_lane)
        right_line = self.pixel_points(y1, y2, right_lane)
        return left_line, right_line

    def average_slope_intercept(self, lines):

        #  Parameters
        filter_value_straight_lines = 0.4  # Defining what is maximum slope for straight lines
        middle = 320

        #  Define Variables
        left_lines = []  # (slope, intercept)
        left_weights = []  # (length,)
        right_lines = []  # (slope, intercept)
        right_weights = []  # (length,)

        straight_lines = []  # Array for all straight lines
        for line in lines:
            for x1, y1, x2, y2 in line:

                #  Ignore all Vertical lines
                if x1 == x2:
                    continue

                # Calculating slope of a line
                slope = (y2 - y1) / (x2 - x1)

                # Calculating weigth dependent of the distance to the middle
                weigth = 1 - (abs(((x1 + x2) / 2) - middle) / middle)

                # Calculating intercept of a line
                intercept = y1 - (slope * x1)

                # Calculating length of a line
                length = np.sqrt(((y2 - y1) ** 2) + ((x2 - x1) ** 2))

                # slope of left lane is negative and for right lane slope is positive
                if slope < -(filter_value_straight_lines):

                    left_lines.append((slope, intercept))
                    left_weights.append((length * weigth))

                elif slope > filter_value_straight_lines:

                    right_lines.append((slope, intercept))
                    right_weights.append((length * weigth))

                else:
                    straight_lines.append(line)

        #  Convert in a single line
        left_lane = np.dot(left_weights,  left_lines) / np.sum(left_weights) if len(left_weights) > 0 else None
        right_lane = np.dot(right_weights, right_lines) / np.sum(right_weights) if len(right_weights) > 0 else None

        return left_lane, right_lane

    def pixel_points(self, y1, y2, line):
        if line is None:
            return None
        slope, intercept = line
        x1 = int((y1 - intercept)/slope)
        x2 = int((y2 - intercept)/slope)
        y1 = int(y1)
        y2 = int(y2)
        return ((x1, y1), (x2, y2))


def main(args=None):
    rclpy.init(args=args)
    node = lane_detect()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':

    main()

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

        """# create subscribers for image data with changed qos
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            qos_profile=qos_policy)
        self.subscription  # prevent unused variable warning

        # create timers for lane detection
        self.lane_timer_period = 0.1
        self.lane_timer = self.create_timer(
            self.lane_timer_period, self.lane_detection)"""

    # raw data formating routine
    def image_callback(self, data):

        #  img_cv = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding='passthrough')
        img_cv = data

        #  Convert RGB image to greyscale
        greyscale = cv2.cvtColor(img_cv, cv2.COLOR_BGR2GRAY)

        #  Applying gaussian blur for noise reduction
        kernel_size = 5
        blur = cv2.GaussianBlur(greyscale, (kernel_size, kernel_size), 0)

        #  Applying canny edge detection of the whole image
        low_t = 120  # first threshhold for hysteresis
        high_t = 250  # secound treshhold
        edges = cv2.Canny(blur, low_t, high_t)

        # Saving edges in self.edges and data in self.raw_image
        self.edges = edges
        self.raw_image = img_cv

    # line detection in formated data
    def lane_detection(self):

        #  Parameter
        number_parts = 3

        #  copy current version of self.edges and self.raw_image
        edges = copy(self.edges)
        raw_image = copy(self.raw_image)

        if edges is None or raw_image is None:
            return

        #  Calculating the y points of the wanted segments
        segment_info = self.segment_info(raw_image, number_parts)

        #  Appling polygon mask to filter only for region of interest
        regions = self.spilter(edges, segment_info)

        #  Looping through all segments
        result = None
        last_lane_lines = None
        last_result = raw_image
        num = 0
        guiding_lines = []
        for region in regions:

            self.get_logger().info(str(num))

            #  Appling hough transform returning set of found lines
            hough = self.hough_transform(region, num)

            #  Catch if no hough lines are detected
            if hough is None:
                continue

            #  Filtering the found lines
            filtered = self.filtering(hough, last_lane_lines, num)

            #  Calculating side lines of road
            lines = self.lane_lines(filtered, segment_info[num])

            #  Calculating guiding lines
            guiding_lines.append(self.get_guiding_line(lines))

            result = self.draw_lane_lines(last_result, lines)
            last_result = result
            num = num + 1

        if result is None:
            return
        
        self.get_logger().info(str(guiding_lines))
        result = self.draw_lane_lines(result, guiding_lines)

        cv2.imshow("Result", result)
        cv2.waitKey(1)

    #  Filtering a set of lines based on lane widthe
    def filtering(self, lines: np.ndarray, last_lane_lines, num):

        #  Parameters
        canvas = np.zeros_like(self.raw_image)
        pair_margin = 0.5
        max_x_distance = 100
        min_intercept_distance = 10
        max_intercept_distance = 30

        arr = []
        #  Calculate the slope, middle and intercept for all lines
        for line in lines:
            for x1, y1, x2, y2 in line:
                # Throw away all vertical lines
                if x1 == x2:
                    continue
                else:
                    slope = (y2 - y1) / (x2 - x1)
                    middle = (x1 + x2) / 2
                    intercept = y1 - (slope * x1)
                    arr.append([x1, y1, x2, y2, slope, middle, intercept])

        #  Sort the array based on slope
        arr.sort(key=lambda x: x[4])

        #  Determine Groups where the lines have similar slope, are near to eachother
        groupes = self.finding_groups(arr, pair_margin, max_x_distance)

        #  Filter each group so only lines at the right distance remain
        # f_groups = self.filter_for_distance(groupes, max_intercept_distance, min_intercept_distance)

        #  Draw Groups
        for i in range(0, len(groupes)):
            groupe = groupes[i]
            color = list(np.random.random(size=3) * 256)
            for j in range(0, len(groupe)):
                line = groupe[j]
                cv2.line(canvas, (line[0], line[1]), (line[2], line[3]), color, 3, cv2.LINE_AA)

        cv2.imshow("canvas" + str(num), canvas)

        out = []
        #  Combine Groups into np array
        for i in range(0, len(groupes)):
            groupe = groupes[i]
            for j in range(0, len(groupe)):
                line = groupe[j]
                out.append([line[0], line[1], line[2], line[3]])
        return out

    #  Group lines based on slope, pair_margin determines range of slope of a group
    def finding_groups(self, a, pair_margin, max_distance):

        #  Variable
        out = []
        group = []

        #  Iterate throuh whole list
        for i in range(0, len(a)):

            #  Check if a new group, and if so just append the current element and continue
            if len(group) == 0:

                group.append(a[i])
                continue

            #  Check if the new elements slope is in the right range
            slope_in_range = a[i][4] >= self.group_min(group, 4) and a[i][4] <= self.group_min(group, 4) + pair_margin

            #  Check if the new element is near the others of the group
            distance_in_range = a[i][5] <= self.group_min(group, 5) + max_distance and a[i][5] >= self.group_max(group, 5) - max_distance

            if slope_in_range and distance_in_range:
                group.append(a[i])

            else:
                if (len(group) > 1):
                    out.append(group)
                group = [a[i]]

        if (len(group) > 1):
            out.append(group)

        return out

    #  Determines the min value of a group of arrays
    def group_min(self, group, n):
        min = None
        for i in range(0, len(group)):
            if min is None:
                min = group[i][n]
            elif group[i][n] < min:
                min = group[i][n]
        return min

    #  Determines the min value of a group of arrays
    def group_max(self, group, n):
        max = None
        for i in range(0, len(group)):
            if max is None:
                max = group[i][n]
            elif group[i][n] > max:
                max = group[i][n]
        return max

    def filter_for_distace(groups, max_intercept_distance, min_intercept_distance):
        out = []
        for i in range(len(groups)):
            group = groups[i]
            new = []
            for j in range(len(group)):
                line = group[j]
        return out

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
        self.get_logger().info(str(rows) + " : " + str(cols))
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
    def hough_transform(self, image, num):

        #  Parameters
        rho = 1
        theta = np.pi/180
        threshold = 30
        minLineLength = 20
        maxLineGap = 50

        lines = cv2.HoughLinesP(image, rho=rho, theta=theta, threshold=threshold,
                                minLineLength=minLineLength, maxLineGap=maxLineGap)

        canvas = copy(self.raw_image)

        if lines is not None:
            for i in range(0, len(lines)):
                line = lines[i][0]
                cv2.line(canvas, (line[0], line[1]), (line[2], line[3]), (0, 0, 255), 3, cv2.LINE_AA)

        cv2.imshow("hough" + str(num), canvas)
        return lines

    def draw_lane_lines(self, image, lines, color=[0, 0, 255], thickness=10):

        self.get_logger().info(str(lines))
        canvas = np.zeros_like(image)

        if lines is not None:
            for i in range(0, len(lines)):
                line = lines[i]
                if line is not None:
                    cv2.line(canvas, (line[0], line[1]), (line[2], line[3]), (0, 0, 255), 3, cv2.LINE_AA)

        return cv2.addWeighted(image, 1.0, canvas, 1.0, 0.0)

    def lane_lines(self, lines, segment_info):
        left_lane, right_lane = self.build_road_lines(lines)
        y1 = segment_info[0]
        y2 = segment_info[1]
        left_line = self.pixel_points(y1, y2, left_lane)
        right_line = self.pixel_points(y1, y2, right_lane)
        out = []
        if left_line is not None and right_line is not None:
            out.append([left_line[0][0], left_line[0][1], left_line[1][0], left_line[1][1]])
            out.append([right_line[0][0], right_line[0][1], right_line[1][0], right_line[1][1]])
        return out

    def build_road_lines(self, lines):

        #  Parameters
        filter_value_straight_lines = 0.4  # Defining what is maximum slope for straight lines
        middle = 320

        #  Define Variables
        left_lines = []  # (slope, intercept)
        left_weights = []  # (length,)
        right_lines = []  # (slope, intercept)
        right_weights = []  # (length,)

        straight_lines = []  # Array for all straight lines
        for i in range(0, len(lines)):
            line = lines[i]
            x1 = line[0]
            y1 = line[1]
            x2 = line[2]
            y2 = line[3]

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

    def get_guiding_line(self, lines):

        if len(lines) == 2:
            line = lines[0]
            n_line = lines[1]
            x1 = round((line[0] + n_line[0]) / 2)
            y1 = line[1]
            x2 = round((line[2] + n_line[2]) / 2)
            y2 = line[3]

            return [x1, y1, x2, y2]
        else:
            self.get_logger().info("Error in get_guiding_line")
            return None


def main(args=None):

    img = cv2.imread("/home/oliver/senv_ws/src/senv/senv/image2.png")

    rclpy.init(args=args)
    node = lane_detect()
    node.image_callback(img)
    node.lane_detection()
    cv2.waitKey(0)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':

    main()

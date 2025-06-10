import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from copy import copy
from cv_bridge import CvBridge
import math
from senv_interfaces.msg import Lane


class lane_detect(Node):
    def __init__(self):
        super().__init__('lane_detect')

        #  Parameters
        self.debug_mode = False

        #  Variables
        self.edges = None
        self.raw_image = None
        self.canvas = None
        self.guiding_lines = None
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

        # create publisher for the driving command
        self.publisher_ = self.create_publisher(Lane, 'lane', qos_profile=qos_policy)

        # create timers for lane detection
        self.lane_timer_period = 0.1
        self.lane_timer = self.create_timer(
            self.lane_timer_period, self.lane_detection)

    # raw data formating routine
    def image_callback(self, data):

        #  Get data from camera and convert to cv2 image
        img_cv = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding='passthrough')

        #  Convert RGB image to greyscale
        greyscale = cv2.cvtColor(img_cv, cv2.COLOR_BGR2GRAY)

        #  Applying gaussian blur for noise reduction
        kernel_size = 5
        blur = cv2.GaussianBlur(greyscale, (kernel_size, kernel_size), 0)

        #  Applying canny edge detection of the whole image
        low_t = 80  # first threshhold for hysteresis
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
        self.canvas = np.zeros_like(self.raw_image)

        if edges is None or raw_image is None:
            return

        #  Calculating the y points of the wanted segments
        segment_info = self.segment_info(raw_image, number_parts)

        #  Split the image in segments with a polygon mask and only use the region of interest
        regions = self.spilter(edges, segment_info)

        #  Setup all loop variables and looping through all segments
        result = None
        last_lane_lines = None
        last_result = raw_image
        num = 0
        guiding_lines = [None, None, None]

        for region in regions:

            #  Appling hough transform returning set of found lines
            hough = self.hough_transform(region, num)

            #  Catch if no hough lines are detected
            if hough is None:
                continue

            #  Filtering the found lines on multiple factors
            filtered = self.filtering(hough, last_lane_lines, num)

            #  Calculating side lines of road
            lines = self.lane_lines(filtered, segment_info[num])

            #  Calculating guiding lines in the middle of the road
            guiding_lines[num] = self.get_guiding_line(lines)

            #  Draw the lane lines on the result image
            result = self.draw_lane_lines(last_result, lines)

            #  Update the loop variables on the end of the loop
            last_result = result
            num = num + 1

        if result is None:
            return

        #  Update the guidling lines with a new value
        self.update_guiding_lines(guiding_lines)

        result = self.draw_lane_lines(result, self.guiding_lines)

        if self.debug_mode is True:
            cv2.imshow("Used", self.canvas)
            cv2.imshow("Result", result)
            cv2.waitKey(1)

        msg = Lane()
        msg.gline_a = self.guiding_lines[0]
        msg.gline_b = self.guiding_lines[1]
        msg.gline_c = self.guiding_lines[2]

        self.publisher_.publish

    #  Filtering a set of lines based on lane widthe
    def filtering(self, lines: np.ndarray, last_lane_lines, num):

        #  Parameters
        canvas = np.zeros_like(self.raw_image)
        pair_margin = 0.7
        max_x_distance = 100
        min_intercept_distance = 10
        max_intercept_distance = 23

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

        #  Draw Groups
        if self.debug_mode is True:
            for i in range(0, len(groupes)):
                groupe = groupes[i]
                color = list(np.random.random(size=3) * 256)
                for j in range(0, len(groupe)):
                    line = groupe[j]
                    cv2.line(canvas, (line[0], line[1]), (line[2], line[3]), color, 3, cv2.LINE_AA)
            cv2.imshow("canvas" + str(num), canvas)

        #  Filter each group so only lines at the right distance remain
        groupes = self.filter_for_distance(groupes, max_intercept_distance, min_intercept_distance)

        out = []
        #  Combine Groups into array and throw away slope, intercept and middle
        for i in range(0, len(groupes)):
            groupe = groupes[i]
            for j in range(0, len(groupe)):
                line = groupe[j]
                out.append([line[0], line[1], line[2], line[3]])
        return out

    #  Group lines based on slope, pair_margin determines range of slope of a group
    def finding_groups(self, a, pair_margin, max_distance):

        #  Variables
        out = []
        group = []

        #  Iterate through whole list
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

    #  Filter inside the groups for lines with a specific distance
    def filter_for_distance(self, groups, max_intercept_distance, min_intercept_distance):

        #  Variables
        out = []

        for i in range(len(groups)):
            group = groups[i]
            new = []

            #  Check each group and compare each element to each other
            for j in range(len(group)):
                for k in range(j+1, len(group)):

                    line1 = group[j]
                    line2 = group[k]

                    #  Calculate the distance between the two line segments
                    distance = self.segments_distance(line1[0], line1[1], line1[2], line1[3], line2[0], line2[1], line2[2], line2[3])

                    #  Check if the distance is in range, if so append both the lines to the new list
                    if min_intercept_distance <= distance <= max_intercept_distance:
                        new.append(line1)
                        new.append(line2)

            out.append(new)
        return out

    #  Calculate the distance between two line segments
    def segments_distance(self, x11, y11, x12, y12, x21, y21, x22, y22):

        #  Check if the two lines intersect
        if self.segments_intersect(x11, y11, x12, y12, x21, y21, x22, y22):
            return 0

        #  Try each of the 4 vertices with the other segment and select the min
        distances = []
        distances.append(self.point_segment_distance(x11, y11, x21, y21, x22, y22))
        distances.append(self.point_segment_distance(x12, y12, x21, y21, x22, y22))
        distances.append(self.point_segment_distance(x21, y21, x11, y11, x12, y12))
        distances.append(self.point_segment_distance(x22, y22, x11, y11, x12, y12))
        return min(distances)

    #  Calculate if two lines intersect
    def segments_intersect(self, x11, y11, x12, y12, x21, y21, x22, y22):
        dx1 = x12 - x11
        dy1 = y12 - y11
        dx2 = x22 - x21
        dy2 = y22 - y21
        delta = dx2 * dy1 - dy2 * dx1
        if delta == 0:
            return False  # parallel segments
        s = (dx1 * (y21 - y11) + dy1 * (x11 - x21)) / delta
        t = (dx2 * (y11 - y21) + dy2 * (x21 - x11)) / (-delta)
        return (0 <= s <= 1) and (0 <= t <= 1)

    #  Calculate the distance between a line and a point
    def point_segment_distance(self, px, py, x1, y1, x2, y2):

        dx = x2 - x1
        dy = y2 - y1

        if dx == dy == 0:  # the segment's just a point
            return math.hypot(px - x1, py - y1)

        # Calculate the t that minimizes the distance.
        t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)

        # See if this represents one of the segment's
        # end points or a point in the middle.
        if t < 0:
            dx = px - x1
            dy = py - y1
        elif t > 1:
            dx = px - x2
            dy = py - y2
        else:
            near_x = x1 + t * dx
            near_y = y1 + t * dy
            dx = px - near_x
            dy = py - near_y

        return math.hypot(dx, dy)

    #  Calculate the y values of n segments
    def segment_info(self, image, n):

        #  Parameter to determine interessting part of image
        cut = 0.4

        segments = []
        rows, cols = image.shape[:2]
        for i in range(0, n):
            bottom = (rows - (i/n * rows * cut))
            top = rows * 0.6 + ((n-i-1) / n * rows * cut)
            segments.append([bottom, top])

        return segments

    #  Split the original image into n different images, hiding the other part with a mask
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

        #  Draw masked canny image
        if self.debug_mode is True:
            for i in range(len(masked_images)):
                cv2.imshow("masked" + str(i), masked_images[i])

        return masked_images

    #  Determine and cut region of interest
    def hough_transform(self, image, num):

        #  Parameters
        rho = 1
        theta = np.pi/180
        threshold = 20
        minLineLength = 10
        maxLineGap = 70

        lines = cv2.HoughLinesP(image, rho=rho, theta=theta, threshold=threshold,
                                minLineLength=minLineLength, maxLineGap=maxLineGap)

        canvas = copy(self.raw_image)

        #  Draw found hough lines on image
        if self.debug_mode is True:
            if lines is not None:
                for i in range(0, len(lines)):
                    line = lines[i][0]
                    cv2.line(canvas, (line[0], line[1]), (line[2], line[3]), (0, 0, 255), 3, cv2.LINE_AA)

            cv2.imshow("hough" + str(num), canvas)

        return lines

    #  Draw Lines on a given image
    def draw_lane_lines(self, image, lines, color=[0, 0, 255], thickness=10):

        canvas = np.zeros_like(image)

        if lines is not None:
            for i in range(0, len(lines)):
                line = lines[i]
                if line is not None:
                    cv2.line(canvas, (line[0], line[1]), (line[2], line[3]), (0, 0, 255), 3, cv2.LINE_AA)

        return cv2.addWeighted(image, 1.0, canvas, 1.0, 0.0)

    #  Calculate the lines of each side of the road
    def lane_lines(self, lines, segment_info):

        #  Get a function for each road side
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

    #  Sort all lines into right and left lines of the road and merge them
    def build_road_lines(self, lines):

        #  Parameters
        filter_value_straight_lines = 0.3  # Defining what is maximum slope for straight lines
        middle = 320

        #  Define Variables
        left_lines = []  # (slope, intercept)
        left_weights = []  # (length,)
        right_lines = []  # (slope, intercept)
        right_weights = []  # (length,)

        selected_lines = []

        straight_lines = []  # Array for all straight lines
        for i in range(0, len(lines)):
            line = lines[i]
            x1 = line[0]
            y1 = line[1]
            x2 = line[2]
            y2 = line[3]

            #  Calculate the middle, slope, intercept and length
            middle_of_line = (x1 + x2) / 2
            slope = (y2 - y1) / (x2 - x1)
            intercept = y1 - (slope * x1)
            length = np.sqrt(((y2 - y1) ** 2) + ((x2 - x1) ** 2))

            #  slope of left lane is negative and for right lane slope is positive,
            #  but only append when the side is matching the slope
            if slope < -(filter_value_straight_lines):
                if middle_of_line <= middle:
                    left_lines.append((slope, intercept))
                    left_weights.append((length))
                    selected_lines.append(line)

            elif slope > filter_value_straight_lines:
                if middle_of_line >= middle:
                    right_lines.append((slope, intercept))
                    right_weights.append((length))
                    selected_lines.append(line)

            else:
                straight_lines.append(line)

        self.canvas = self.draw_lane_lines(self.canvas, selected_lines)

        #  Convert in a single line
        left_lane = np.dot(left_weights,  left_lines) / np.sum(left_weights) if len(left_weights) > 0 else None
        right_lane = np.dot(right_weights, right_lines) / np.sum(right_weights) if len(right_weights) > 0 else None

        return left_lane, right_lane

    #  Calculate points from a function and return line as two points
    def pixel_points(self, y1, y2, line):
        if line is None:
            return None
        slope, intercept = line
        x1 = int((y1 - intercept)/slope)
        x2 = int((y2 - intercept)/slope)
        y1 = int(y1)
        y2 = int(y2)
        return ((x1, y1), (x2, y2))

    #  Calculate the middle line of the round based on the lines on both sides
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
            return None

    #  Update the middle lines to prevent jumps, missing values and overall smooth the driving
    def update_guiding_lines(self, lines):

        old = self.guiding_lines

        if old is None:
            self.guiding_lines = lines
            return

        for i in range(len(lines)):

            if old[i] is None:
                lines[i] = lines[i]

            elif lines[i] is None:
                lines[i] = old[i]

            else:
                x1_c = old[i][0] - lines[i][0]
                x2_c = old[i][2] - lines[i][2]

                if abs(x1_c) > 50 or abs(x2_c > 50):
                    lines[i][0] = round((old[i][0] + lines[i][0]) / 2)
                    lines[i][2] = round((old[i][2] + lines[i][2]) / 2)
                else:
                    lines[i] = lines[i]

        self.guiding_lines = lines

    """#  Calculate a Twist msg based on the middle line of the road
    def drive(self):

        #  Parameters
        middle = 320
        area_short = 15
        area_long = 35

        lines = self.guiding_lines

        if lines is None or lines[0] is None:
            return

        #  Calculate middle of the line
        x1 = lines[0][0]
        x2 = lines[0][2]
        x_d = (x1 + x2) / 2

        #  Determine Turn Value
        if x_d < middle - area_short:
            if x_d < middle - area_long:
                turn = 0.35
            else:
                turn = 0.3
        elif x_d > middle + area_short:
            if x_d > middle + area_long:
                turn = -0.35
            else:
                turn = -0.3
        else:
            turn = 0.0

        speed = 0.2*(1-abs(turn))
    """


def main(args=None):

    rclpy.init(args=args)
    node = lane_detect()
    cv2.waitKey(0)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':

    main()

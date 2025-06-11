import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from senv_interfaces.msg import Pic, Laser
from senv_interfaces.action import ConTask
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from senv.stopper import Stopper
import time
from senv.description import float_desc, int_desc, light_int_desc, bool_desc
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
import numpy as np

# model for detecting human near crosswalk
from ultralytics import YOLO
model = YOLO("senv/human.pt")


class crosswalk_con(Node):
    def __init__(self):
        super().__init__('crosswalk_con')
        self.get_logger().info('crosswalk_con node has been started.')

        # Parameters
        self.turned_on = False

        # QOS Policy Setting
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)

        # Subscribing to camera and laser topics
        self.subscriber_pic = self.create_subscription(
            Pic,  # Replace with the actual message type
            'pic',
            self.pic_callback,
            qos_profile=qos_policy
        )
        self.subscriber_pic  # prevent unused variable warning

        self.subscriber_laser = self.create_subscription(
            Laser,  # Replace with the actual message type
            'laser',
            self.laser_callback,
            qos_profile=qos_policy
        )
        self.subscriber_laser  # prevent unused variable warning

        self.crosswalk_task_server_ = ActionServer(
            self,
            ConTask,
            "crosswalk_task",
            self.execute_callback
        )
        # create subscribers for image data with changed qos
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            qos_profile=qos_policy)
        self.subscription  # prevent unused variable warning

        self.bridge = CvBridge()
        self.img = []

        self.publisher_driver = self.create_publisher(Twist, 'driving', qos_profile=qos_policy)

        self.x1 = 0
        self.x2 = 0
        self.y1 = 0
        self.y2 = 0

    def execute_callback(self, goal_handle: ServerGoalHandle):

        # Get request from goal
        target = goal_handle.request.start_working
        self.get_logger().info("starting crosswalk server")

        # Execute action
        self.turned_on = target
        self.datahandler()
        self.get_logger().info("Handling crosswalk complete")

        # Final Goal State
        goal_handle.succeed()

        # Result
        result = ConTask.Result()
        result.finished = True
        return result

    def pic_callback(self, msg):
        if self.turned_on is False:
            return
        # Process the incoming message
        self.get_logger().info('Received message pic')
        # Add driving logic here

    def laser_callback(self, msg):
        if self.turned_on is False:
            return
        # Define your callback function here
        self.get_logger().info('Received message laser')
        # obstacle avoidance

    # raw data formating routine
    def image_callback(self, data):
        self.img = data

    def datahandler(self):
        while True:
            self.get_logger().info("Handling crosswalk data")
            stopmsg = Twist()
            stopmsg.angular.z = 0.0
            stopmsg.angular.x = 0.0
            self.get_logger().info("Suche nach Mensch")

            # returns bool (if human -> True)
            human_there = self.detect_human()

            while human_there:
                self.get_logger().info("Mensch wartet am Zebrastreifen")
                self.publisher_driver.publish(stopmsg)
                human_still_there = self.detect_human()
                # if the human hasnt been removed
                if not human_still_there:
                    human_there = False
            # if there is no human
            else:
                # driverlogic here
                pass

    # detect human
    def detect_human(self):
        """
        Predict the class of an image using a pre-trained model. DO NOT USE cv2.imshow IN HERE OR IT WONT WORK
        """
        if self.img != []:
            # threshhold/least confidence of detected signs
            threshhold = 0.8
            # get the image from the camera as np.array
            image = self.bridge.compressed_imgmsg_to_cv2(self.img, desired_encoding='passthrough')
            # Prediction (Inference)
            results = model(image, verbose=False)  # kann auch save=True sein

            # classlist from model (the names must match your `data.yaml`)
            class_names = model.names

            # IDs of the detected classes (e.g. 0, 1, 2 …)
            class_ids = results[0].boxes.cls.cpu().numpy().astype(int)
            # x and y coordinates of the bounding boxes, might be used later on for filtering small humans in signs
            xyxy = results[0].boxes.xyxy.cpu().numpy()
            self.x1, self.y1, self.x2, self.y2 = map(int, xyxy[0])

            # folter for signs with a probability of at least 0.8
            high_conf_indices = [i for i, conf in enumerate(results[0].boxes.conf.cpu().numpy())
                                 if conf > threshhold]
            # if there is no sign with at least 80% recognition
            if not high_conf_indices:
                return ""

            filtered_class_ids = [class_ids[i] for i in high_conf_indices]
            detected_labels = [class_names[i] for i in filtered_class_ids]
            # when there is no sign detected, the list is empty
            if detected_labels == []:
                return False
            # self.get_logger().info(f"Detected Sign is {detected_labels}")
            else:
                return True

        else:
            return ""


def main(args=None):
    rclpy.init(args=args)
    node = crosswalk_con()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.destroy_node()

    finally:
        node.destroy_node()
        print('Shutting Down crosswalk_Con')


if __name__ == '__main__':
    main()

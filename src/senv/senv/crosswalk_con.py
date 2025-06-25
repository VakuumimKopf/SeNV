import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.action.server import ServerGoalHandle
from senv_interfaces.msg import Pic, Laser, Move
from senv_interfaces.action import ConTask
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2


class Crosswalk_con(Node):
    def __init__(self):
        super().__init__('crosswalk_con')
        self.get_logger().info('crosswalk_con node has been started.')

        # Parameters
        self.turned_on = False
        self.last_pic_msg = None
        self.bridge = CvBridge()
        self.raw_image = None
        self.model = YOLO("/home/oliver/senv_ws/src/senv/senv/human.pt")
        self.middle_x = 0.0
        self.crossed = False
        self.saw_person_once = False
        self.start_pos = -1
        self.count = 0
        self.turned = False
        self.threshold = 641
        self.seen_sign = False
        # QOS Policy Setting
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)

        # Subscribing to camera and laser topics
        self.subscriber_pic = self.create_subscription(
            Pic,
            'pic',
            self.pic_callback,
            qos_profile=qos_policy
        )
        self.subscriber_pic  # prevent unused variable warning

        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            qos_profile=qos_policy)
        self.subscription  # prevent unused variable warning

        self.subscriber_laser = self.create_subscription(
            Laser,
            'laser',
            self.laser_callback,
            qos_profile=qos_policy
        )
        self.subscriber_laser  # prevent unused variable warning

        self.crosswalk_task_server_ = ActionServer(
            self,
            ConTask,
            "crosswalk_task",
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self.publisher_driver = self.create_publisher(Move, "drive", 1)

    def execute_callback(self, goal_handle: ServerGoalHandle):

        # Get request from goal
        target = goal_handle.request.start_working
        info = goal_handle.request.info
        self.get_logger().info("starting crosswalk server")

        # Turn in action server
        self.turned_on = target

        # Execute code as long as node is turned on
        while self.turned_on is True:
            self.datahandler(info)

        self.last_pic_msg = None
        self.raw_image = None
        self.middle_x = 0.0
        self.crossed = False
        self.stop_the_turn = False
        self.saw_person_once = False
        self.start_pos = -1
        self.count = 0
        self.turned = False
        self.threshold = 641
        self.seen_sign = False
        self.get_logger().info("Hand back")

        # Final Goal State
        goal_handle.succeed()

        # Result
        result = ConTask.Result()
        result.finished = True
        return result

    def cancel_callback(self, goal_handle):
        if self.turned_on is True:
            return CancelResponse.REJECT
        else:
            return CancelResponse.ACCEPT

    def pic_callback(self, msg):
        if self.turned_on is False:
            return

        self.last_pic_msg = msg

    def image_callback(self, msg):
        if self.turned_on is False:
            return
        self.raw_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def laser_callback(self, msg):
        if self.turned_on is False:
            return

    def datahandler(self, info):
        if self.last_pic_msg is None or self.raw_image is None:
            return
        msg = Move()
        person_there = self.detect_human()
        middle_x = self.middle_x
        self.get_logger().info("Human: " + str(person_there))
        self.get_logger().info(f"Detected Human at x: {self.middle_x}")

        if self.saw_person_once is False and person_there:
            msg.follow = False
            msg.speed = 0.0
            msg.turn = 0
            self.publisher_driver.publish(msg)
            self.detect_human()
            self.start_pos = self.middle_x
            self.saw_person_once = True
            person_from_center = abs(middle_x - 320)
            self.threshold = person_from_center + 75

        if self.last_pic_msg.sign == "crosswalk" and person_there:

            msg.follow = False
            msg.speed = 0.0
            msg.turn = 0
            self.get_logger().info("Crosswalk and Human")
            self.publisher_driver.publish(msg)
            self.seen_sign = True
            if self.crossed is False:
                self.get_logger().info(f"Starting Position: {self.start_pos}")
                if 10 >= abs(self.threshold - abs(self.start_pos-middle_x)) >= 0 and person_there:
                    self.get_logger().info("Crossing Far")
                    self.crossed = True
                    msg.follow = True
                    msg.speed = 0.5
                    msg.turn = 0
                    self.publisher_driver.publish(msg)
                    self.turned_on = False
            else:
                msg.follow = True
                msg.speed = 0.5
                msg.turn = 0
                self.publisher_driver.publish(msg)
                self.turned_on = False
                return 
            # person_there = self.detect_human()

        elif self.last_pic_msg.sign == "crosswalk" and not person_there:
            msg.follow = True
            msg.speed = 0.5
            msg.turn = 0
            self.publisher_driver.publish(msg)
            self.get_logger().info("Crosswalk")
            self.count += 1
            '''
            if self.count > 15:
                msg.follow = False
                msg.override = True
                msg.speed = 1.0
                msg.speed_o = 0.0
                msg.turn_o = 0.2
                i = 0
                while not person_there and i < 10:

                    person_there = self.detect_human()
                    self.publisher_driver.publish(msg)
                    i += 1
                    self.get_logger().info("Biele in Hallo")
                    self.wait_ros2(0.1)
            '''

        elif (self.last_pic_msg != "crosswalk" or self.seen_sign) and person_there:
            msg.follow = False
            msg.speed = 0.0
            msg.turn = 0
            self.get_logger().info("Human")
            self.publisher_driver.publish(msg)
            if self.crossed is False:
                self.get_logger().info(f"Starting Position: {self.start_pos}")
                if 10 >= abs(self.threshold - abs(self.start_pos-middle_x)) >= 0 and person_there:
                    self.get_logger().info("Crossing close")
                    self.crossed = True
                    msg.follow = True
                    msg.speed = 0.5
                    msg.turn = 0
                    self.publisher_driver.publish(msg)
                    self.turned_on = False
            else:
                msg.follow = True
                msg.speed = 0.5
                msg.turn = 0
                self.publisher_driver.publish(msg)
                self.turned_on = False

        else:
            msg = Move()
            msg.follow = True
            msg.speed = 0.5
            msg.turn = 0
            self.publisher_driver.publish(msg)
            self.turned_on = False
            self.get_logger().info("Nothing")

    def detect_human(self):
        """
        Predict the class of an image using a pre-trained model.
        DO NOT USE cv2.imshow IN HERE OR IT WONT WORK
        """

        if self.raw_image is None:
            return

        # threshhold/least confidence of detected signs
        threshhold = 0.7
        # get the image from the camera as np.array
        image = self.raw_image
        # Prediction (Inference)
        results = self.model(image, verbose=False)
        # classlist from model (the names must match your `data.yaml`)
        class_names = self.model.names

        # IDs of the detected classes (e.g. 0, 1, 2 …)
        class_ids = results[0].boxes.cls.cpu().numpy().astype(int)
        # x and y coordinates of the bounding boxes, for position of human while crossing
        # folter for signs with a probability of at least 0.8
        high_conf_indices = [i for i, conf in enumerate(results[0].boxes.conf.cpu().numpy())
                             if conf > threshhold]
        # if there is no sign with at least 80% recognition
        if not high_conf_indices:
            return False
        xyxy = results[0].boxes.xyxy.cpu().numpy()
        filtered_class_ids = [class_ids[i] for i in high_conf_indices]
        detected_labels = [class_names[i] for i in filtered_class_ids]
        # when there is no sign detected, the list is empty
        if detected_labels == []:
            return False
        # self.get_logger().info(f"Detected Sign is {detected_labels}")
        else:
            x1, y1, x2, y2 = map(int, xyxy[0])
            self.middle_x = (x1 + x2) / 2
            '''
            label = f"{class_names[class_ids[0]]}: {results[0].boxes.conf.cpu().numpy()[0]:.2f}"

            # Zeichne Rechteck
            cv2.rectangle(image, (self.x1, y1), (self.x2, y2), (255, 0, 0), 2)
            # Zeichne Label
            cv2.putText(image, label, (self.x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            # Zeige das bearbeitete Bild mit nur den gefilterten Erkennungen
            cv2.imshow("YOLOv8-Erkennung (Confidence > 0.8)", image)
            cv2.waitKey(1)
            '''
            return True

    def wait_ros2(self, duration):
        """ROS 2-kompatibles Warten, ohne Callbacks zu blockieren."""
        start_time = self.get_clock().now().nanoseconds
        while (self.get_clock().now().nanoseconds - start_time) / 1e9 < duration:
            rclpy.spin_once(self, timeout_sec=0.1)


def main(args=None):
    rclpy.init(args=args)
    crosswalk_con = Crosswalk_con()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(crosswalk_con, executor=executor)
    except KeyboardInterrupt:
        crosswalk_con.destroy_node()
    finally:
        crosswalk_con.destroy_node()
        print('Shutting Down lane_con')


if __name__ == '__main__':
    main()

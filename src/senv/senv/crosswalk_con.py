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


class Crosswalk_con(Node):
    def __init__(self):
        super().__init__('crosswalk_con')
        self.get_logger().info('crosswalk_con node has been started.')

        # Parameters
        self.turned_on = False
        self.last_pic_msg = None
        self.bridge = CvBridge()
        self.raw_image = None
        self.model = YOLO("/home/lennart/ros2_ws/src/senv/senv/human.pt")
        self.x1 = 0
        self.x2 = 0

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

        if self.last_pic_msg.sign != "crosswalk":

            msg = Move()
            msg.follow = False
            msg.speed = 0.0
            msg.turn = 0
            self.publisher_driver.publish(msg)

            person_there = self.detect_human()
            self.get_logger().info("Human: " + str(person_there))

            if person_there is True:
                msg = Move()
                msg.follow = False
                msg.speed = 0.0
                msg.turn = 0
                self.publisher_driver.publish(msg)
            else:
                msg = Move()
                msg.follow = True
                msg.speed = 1.0
                msg.turn = 0
                self.publisher_driver.publish(msg)

                self.turned_on = False

        else:
            msg = Move()
            msg.follow = True
            msg.speed = 1
            msg.turn = 0
            self.publisher_driver.publish(msg)

    def detect_human(self):
        """
        Predict the class of an image using a pre-trained model.
        DO NOT USE cv2.imshow IN HERE OR IT WONT WORK
        """

        if self.raw_image is None:
            return

        # threshhold/least confidence of detected signs
        threshhold = 0.8
        # get the image from the camera as np.array
        image = self.raw_image
        # Prediction (Inference)
        results = self.model(image, verbose=False)
        # classlist from model (the names must match your `data.yaml`)
        class_names = self.model.names

        # IDs of the detected classes (e.g. 0, 1, 2 …)
        class_ids = results[0].boxes.cls.cpu().numpy().astype(int)
        # x and y coordinates of the bounding boxes, for position of human while crossing
        self.x1, y1, self.x2, y2 = results[0].boxes.xyxy.cpu().numpy()
        self.get_logger().info()(f"Detected Human at x1: {self.x1}, y1: {y1}, x2: {self.x2}, y2: {y2}")
        # folter for signs with a probability of at least 0.8
        high_conf_indices = [i for i, conf in enumerate(results[0].boxes.conf.cpu().numpy())
                             if conf > threshhold]
        # if there is no sign with at least 80% recognition
        if not high_conf_indices:
            return False

        filtered_class_ids = [class_ids[i] for i in high_conf_indices]
        detected_labels = [class_names[i] for i in filtered_class_ids]
        # when there is no sign detected, the list is empty
        if detected_labels == []:
            return False
        # self.get_logger().info(f"Detected Sign is {detected_labels}")
        else:
            return True


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

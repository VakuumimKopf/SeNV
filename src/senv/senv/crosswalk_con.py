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
import cv2
import numpy as np


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
        self.publisher_driver = self.create_publisher(Twist, 'driving', qos_profile=qos_policy)

        self.template = cv2.imread('human.jpg', cv2.IMREAD_GRAYSCALE)

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

    def datahandler(self):
        self.get_logger().info("Handling crosswalk data")
        stopmsg = Twist()
        stopmsg.angular.z = 0
        stopmsg.angular.x = 0
        self.get_logger().info("Suche nach Mensch")
        human_there = self.detect_human()
        if human_there:
            self.get_logger().info("Mensch wartet am Zebrastreifen")
            self.publisher_driver.publish(stopmsg)
        else:
            pass

    def detect_human(self):
        human = False
        if self.img != []:
            # get the image from the camera as np.array
            image = self.bridge.compressed_imgmsg_to_cv2(self.img, desired_encoding='passthrough')
            image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            template = self.template
            w, h = template.shape[::-1]
            res = cv2.matchTemplate(image_gray, template, cv2.TM_CCOEFF_NORMED)
            threshhold = 0.8
            loc = np.where(res >= threshhold)
            for pt in zip(*loc[::-1]):
                cv2.rectangle(image, pt, (pt[0] + w, pt[1] + h), (0, 255, 255), 2)
            cv2.imshow('Detected Human', image)
            if len(loc) != 0:
                human = True
        else:
            return ""

        return human


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

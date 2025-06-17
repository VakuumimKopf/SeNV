
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from senv_interfaces.msg import Pic, Laser
from senv_interfaces.action import ConTask
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup


class state_machine(Node):
    def __init__(self):
        super().__init__('state_machine')
        self.get_logger().info('state_machine node has been started.')

        self.state = ""
        self.debug_mode = True
        self.seen_red_once = False
        self.goal_handle_ = None
        self.cancel_await = True
        self.is_turned_on = True

        # Last incoming data
        self.last_pic_msg = None
        self.last_laser_msg = None

        # Action trigger parameter
        self.intersection_con_triggers = ["left", "right", "straight"]

        # Qos policy setting
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

        self.subscriber_laser = self.create_subscription(
            Laser,
            'laser',
            self.laser_callback,
            qos_profile=qos_policy
        )
        self.subscriber_laser  # prevent unused variable warning

        # Creating clients for actions
        self.intersection_client = ActionClient(
            self,
            ConTask,
            "intersection_task",
            callback_group=ReentrantCallbackGroup(),
        )

        self.park_client = ActionClient(
            self,
            ConTask,
            "park_task",
            callback_group=ReentrantCallbackGroup(),
        )

        self.obstacle_client = ActionClient(
            self,
            ConTask,
            "obstacle_task",
            callback_group=ReentrantCallbackGroup(),
        )

        self.lane_client = ActionClient(
            self,
            ConTask,
            "lane_task",
            callback_group=ReentrantCallbackGroup(),
        )

        self.crosswalk_client = ActionClient(
            self,
            ConTask,
            "crosswalk_task",
            callback_group=ReentrantCallbackGroup(),
        )

        # Topics to publish to
        self.publisher_state = self.create_publisher(String, "state", 1)

        # create timers for data handling
        self.line_timer_period = 0.1
        self.line_timer = self.create_timer(
            self.line_timer_period, self.status_evaluation)

        # dictionary for state to client mapping
        self.client_dict = {
            "obstacle": self.obstacle_client,
            "lane": self.lane_client,
            "park": self.park_client,
            "intersection": self.intersection_client,
            "crosswalk": self.crosswalk_client,
        }
        self.get_logger().info("State machine initialized")

    # region action client

    # Sending request to action server
    def send_goal(self, working_status, info, client: ActionClient):

        client.wait_for_server()

        goal = client._action_type.Goal()

        goal.start_working = working_status
        goal.info = info

        if self.debug_mode is True:
            self.get_logger().info("Sending data to server")

        self._send_goal_future = client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback),

        self._send_goal_future.add_done_callback(self.goal_response_callback)
        self.cancel_await = None

    # Awaiting response from action server if request accepted
    def goal_response_callback(self, future):
        self.goal_handle_: ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:  # if accepted call goal_result_callback
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)
 

    # Feedback from action server
    def feedback_callback(self, feedback):
        self.get_logger().info("Feedback: " + str(feedback))

    # Sending Shutdown to action server
    def shutdown_action(self):
        self.get_logger().info("Canceling action server...")
        cancel_future = self.goal_handle_.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_done)

    #  Waiting for confimation to shut down
    def cancel_done(self, cancel_future):
        cancel_response = cancel_future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
            self.cancel_await = True
        else:
            self.get_logger().info('Goal failed to cancel')
            self.cancel_await = False

    # Requesting Result from action server
    def goal_result_callback(self, future):
        result = future.result().result

        self.get_logger().info("Result:" + str(result.finished))
        self.is_turned_on = True
        self.cancel_await = None
        self.goal_handle_ = None

    # endregion

    def pic_callback(self, msg: Pic):

        self.last_pic_msg = msg

    def laser_callback(self, msg: Laser):

        self.last_laser_msg = msg

    # Determine the current status based on inputs
    def status_evaluation(self):

        # Ensure laser and pic message is available
        if self.last_laser_msg is None or self.last_pic_msg is None:
            return

        if self.is_turned_on is False:
            return

        # Get most recent msg objects
        last_pic_msg = self.last_pic_msg
        last_laser_msg = self.last_laser_msg

        # Get current state
        old_state = self.state
        new_state = None

        if last_laser_msg.front_distance <= 0.5 and last_laser_msg.front_distance != 0.0:

            new_state = "obstacle"
            info = ""

        elif last_pic_msg.sign == "":

            new_state = "lane"
            info = ""

        elif last_pic_msg.sign == "Park":

            new_state = "park"
            info = ""

        elif last_pic_msg.sign in self.intersection_con_triggers:

            new_state = "intersection"
            info = last_pic_msg.sign

        elif last_pic_msg.sign == "crosswalk":

            new_state = "crosswalk"
            info = ""

        elif last_pic_msg.sign == "red light" and self.seen_red_once is False:

            new_state = "wait_for_green"
            self.state = new_state
            return

        elif last_pic_msg.sign == "green light" and old_state == "wait_for_green":

            new_state = "lane"
            info = ""

        else:
            self.get_logger().info("Error in pic_callback string sign: false value")
        self.state = new_state
        if new_state != old_state:
            self.update_node_state(new_state, info)

    # Update node state if changed
    def update_node_state(self, state, info):

        if self.goal_handle_ is not None:
            self.shutdown_action()

            self.get_logger().info("Wait for cancel response")

            while self.cancel_await is None:
                pass

            if self.cancel_await is False:
                self.is_turned_on = False
                self.get_logger().info("Cancel denied")
                return

            self.get_logger().info("Cancel accepted")
            #  Start Action with paired action server

        if self.debug_mode is True:
            self.get_logger().info("Übergeben an: " + state)

        self.send_goal(True, info, self.client_dict[state])

        #  Update state of the node and publish
        self.state = state
        out = String()
        out.data = "Changed to " + str(state)
        self.publisher_state.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = state_machine()
    executor = MultiThreadedExecutor()

    try:
        rclpy.spin(node, executor=executor)

    except KeyboardInterrupt:
        node.destroy_node()
    finally:
        node.destroy_node()
        print('Shutting Down state_machine')


if __name__ == '__main__':
    main()

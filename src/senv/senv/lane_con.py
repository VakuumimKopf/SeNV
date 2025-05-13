
import rclpy
import time
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from senv_interfaces.msg import Pic, Laser
from senv_interfaces.action import ConTask
from std_msgs.msg import String
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, IntegerRange, FloatingPointRange
from senv.stopper import Stopper

def light_int_desc(desc):
    min_val=0 
    max_val=255 
    step=1
    return ParameterDescriptor(type= ParameterType.PARAMETER_INTEGER, description=desc, 
                                integer_range=[IntegerRange(from_value=min_val, to_value=max_val, step=step)])
def int_desc(desc):
    min_val=0
    max_val=1000
    step=1
    return ParameterDescriptor(type= ParameterType.PARAMETER_INTEGER, description=desc, 
                                integer_range=[IntegerRange(from_value=min_val, to_value=max_val, step=step)])
def float_desc(desc):
    min_val=0.0
    max_val=2.0
    step=0.001
    return ParameterDescriptor(type= ParameterType.PARAMETER_DOUBLE, description=desc, 
                                floating_point_range=[FloatingPointRange(from_value=min_val, to_value=max_val, step=step)])
def bool_desc(desc):
    return ParameterDescriptor(type=ParameterType.PARAMETER_BOOL, description = desc)
    
class lane_con(Node):
    def __init__(self):
        super().__init__('lane_con')
        self.get_logger().info('lane_con node has been started.')

        # Lane Holding Parameter
        self.declare_parameter('boundary_left', 100)  # 200 für 640px, 100 für 320
        self.declare_parameter('boundary_right', 630)  # 440 für 640px, 220 für 320px
        self.declare_parameter('speed_drive', 0.115, float_desc('Fahr Geschwindigkeit Lane_Con'))
        self.declare_parameter('speed_turn', 0.3, float_desc("Drehgeschwindigkeit Lane_Con"))
        self.declare_parameter('light_lim', 100, light_int_desc("Helligkeits Grenzwert"))
        self.declare_parameter('middle_pix', 550, int_desc("Ausrichtungs Faktor (gering links hoch, rechts Maximal Kamera Auflösung)"))
        self.declare_parameter('offset_scale', 23, int_desc("Offset Scaling"))
       

        # Other Parameter
        self.last_spin = False  # False == gegen Uhrzeigersinn True==mit Uhrzeigersinn

        # State Parameters of the node
        self.is_turned_on = True
        self.last_state = ""

        # Last incoming msg
        self.last_pic_msg = Pic()
        self.last_laser_msg = 0.0

        # Action trigger parameter
        self.park_con_triggers = ["park_sign"]
        self.intersection_con_triggers = ["intersection_sign_left", "intersection_sign_right",
                                          "intersection_sign_straight"]

        # Qos policy setting
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

        # Creating clients for actions
        self.intersection_client = ActionClient(
            self,
            ConTask,
            "intersection_task"
        )

        self.park_client = ActionClient(
            self,
            ConTask,
            "park_task"
        )

        self.obstacle_client = ActionClient(
            self,
            ConTask,
            "obstacle_task"
        )

        # Topics to publish to
        self.publisher_ = self.create_publisher(Twist, "driving", 1)
        self.publisher_state = self.create_publisher(String, "state", 1)

        # create timers for data handling
        self.line_timer_period = 0.1
        self.line_timer = self.create_timer(
            self.line_timer_period, self.status_evaluation)

    # region action client

    # Sending request to action server
    def send_goal(self, working_status, client: ActionClient):

        # self.intersection_client_.wait_for_server()
        client.wait_for_server()

        # goal = ConTask.Goal()
        goal = client._action_type.Goal()

        goal.start_working = working_status

        self.get_logger().info("sending data to intersection server")
        client. \
            send_goal_async(goal).add_done_callback(self.goal_response_callback)

    # Awaiting response from action server if request accepted
    def goal_response_callback(self, future):
        self.goal_handle_: ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:  # if accepted call goal_result_callback
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)

    # Requesting Result from action server
    def goal_result_callback(self, future):
        result = future.result().result

        if result.finished is True:
            self.finish_move()

        self.get_logger().info("Result:" + str(result.finished))
        self.is_turned_on = True

    # endregion

    # Process the incoming message and decide whats to do --> give this information to sender
    def pic_callback(self, msg: Pic):

        self.last_pic_msg = msg

    # Process incoming message and decide what to do --> give this information to sender
    def laser_callback(self, msg: Laser):

        self.last_laser_msg = msg.front_distance
        # self.get_logger().info("laserscanner callback" + str(msg.front_distance))

    # Determine the current status based on inputs
    def status_evaluation(self):

        # Check if node has controll
        if self.is_turned_on is False:
            return

        # Ensure laser message is available
        if self.last_laser_msg is None:
            self.get_logger().info("No laser message received yet")
            return

        # Get most recent msg objects
        last_pic_msg = self.last_pic_msg
        last_laser_msg = self.last_laser_msg

        # Call lane_holding to get speed and turn value
        speed, turn = self.lane_holding(last_pic_msg.line)

        # Check laser always first because this is more reliable
        if last_laser_msg <= 0.5 and last_laser_msg != 0.0:

            self.get_logger().info("Übergeben an obstacle_con")

            # send action to obstacle_con
            self.send_goal(True, self.obstacle_client)

            # stop controll
            self.is_turned_on = False

            # Update state
            self.update_node_state("drive_around_obstacle")
            return

        elif last_pic_msg.sign == "":

            # do nothing special
            # lane holding algorithm
            self.driving_sender("drive_normal", speed, turn)

            # Update state
            self.update_node_state("drive_normal")
            return

        elif last_pic_msg.sign in self.park_con_triggers:

            self.get_logger().info("Übergeben an park_con")

            # send action to park_con
            self.send_goal(True, self.park_client)

            # stop controll
            self.is_turned_on = False

            # Update state
            self.update_node_state("parking")
            return

        elif last_pic_msg.sign in self.intersection_con_triggers:

            self.get_logger().info("Übergeben an intersection_con")

            # send action to intersection_con
            self.send_goal(True, self.intersection_client)

            # stop controll
            self.is_turned_on = False

            # Update state
            self.update_node_state("driving_in_intersection")
            return

        elif last_pic_msg.sign == "red light":

            self.get_logger().info("Halten an roter Ampel")
            self.driving_sender("stop", speed, turn)

            # Update state
            self.update_node_state("waiting_on_redlight")
            # return lane_holding

        elif last_pic_msg.sign == "green light":

            self.get_logger().info("Weiterfahren an grüner Ampel")
            self.driving_sender("drive_normal", speed, turn)

            # Update state
            self.update_node_state("driving_on_greenlight")
            return

        else:
            self.get_logger().info("Error in pic_callback string sign: false value")

            # Update state
            self.update_node_state("Error unkown state")
            return

    # Determine and return turn speed to hold the lane
    def lane_holding(self, data):

        # Get needed parameters
        speed_drive = self.get_parameter('speed_drive').get_parameter_value().double_value
        speed_turn = self.get_parameter('speed_turn').get_parameter_value().double_value
        middle_pix = self.get_parameter('middle_pix').get_parameter_value().integer_value
        offset_scaling = self.get_parameter('offset_scale').get_parameter_value().integer_value
        last_spin = self.last_spin

        #middle_pix = 550  # für 320px
        #offset_scaling = 23

        line_pos = data
        offset = abs(line_pos-middle_pix)

        speed = 0.0
        turn = 0.0

        if (line_pos == 0 and last_spin is False):

            # no white in bottom line of image
            speed = 0.0
            turn = -speed_turn/2

        elif (line_pos == 0 and last_spin is True):

            speed = 0.0
            turn = speed_turn/2

        else:
            speed = speed_drive

            # white pixel in image line / and bright_pos > boundary_left)
            if line_pos < (middle_pix):

                # left side is brightest: turn left '+' (Drehrichtung Roboter 30)
                turn = speed_turn * (offset/offset_scaling)
                self.last_spin = False

            # and bright_pos < boundary_right)
            elif line_pos > (middle_pix):

                # right side right turn '-'
                turn = -speed_turn * (offset/offset_scaling)
                self.last_spin = True

            else:
                # bright pixel is in the middle
                turn = 0.0

        return speed, turn

    def driving_sender(self, command, speed, turn):

        # commands: stop, drive_normal, drive_slow, drive_fast
        msg = Twist()

        if command == "stop":
            msg.angular.z = 0.0
            msg.linear.x = 0.0

            self.get_logger().info("stop command")

        elif command == "drive_normal":
            msg.linear.x = speed
            msg.angular.z = turn

            self.get_logger().info("drive_normal command")
        else:
            self.get_logger().info("Error lane_con: driving_sender: unkown command")

        self.publisher_.publish(msg)

    # Update node state if changed
    def update_node_state(self, state):

        if state != self.last_state:
            self.last_state == state
            out = String()
            out.data = "Changed to " + str(state)
            self.publisher_state.publish(out)

    def finish_move(self):
        self.driving_sender("drive_normal", 0.115, 0.0)
        time.sleep(1)


def main(args=None):
    rclpy.init(args=args)
    node = lane_con()
    
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.destroy_node()

    finally:
        #stop = Stopper()
        node.destroy_node()
        #stop.destroy_node()
        #rclpy.shutdown()
        print('Shutting Down Lane_con')
    
if __name__ == '__main__':
    main()

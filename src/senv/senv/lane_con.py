
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from senv_interfaces.msg import Pic,Laser
from senv_interfaces.action import ConTask
import numpy as np


class lane_con(Node):
    def __init__(self):
        super().__init__('lane_con')
        self.get_logger().info('lane_con node has been started.')
        
        self.declare_parameter('boundary_left', 100) # 200 für 640px, 100 für 320
        self.declare_parameter('boundary_right', 630) # 440 für 640px, 220 für 320px
        self.declare_parameter('speed_drive', 0.115)
        self.declare_parameter('speed_turn', 0.3)
        self.declare_parameter('light_lim', 100)
        self.declare_parameter('middle_tol', 20)
        self.declare_parameter('speed_turn_adjust',0.3)
        self.min_obstacle_distance = 0.2
        self.line_pos = 0
        self.hold_on_red = False
        self.last_spin = False # False == gegen Uhrzeigersinn True==mit Uhrzeigersinn

        self.is_turned_on = True
        self.park_con_triggers = ["park_sign"]
        self.intersection_con_triggers = ["intersection_sign_left", "intersection_sign_right", "intersection_sign_straight"]
        # Qos policy setting
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,depth=1)
        self.last_spin = False
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
        self.intersection_client_ = ActionClient(
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

        self.publisher_ = self.create_publisher(Twist, "driving", 1)

    # Sending request to action server 
    def send_goal(self, working_status, client: ActionClient):

        #self.intersection_client_.wait_for_server()
        client.wait_for_server()

        #goal = ConTask.Goal()
        goal = client._action_type.Goal()

        goal.start_working = working_status
        
        self.get_logger().info("sending data to intersection server")
        client. \
            send_goal_async(goal). \
                add_done_callback(self.goal_response_callback)

    # Awaiting response from action server if request accepted 
    def goal_response_callback(self, future):
        self.goal_handle_: ClientGoalHandle = future.result()
        if self.goal_handle_.accepted: # if accepted call goal_result_callback
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)

    # Requesting Result from action server
    def goal_result_callback(self, future):
        result = future.result().result
        self.get_logger().info("Result:" + str(result.finished))

    # Process the incoming message and decide whats to do --> give this information to sender    
    def pic_callback(self, msg: Pic):
        
        self.get_logger().info('Received message pic: ' + msg.sign)

        speed,turn = self.lane_holding(msg.line)


        if self.hold_on_red == False:
            if msg.sign in self.park_con_triggers:

                #send action to park_con
                #stop controll 
                return
            
            elif msg.sign in self.intersection_con_triggers:

                #send action to intersection_con
                #stop controll
                return
            
            elif msg.sign == "red light":
            
                self.hold_on_red = True
                self.get_logger().info("Halten an roter Ampel")
                self.driving_sender("stop",speed, turn)   
                return
             
            elif msg.sign == "":

                #do nothing special
                #lane holding algorithm
                self.driving_sender("drive_normal",speed, turn)
                return
                
            else:
                self.get_logger().info("Error in pic_callback string sign: false value")
                return
        else:
            if msg.sign == "green light":

                self.hold_on_red == False
                self.get_logger().info("Weiterfahren an grüner Ampel") 
                self.driving_sender("drive_normal",speed, turn)
                return

    def laser_callback(self, msg: Laser):
        # Process incoming message and decide what to do --> give this information to sender
        self.get_logger().info('Received message laser')
        if msg.distance <= self.min_obstacle_distance:
            #tell sender to stop
            pass 
        # obstacle avoidance algorithm

    # Determine and return turn speed to hold the lane
    def lane_holding(self, data):
        speed_drive = self.get_parameter('speed_drive').get_parameter_value().double_value
        speed_turn = self.get_parameter('speed_turn').get_parameter_value().double_value
        last_spin = self.last_spin
        '''
        # Needed parameters 
        boundary_left = self.get_parameter('boundary_left').get_parameter_value().integer_value
        boundary_right = self.get_parameter('boundary_right').get_parameter_value().integer_value

        # Formating data 
        img_row = data[boundary_left:boundary_right]
        
        # 20 greatest items sorted
        img_row_sorted = np.argsort(img_row)[-20:]

        # Values of 20 greatest items
        largest_values = img_row[img_row_sorted]

        # Index des mittleren Wertes der 20 größten im Teil-Array
        middle_index_in_subset = np.argsort(largest_values)[len(largest_values) // 2]

        # Index des mittleren Wertes im Original-Array
        middle_index_in_original = img_row_sorted[middle_index_in_subset]

        # Hellstes Element 
        brightest = max(img_row)

        line_pos = middle_index_in_original + boundary_left
        '''
        middle_pix = 550 # für 320px
        line_pos = data
        offset = abs(line_pos-middle_pix)
        offset_scaling = 23

        speed = 0.0
        turn = 0.0
        
        if(line_pos == 0  and last_spin == False):        
            # no white in bottom line of image 
            speed= 0.0
            turn = -speed_turn/2
            #self.get_logger().info('Hallo if1')
        elif(line_pos == 0 and last_spin == True):
            speed= 0.0
            turn = speed_turn/2
            #self.get_logger().info('Hallo elif1')
        else:
            speed=speed_drive
            #white pixel in image line / and bright_pos > boundary_left)
            if line_pos < (middle_pix) :
                #left side is brightest: turn left '+' (Drehrichtung Roboter 30)
                turn = speed_turn * (offset/offset_scaling)
                self.last_spin = False
                #self.get_logger().info('Hallo if2')
            # and bright_pos < boundary_right)
            elif line_pos > (middle_pix) :
                # right side right turn '-'
                turn = -speed_turn * (offset/offset_scaling)
                self.last_spin = True
                #self.get_logger().info('Hallo elif2')
            else :
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



def main(args=None):
    rclpy.init(args=args)
    node = lane_con()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

    try:
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print('Except in lane_con')
        node.destroy_node()

    finally:
        print('Shutting Down lane_con')

if __name__ == '__main__':
    main()

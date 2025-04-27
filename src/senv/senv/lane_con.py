
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from senv_interfaces.msg import Pic,Laser
from senv_interfaces.action import ConTask
from geometry_msgs.msg import Twist
import numpy as np
#
class lane_con(rclpy.node.Node):
    def __init__(self):
        super().__init__('follower')
        
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

        self.intersection_client_ = ActionClient(
            self,
            ConTask,
            "intersection_task"
        )

    # Sending request to action server 
    def send_goal(self, working_status):
        self.intersection_client_.wait_for_server()

        goal = ConTask.Goal()
        goal.start_working = working_status
        
        self.get_logger().info("sending data to intersection server")
        self.intersection_client_. \
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
        
    def pic_callback(self, msg):
        # Process the incoming message
        self.get_logger().info('Received message pic')
        # Add driving logic here

    def laser_callback(self, msg):
        # Define your callback function here
        boundary_left = 100
        boundary_right = 630
        speed_drive = 0.115
        speed_turn = 0.3
        light_lim = 100
        last_spin = self.last_spin
        self.get_logger().info('Received message laser')
        # obstacle avoidance
        img_row = self.img_row[boundary_left:boundary_right]

        #20 Größten Werte nach Größe geordnet
        img_row_sorted = np.argsort(img_row)[-20:]
        # Werte der 20 größten Elemente
        largest_values = img_row[img_row_sorted]

        # Index des mittleren Wertes der 20 größten im Teil-Array
        middle_index_in_subset = np.argsort(largest_values)[len(largest_values) // 2]

        # Index des mittleren Wertes im Original-Array
        middle_index_in_original = img_row_sorted[middle_index_in_subset]

        max_avg = np.mean(img_row_sorted)
        closest_value = img_row[middle_index_in_original]
        #middle_pix = 320 # für 640px x irgendwas
        middle_pix = 550 
        speed = 0.0
        turn = 0.0
        brightest = max(img_row)
        line_pos = middle_index_in_original + boundary_left

        #self.get_logger().info(f"Hellster Wert: {brightest}")
        #self.get_logger().info(f"Durchschnittlich hellster Wert: {closest_value}")
        #self.get_logger().info(f"Durchschnittlich hellster Index: {closest_index}")
        #self.get_logger().info(f"Helligkeit in Mitte: {img_row_original[middle_pix]})")
        #print(img_row)
        
        offset = abs(line_pos-middle_pix)
        offset_scaling = 23
        
        if(brightest < light_lim  and last_spin == False):        
            #no white in bottom line of image 
            speed= 0.0
            turn = -speed_turn/2
            #self.get_logger().info('Hallo if1')
        elif(brightest < light_lim and last_spin == True):
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

            # create message
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = turn
            # send message
        #self.get_logger().info('Line publisher')
        self.publisher_lane_con.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = lane_con()
    node.send_goal(True)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

    try:
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print('Except in lane_con')

    finally:
        node.destroy_node()
        print('Shutting Down lane_con')

if __name__ == '__main__':
    main()

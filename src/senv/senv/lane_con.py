import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from senv_interfaces.msg import Pic,Laser
from senv_interfaces.action import ConTask


class lane_con(Node):
    def __init__(self):
        super().__init__('lane_con')
        self.get_logger().info('lane_con node has been started.')
        
        self.is_turned_on = True
        self.park_con_triggers = ["park_sign"]
        self.intersection_con_triggers = ["intersection_sign_left", "intersection_sign_right", "intersection_sign_straight"]

        # Qos policy setting
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,depth=1)

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
    def send_goal(self, working_status, client: ActionClient):

        #self.intersection_client_.wait_for_server()
        client.wait_for_server()

        #goal = ConTask.Goal()
        goal = client._action_type()

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
        
    def pic_callback(self, msg: Pic):
        # Process the incoming message and decide whats to do --> give this information to sender
        self.get_logger().info('Received message pic')

        if msg.sign in self.park_con_triggers:
            #send action to park_con
            #stop controll 
            return
        elif msg.sign in self.intersection_con_triggers:
            #send action to intersection_con
            #stop controll
            return
        elif msg.line == "red light":
            #hold on line until green light
            pass 
        elif msg.line == "green light":
            #continue 
            #if red light mode also continue
            pass 
        elif msg.line == "":
            #do nothing special
            #lane holding algorithm
            pass 
        else:
            self.get_logger().info("Error in pic_callback string sign: false value")

    def laser_callback(self, msg: Laser):
        # Process incoming message and decide what to do --> give this information to sender
        self.get_logger().info('Received message laser')
        if msg.distance <= self.min_obstacle_distance:
            #tell sender to stop
            pass 
        # obstacle avoidance algorithm

    def driving_sender(command):
        # commands: stop, drive_normal, drive_slow, drive_fast
        pass 

def main(args=None):
    rclpy.init(args=args)
    node = lane_con()
    node.send_goal(True)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
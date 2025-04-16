import rclpy
import rclpy.executors
import rclpy.node
from std_msgs.msg import Int16
from senv.stopper import Stopper
from geometry_msgs.msg import Twist
from enum import Enum
from std_msgs.msg import String

class State(Enum):
    FollowLine = 1
    TurnOnObject = 2
    Error = 3

class Driver(rclpy.node.Node):
    def __init__(self):
        super().__init__('driver')

        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        self.stateint = 1 # 1 is for Follow 2 is for Turn 3 is Error
        self.previous_state = None
        self.line_msg = Twist()
        self.laser_msg = 1

        #self.state = State(1)

        self.subscription_laser = self.create_subscription(Int16, 'laser', self.laser_callback, qos_profile=qos_policy)
        self.subscription_line = self.create_subscription(Twist, 'line', self.line_callback, qos_profile=qos_policy)

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)
        self.publisher_state= self.create_publisher(String, 'state_info', 1)

        timer_period = 0.05
        self.my_timer = self.create_timer(timer_period, self.timer_callback )

    def laser_callback(self, msg):
        self.get_logger().info(f"Message: {msg}")
        if(msg.data == 1):
            #self.state = State.FollowLine
            self.stateint = 1
        elif(msg.data == 0):
            #self.state = State.TurnOnObject
            self.stateint = 2
        else:
            #self.state = State.Error
            self.stateint = 3
        self.laser_msg = msg

    def line_callback(self, msg):

        self.line_msg = msg

    def timer_callback(self):
        
            state_msg = String()
            #self.get_logger().info("DriverLogic")
            if(self.stateint == 1):
                #self.get_logger().info("using LineMsg")
                msg = self.line_msg
                state_msg.data = "Drive"
                
            elif(self.stateint == 2):
                #self.get_logger().info("using LaserMsg")
                msg = Twist()
                msg.linear.x = 0.0
                msg.angular.z = 0.0	
                state_msg.data = "Stop"

            else:
                self.get_logger().info('Error State entered in Driving Logic')
                msg = Twist()
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                state_msg.data = "Error"
            if self.stateint != self.previous_state:
                self.publisher_state.publish(state_msg)
            self.get_logger().info(f'Speed: {msg.linear.x} Turn: {msg.angular.z}')
            self.publisher_.publish(msg)
            self.previous_state = self.stateint
        

def main(args=None):

    print('Hi from Driver')
    rclpy.init(args=args, signal_handler_options=rclpy.SignalHandlerOptions.NO)
    driver_node = Driver()
    
    try:
        rclpy.spin(driver_node)

    except KeyboardInterrupt:
        driver_node.destroy_node()
        stop = Stopper()

    finally:
        driver_node.destroy_node()
        stop.destroy_node()
        rclpy.shutdown()
        print('Shutting Down Driver')


if __name__ == '__main__':
    main()

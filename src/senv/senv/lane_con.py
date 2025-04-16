import rclpy 
import rclpy.node
import cv2
import numpy as np

from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

class lane_con(rclpy.node.Node):

    def __init__(self):
        super().__init__('follower')
        
        # definition of the parameters that can be changed at runtime
        self.declare_parameter('boundary_left', 100) # 200 für 640px, 100 für 320
        self.declare_parameter('boundary_right', 630) # 440 für 640px, 220 für 320px
        self.declare_parameter('threshold_line', 100) #is now light_lim
        self.declare_parameter('speed_drive', 0.115)
        self.declare_parameter('speed_turn', 0.3)
        self.declare_parameter('light_lim', 100)
        self.declare_parameter('middle_tol', 20)
        self.declare_parameter('speed_turn_adjust',0.3)
        self.lineposition = 0

        # init openCV-bridge
        self.bridge = CvBridge()

        #self.img_row = np.array([0, 64, 128, 192, 255], dtype=np.uint8) # Beispiel
        self.img_row = np.random.randint(0, 256, 640, dtype=np.uint8)

        # definition of the QoS in order to receive data despite WiFi
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        
        self.last_spin = False # False == gegen Uhrzeigersinn True==mit Uhrzeigersinn
        # create subscribers for image data with changed qos
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.scanner_callback,
            qos_profile=qos_policy)
        self.subscription  # prevent unused variable warning

        # create publisher for driving commands
        self.publisher_lane_con = self.create_publisher(Twist, 'line', 1)

        # create timer to periodically invoke the driving logic
        self.timer_period = 0.1  # seconds
        self.my_timer = self.create_timer(self.timer_period, self.timer_callback)
    
    # handling received image data
    def scanner_callback(self, data):
        # convert message to opencv image
        img_cv = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding = 'passthrough')

        # convert image to grayscale
        img_gray = cv2.cvtColor(img_cv, cv2.COLOR_BGR2GRAY)

        # get image size
        height, width = img_gray.shape[:2]
        #self.get_logger().info(f"Image Höhe: {width} ")
        # get the lowest row from image
        img_row = img_gray[height-9,:]
        self.img_row = img_row 

        # show image
        cv2.imshow("IMG", img_gray)
        cv2.imshow("IMG_ROW", img_row)
        cv2.waitKey(1)


    # driving logic for lane_coning
    def timer_callback(self):
        #self.get_logger().info("FollowerLogic")
        
        boundary_left = self.get_parameter('boundary_left').get_parameter_value().integer_value
        boundary_right = self.get_parameter('boundary_right').get_parameter_value().integer_value
        speed_drive = self.get_parameter('speed_drive').get_parameter_value().double_value
        speed_turn = self.get_parameter('speed_turn').get_parameter_value().double_value
        speed_turn_adjust = self.get_parameter('speed_turn_adjust').get_parameter_value().double_value
        light_lim = self.get_parameter('light_lim').get_parameter_value().integer_value
        middle_tol = self.get_parameter('middle_tol').get_parameter_value().integer_value
        last_spin = self.last_spin
        #self.get_logger().info(f"Länge cam array: {len(img_row)}")
        img_row_original = self.img_row
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
        middle_pix = 550 # für 320px
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

    try:
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print('Except in lane_con')

    finally:
        node.destroy_node()
        print('Shutting Down lane_con')

if __name__ == '__main__':
    main()

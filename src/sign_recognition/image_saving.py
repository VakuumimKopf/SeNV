# Use following function to create dataset of raw images

def image_saving(self):
    # Save the image to a file.

    # Change to directory of your choice
    # Change the directory and sign as needed
    # 0 output
    # directory = "/home/lennart/ros2_ws/src/senv/Images/left"
    # sign = "left"
    # 1 output
    # directory = "/home/lennart/ros2_ws/src/senv/Images/right"
    # sign = "right"
    # 2 output
    # directory = "/home/lennart/ros2_ws/src/senv/Images/straight"
    # sign = "straight"
    # 3 output
    # directory = "/home/lennart/ros2_ws/src/senv/Images/crosswalk"
    # sign = "crosswalk"
    # 4 output
    directory = "/home/lennart/ros2_ws/src/senv/Images/park"
    sign = "park"

    # Get the current time
    current_time = self.get_clock().now().to_msg()

    # Format the time as a string
    time_str = f"{current_time.sec}_{current_time.nanosec}"

    # Create the filename
    filename = f"image_{sign}_{time_str}.jpg"

    # Convert the image to a format that can be saved
    img_cv = self.bridge.compressed_imgmsg_to_cv2(self.img, desired_encoding='passthrough')
    os.chdir(directory)
    # Save the image
    cv2.imwrite(filename, img_cv)

    print("After saving image:")

    print(f'Successfully saved {time_str}')

    elapsed_time = time.time() - self.start_time
    if elapsed_time > 32:
        self.get_logger().info("32 Sekunden erreicht – Programm wird beendet.")
        rclpy.shutdown()
    return ""

#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32

# Global variables
color_light = 2.0

# Callback function to receive the image
def camera_callback(msg):
    global cv_image

    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    # Create masks for the red, green, and yellow colors
    red_mask = cv2.inRange(cv_image, lower_red, upper_red)
    green_mask = cv2.inRange(cv_image, lower_green, upper_green)
    yellow_mask = cv2.inRange(cv_image, lower_yellow, upper_yellow)

    # Apply the masks to the original image
    red_result = cv2.bitwise_and(cv_image, cv_image, mask=red_mask)
    green_result = cv2.bitwise_and(cv_image, cv_image, mask=green_mask)
    yellow_result = cv2.bitwise_and(cv_image, cv_image, mask=yellow_mask)

    # Convert the results to grayscale
    red_gray = cv2.cvtColor(red_result, cv2.COLOR_BGR2GRAY)
    green_gray = cv2.cvtColor(green_result, cv2.COLOR_BGR2GRAY)
    yellow_gray = cv2.cvtColor(yellow_result, cv2.COLOR_BGR2GRAY)

    # Apply a Gaussian blur to reduce noise
    red_gray = cv2.GaussianBlur(red_gray, (5, 5), 0)
    green_gray = cv2.GaussianBlur(green_gray, (5, 5), 0)
    yellow_gray = cv2.GaussianBlur(yellow_gray, (5, 5), 0)

    # Detect circles in the results
    red_circles = cv2.HoughCircles(red_gray, cv2.HOUGH_GRADIENT, 1, 20,
                                  param1=50, param2=30, minRadius=0, maxRadius=0)
    green_circles = cv2.HoughCircles(green_gray, cv2.HOUGH_GRADIENT, 1, 20,
                                    param1=50, param2=30, minRadius=0, maxRadius=0)
    yellow_circles = cv2.HoughCircles(yellow_gray, cv2.HOUGH_GRADIENT, 1, 20,
                                      param1=50, param2=30, minRadius=0, maxRadius=0)

    # Draw the detected circles on the results
    if red_circles is not None:
        color_light = 0.0
        red_circles = np.round(red_circles[0, :]).astype("int")
        for (x, y, r) in red_circles:
            cv2.circle(cv_image, (x, y), r, (0, 0, 255), 2)

    if yellow_circles is not None:
        color_light = 1.0
        yellow_circles = np.round(yellow_circles[0, :]).astype("int")
        for (x, y, r) in yellow_circles:
            cv2.circle(cv_image, (x, y), r, (255, 0, 0), 2)

    if green_circles is not None:
        color_light = 2.0
        green_circles = np.round(green_circles[0, :]).astype("int")
        for (x, y, r) in green_circles:
            cv2.circle(cv_image, (x, y), r, (0, 255, 0), 2)


# Stop Condition
def stop():
  print("Stopping")

if __name__=='__main__':
    print("The Traffic Light is Running")
    
    # Initialize and Setup node at 100Hz
    rospy.init_node("traffic_light")
    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    # Setup Publishers and Suscribers
    colors_pub = rospy.Publisher("colors", Float32, queue_size=1)
    rospy.Subscriber("/video_source/raw", Image, camera_callback)

    # Define the ranges of red, green, and yellow colors in the BGR format
    lower_red = np.array([0, 0, 100])
    upper_red = np.array([50, 50, 255])
    lower_green = np.array([0, 100, 0])
    upper_green = np.array([50, 255, 50])
    lower_yellow = np.array([0, 200, 200])
    upper_yellow = np.array([50, 255, 255])

    #Run the node
    while not rospy.is_shutdown():
        colors_pub.publish(color_light)
        rate.sleep()
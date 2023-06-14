#!/usr/bin/env python2.7
import rospy
import cv2
import numpy as np
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32

#Global Variables
error = 0.0

def camera_callback(msg):
    global error
    
    global cv_image
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    # Resize the image to 640x480
    resized = cv2.resize(image, (640, 480))

    x1 = 140  # x-coordinate of the top-left corner of the ROI
    y1 = 420  # y-coordinate of the top-left corner of the ROI
    x2 = 500  # Width of the ROI
    y2 = 480  # Height of the ROI

    # Define the region of interest (ROI)
    roi = resized[y1:y2, x1:x2]

    #Center of the puzzlebot
    puzzlebot_x = (x2-x1) /2

    #Mask to detect color black o similar
    blanco = np.array([120, 120, 120])
    negro = np.array([0, 0 ,0])

    #Detect colors in the region of interest 
    roi = cv2.inRange(roi, negro, blanco)

    #Invert colors in roi
    roi = 255-roi

    # Apply a Gaussian Blur filterx
    blurred = cv2.GaussianBlur(roi, (3, 3), 0)

    #Average of colors detected
    average = np.mean(blurred, axis=0)
    
    #Array of center line
    black_segment = []

    #Find center line
    for x in range(len(average)-1):
        if average[x] == np.min(average) or average[x] < np.min(average) + 10:
            black_segment.append(x)

    if(len(black_segment) > 1): #If line is detected
        #Center of the line
        midpoint = (max(black_segment) - min(black_segment))/2
        line_x= black_segment[midpoint]

        # #Center of the line
        # line_x = black_segment[(len(black_segment)-1)/2]
        
    else: 
        line_x = puzzlebot_x

    cv2.circle(blurred, (line_x, 40), 5, (255, 255, 255), -1)

    print("Center Line :" + str(line_x))
    print("Center Image :" + str(puzzlebot_x))

    #Error from center of the line
    error = puzzlebot_x - line_x


    print("Error: " + str(error))

    #Publish image
    procesed_pub.publish(bridge.cv2_to_imgmsg(blurred, 'mono8'))


# Stop Condition
def stop():
  print("Stopping")

if __name__=='__main__':

    print("Line Follower is Running")
    
    # Initialize and Setup node at 100Hz
    rospy.init_node("Follower")
    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    # Setup Publishers and Suscribers
    instructor_pub = rospy.Publisher("Instructor", Float32, queue_size=10)
    procesed_pub = rospy.Publisher("processed", Image, queue_size=1)
    rospy.Subscriber("video_source/raw", Image, camera_callback)

    #Run the node
    while not rospy.is_shutdown():
        instructor_pub.publish(error)
        rate.sleep()
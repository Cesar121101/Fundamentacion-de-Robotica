#!/usr/bin/env python2.7
import rospy
import cv2
import numpy as np
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32

error = 0.0

def camera_callback(msg):
    global error
    
    global cv_image
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    # Resize the image to 640x480
    resized = cv2.resize(image, (640, 480))

    x1 = 60  # x-coordinate of the top-left corner of the ROI
    y1 = 350  # y-coordinate of the top-left corner of the ROI
    x2 = 580  # Width of the ROI
    y2 = 480  # Height of the ROI

    # Define the region of interest (ROI)
    roi = resized[y1:y2, x1:x2]

    blanco = np.array([110, 110, 110])
    negro = np.array([0, 0 ,0])

    roi = cv2.inRange(roi, negro, blanco)

    roi = 255-roi

    # Apply a Gaussian Blur filterx
    blurred = cv2.GaussianBlur(roi, (5, 5), 0)

    average = np.mean(blurred, axis=0)
    
    black_segment = []
    segment_start = None
    for x in range(len(average)-1):
        if average[x] == np.min(average) or average[x] < np.min(average) + 10:
            black_segment.append(x)
    
    midpoint = (max(black_segment) - min(black_segment))/2

    line_x= black_segment[midpoint]

    cv2.circle(blurred, (line_x, 100), 5, (255, 255, 255), -1)


    puzzlebot_x = (x2-x1) /2

    print("Center Line :" + str(line_x))
    print("Center Image :" + str(puzzlebot_x))

    error = puzzlebot_x - line_x

    print("Error: " + str(error))

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
    
    # camera_callback()


    #Run the node
    while not rospy.is_shutdown():
        instructor_pub.publish(error)
        rate.sleep()
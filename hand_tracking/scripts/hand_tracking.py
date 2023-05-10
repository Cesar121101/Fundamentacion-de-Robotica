#!/usr/bin/env python3.7
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import mediapipe as mp
from std_msgs.msg import Float32

#Global variables
mp_hands = mp.solutions.hands
hands = mp_hands.Hands()

mp_drawing = mp.solutions.drawing_utils

cap = cv2.VideoCapture(0)

# Create tracker
tracker = cv2.TrackerCSRT_create()

# Read first frame
ret, frame = cap.read()

# Define object to track
bbox = cv2.selectROI(frame, False)

# Initialize tracker
tracker.init(frame, bbox)

# Stop Condition
def stop():
  print("Stopping")

if __name__=='__main__':
    print("Hand Tracking is Running")
    # Initialize and Setup node at 100Hz
    rospy.init_node("Hand_Tracking")
    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    # Setup Publishers and Suscribers
    object_pub = rospy.Publisher("object_coordinates", Float32, queue_size=10)
    finger_pub = rospy.Publisher("finger_coordinates", Float32, queue_size=10)

    # rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)

    #Run the node
    while not rospy.is_shutdown():
        # Read a frame from the video capture object
        ret, frame = cap.read()

        # Update tracker with new frame
        success, bbox = tracker.update(frame)

        # if success draw the box
        if success:
            # Box coordinates to int
            bbox = [int(i) for i in bbox]
            # Draw box
            cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[0]+bbox[2], bbox[1]+bbox[3]), (0, 255, 0), 2)
            # Calculate center coordinates
            x = int(bbox[0] + bbox[2]/2)
            y = int(bbox[1] + bbox[3]/2)
            # Draw a circle in the middle of the box
            cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)
            print("X,Y coordinates of Box:" + str(x) + (" ") + str(y))

        # Convert the frame to RGB
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Detect the hands in the frame
        results = hands.process(frame)
        
        # Check if any hands were detected
        if results.multi_hand_landmarks:
            # Loop through each detected hand
            for hand_landmarks in results.multi_hand_landmarks:
                
                mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                #Obtain x,y coordinates of the fingers
                x1, y1 = int(hand_landmarks.landmark[1].x * frame.shape[1]), int(hand_landmarks.landmark[1].y * frame.shape[1])
                x2, y2 = int(hand_landmarks.landmark[2].x * frame.shape[1]), int(hand_landmarks.landmark[2].y * frame.shape[1])

                print("X,Y coordinates of Finger 1:" + str(x1) + (" ") + str(y1))
                print("X,Y coordinates of Finger 2:" + str(x2) + (" ") + str(y2))

            cv2.imshow('MediaPipe Hands', frame)
            # Check if the user pressed the 'q' key
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        object_pub.publish(1.0)
        finger_pub.publish(2.0)
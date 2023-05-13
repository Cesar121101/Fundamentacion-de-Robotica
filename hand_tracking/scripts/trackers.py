#!/usr/bin/env python3.7
import rospy
import cv2
import numpy as np
import mediapipe as mp
from std_msgs.msg import Float32
from hand_tracking.msg import hand_cords

#Global variables
handObj = hand_cords()
handObj.x = 0.0
handObj.y = 0.0
handObj.status = "close"
handObj.orientation = "up"

#Import ML model
mp_hands = mp.solutions.hands
hands = mp_hands.Hands()
mp_drawing = mp.solutions.drawing_utils

#Capture video from the camera
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
    hand_pub = rospy.Publisher("hand_coordinates", hand_cords, queue_size=1)

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
            # print("X,Y coordinates of Box:" + str(x) + (" ") + str(y))

        # Convert the frame to RGB
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Detect the hands in the frame
        results = hands.process(frame)
        
        # Check if any hands were detected
        if results.multi_hand_landmarks:
            # Loop through each detected hand
            for hand_landmarks in results.multi_hand_landmarks:
                
                mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                #Obtain x,y coordinates of the Hand
                x1, y1 = int(hand_landmarks.landmark[9].x * frame.shape[1]), int(hand_landmarks.landmark[9].y * frame.shape[0])
                #Obtain x,y coordinates of the fingers to know if the hand is open or close
                x2, y2 = int(hand_landmarks.landmark[4].x * frame.shape[1]), int(hand_landmarks.landmark[4].y * frame.shape[0])
                x3, y3 = int(hand_landmarks.landmark[8].x * frame.shape[1]), int(hand_landmarks.landmark[8].y * frame.shape[0])
                #0btain coords of another finger to verify orientation
                x4, y4 = int(hand_landmarks.landmark[12].x * frame.shape[1]), int(hand_landmarks.landmark[12].y * frame.shape[0])
                
            #Show tracked frame
            cv2.imshow('MediaPipe Hands', frame)
            # Check if the user pressed the 'q' key
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        #Verify the status of the hand
        if((x3-x2 < 20) and (y3-y2 < 20)):
            handObj.status = "close"
        else: 
            handObj.status = "open"

        #Verify orientation of the hand
        if(y1-y4 > 20):
            handObj.orientation = "up"
        elif(x4-x1 > 20):
            handObj.orientation = "left"
        elif(y4-y1 > 20):
            handObj.orientation = "down"
        elif(x1-x4 > 20):
            handObj.orientation = "right"

        # Assing values to hand object
        handObj.x = x1
        handObj.y = y1

        # Publish the coordinates and the status
        hand_pub.publish(handObj)

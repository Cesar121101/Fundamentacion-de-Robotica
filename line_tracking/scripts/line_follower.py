#!/usr/bin/env python3.7
import rospy
import cv2
import numpy as np
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32
from std_msgs.msg import String

instruction = ""

#Add mesage to callback
def camera_callback():

    global instruction

    # global cv_image
    # bridge = CvBridge()
    # cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    image_file = "pista.jpg" 
    package_path = rospkg.RosPack().get_path('line_tracking')

    image_path = package_path + "/images/" + image_file

    try:
        image = cv2.imread(image_path)
    except Exception as e:
        print("Error al leer imagen")

    # Apply a Gaussian Blur filter
    blurred = cv2.GaussianBlur(image, (9, 9), 0)

    # Apply adaptive threshold to obtain a binary image
    gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
    binary = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 11, 4)

    # Apply Canny Edge Detector
    edges = cv2.Canny(gray, 100, 100)


    # Create a black mask with the same dimensions as the edges image
    mask = np.zeros_like(edges)

    # Define the coordinates of the area of interest (example: a rectangular area)
    x1, y1 = 600, 500  # Top-left corner
    x2, y2 = 900, 900  # Bottom-right corner

    # Set the area inside the specified coordinates as white in the mask
    mask[y1:y2, x1:x2] = 255

    # Apply the mask to the edge image
    masked_edges = cv2.bitwise_and(edges, mask)

    # Draw area of interest
    rectangle = cv2.rectangle(image, (x1, y1), (x2, y2), (0, 0, 255), 2)

    # Find the contours
    contours, _ = cv2.findContours(masked_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    widest_line_width = 0
    highest_line_height = 0
    widest_contours = []
    contour_size = []

    #Find the widthest and longest contour
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)

        # Calculate line width and height
        line_width = w
        line_height = h

        # Check if the contour meets the criteria
        if line_width > widest_line_width and line_height > highest_line_height:
            widest_line_width = line_width
            highest_line_height = line_height

    #Find the contours similar to the biggest contour
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)

        # Calculate line width and height
        line_width = w
        line_height = h
        
        # Collect all contours that meet the criteria
        if line_width >= widest_line_width * 0.04 and line_height >= highest_line_height * 0.1:
            # print(str(line_width) + " " + str(line_height))
            contour_size.append([x,y])
            widest_contours.append(contour)

    # Draw all widest line contours
    cv2.drawContours(image, widest_contours, -1, (0, 255, 0), 2)

    # Left rectangle
    x1_r1, x2_r1, y1_r1, y2_r1 = 550, 600, 700, 750
    rectangle = cv2.rectangle(image, (x1_r1,y1_r1), (x2_r1, y2_r1), (255, 0, 0), 2)
    roi1 = image[y1_r1:y2_r1, x1_r1:x2_r1]
    color_promedio1 = np.mean(roi1, axis=(0, 1))
    color_promedio_rgb1 = cv2.cvtColor(np.uint8([[color_promedio1]]), cv2.COLOR_BGR2RGB)
    if(color_promedio_rgb1.mean() > 127):
        color_promedio_rgb1 = [255, 255, 255]
    else: 
        color_promedio_rgb1 = [0, 0, 0]

    # Right rectangle
    x1_r2, x2_r2, y1_r2, y2_r2 = 900, 950, 700, 750
    rectangle = cv2.rectangle(image, (x1_r2,y1_r2), (x2_r2, y2_r2), (0, 255, 0), 2)
    roi2 = image[y1_r2:y2_r2, x1_r2:x2_r2]
    color_promedio2 = np.mean(roi2, axis=(0, 1))
    color_promedio_rgb2 = cv2.cvtColor(np.uint8([[color_promedio2]]), cv2.COLOR_BGR2RGB)
    if(color_promedio_rgb2.mean() > 127):
        color_promedio_rgb2 = [255, 255, 255]
    else: 
        color_promedio_rgb2 = [0, 0, 0]


    # Middle rectangle
    x1_r3, x2_r3, y1_r3, y2_r3 = 725, 775, 700, 750
    rectangle = cv2.rectangle(image, (x1_r3,y1_r3), (x2_r3, y2_r3), (0, 0, 255), 2)
    roi3 = image[y1_r3:y2_r3, x1_r3:x2_r3]
    color_promedio3 = np.mean(roi3, axis=(0, 1))
    color_promedio_rgb3 = cv2.cvtColor(np.uint8([[color_promedio3]]), cv2.COLOR_BGR2RGB)
    if(color_promedio_rgb3.mean() > 127):
        color_promedio_rgb3 = [255, 255, 255]
    else: 
        color_promedio_rgb3 = [0, 0, 0]

    print("Color promedio 1: ", color_promedio_rgb1)
    print("Color promedio 2: ", color_promedio_rgb2)
    print("Color promedio 3: ", color_promedio_rgb3)

    #Continue forward
    if(color_promedio_rgb1 == color_promedio_rgb2 and color_promedio_rgb1 != color_promedio_rgb3):
        instruction = "forward"
    elif(color_promedio_rgb2 == color_promedio_rgb3 and color_promedio_rgb3 != color_promedio_rgb1): #Turn left
        instruction = "left"
    elif(color_promedio_rgb1 == color_promedio_rgb3 and color_promedio_rgb3 != color_promedio_rgb2): #Turn right
        instruction = "right"
    else: 
        instruction = "stop"

    # Show the image with the enclosed line
    cv2.imshow('Enclosed Line', image)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

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
    instructor_pub = rospy.Publisher("Instructor", String, queue_size=10)
    # rospy.Subscriber("video_source/raw", Image, camera_callback)


    #Run the node
    while not rospy.is_shutdown():
        camera_callback()
        instructor_pub.publish(instruction)
        rate.sleep()
        print("")
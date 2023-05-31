#!/usr/bin/env python2.7
import rospy
import cv2
import numpy as np
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32
from std_msgs.msg import String

#Terminal en el puzzlebot 
# ssh puzzlebot@10.42.0.1
# roslaunch ros_deep_learning video_source.ros1.launch

#Terminal en la compu
# export ROS_IP=10.42.0.16
# export ROS_MASTER_URI=http://10.42.0.1:11311
# scp line_follower.py puzzlebot@10.42.0.1:/home/puzzlebot/catkin_ws/src/line_tracking/scripts 

# Global variables
instruction = ""
# video_writer = None

# Add mesage to callback
def camera_callback(msg):

    global instruction
    # global video_writer

    global cv_image
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    # Obtener las dimensiones de la imagen
    height, width, _ = image.shape

    # Apply a Gaussian Blur filter
    blurred = cv2.GaussianBlur(image, (13, 13), 0)

    # Apply adaptive threshold to obtain a binary image
    gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
    binary = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 11, 4)

    # Apply Canny Edge Detector
    edges = cv2.Canny(binary, 100, 100)

    # Create a black mask with the same dimensions as the edges image
    mask = np.zeros_like(edges)

    # Define the coordinates of the area of interest (example: a rectangular area)
    x1, y1 = 500, 500  # Top-left corner
    x2, y2 = 800, 750  # Bottom-right corner

    # Set the area inside the specified coordinates as white in the mask
    mask[y1:y2, x1:x2] = 255

    # Apply the mask to the edge image
    masked_edges = cv2.bitwise_and(edges, mask)

    # # Draw area of interest
    # cv2.rectangle(image, (x1, y1), (x2, y2), (0, 0, 255), 2)

    # Definir las coordenadas de la region central
    x1_centro = int(width / 2 + 80)
    x2_centro = int(width / 2 + 170)
    y1_centro = int(height / 2 + 200)
    y2_centro = int(height/ 2 + 400)
    x1_centro2 = int(width / 2 - 170)
    x2_centro2 = int(width / 2 - 80)

    # Dibujar un rectangulo para visualizar la region central
    cv2.rectangle(image, (x1_centro, y1_centro), (x2_centro, y2_centro), (255, 0, 0), 2)
    # Dibujar un rectangulo para visualizar la region central
    cv2.rectangle(image, (x1_centro2, y1_centro), (x2_centro2, y2_centro), (0, 255, 0), 2)

    # Obtener la region central de la imagen
    roi_centro = image[y1_centro:y2_centro, x1_centro:x2_centro]
    roi_centro2 = image[y1_centro:y2_centro, x1_centro2:x2_centro2]

    # Calcular el color promedio de la region central
    color_promedio_centro = np.mean(roi_centro, axis=(0, 1))
    color_promedio_rgb_centro = cv2.cvtColor(np.uint8([[color_promedio_centro]]), cv2.COLOR_BGR2RGB)

    # Calcular el color promedio de la region central
    color_promedio_centro2 = np.mean(roi_centro2, axis=(0, 1))
    color_promedio_rgb_centro2 = cv2.cvtColor(np.uint8([[color_promedio_centro2]]), cv2.COLOR_BGR2RGB)

    # Imprimir el color promedio
    print("Derecho: " + str(color_promedio_rgb_centro.mean()))
    print("Izquierdo: " + str(color_promedio_rgb_centro2.mean()))
    print("")
    
    umbral = 120
    if(color_promedio_rgb_centro.mean() > umbral and color_promedio_rgb_centro2.mean() > umbral):
        instruction = "forward"
    elif(color_promedio_rgb_centro.mean() < umbral and color_promedio_rgb_centro2.mean() > umbral):
        instruction = "right"
    elif(color_promedio_rgb_centro.mean() > umbral and color_promedio_rgb_centro2.mean() < umbral):
        instruction = "left"
    else: 
        instruction = "stop"

    # # Create video writer object
    # if video_writer is None:
    #     video_writer = cv2.VideoWriter('output.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 2, (width, height))

    # # Write image to video
    # video_writer.write(image)

# Stop Condition
def stop():
  print("Stopping")
#   if video_writer is not None:
#         video_writer.release()


if __name__=='__main__':

    print("Line Follower is Running")
    
    # Initialize and Setup node at 100Hz
    rospy.init_node("Follower")
    rate = rospy.Rate(80)
    rospy.on_shutdown(stop)

    # Setup Publishers and Suscribers
    instructor_pub = rospy.Publisher("Instructor", String, queue_size=10)
    rospy.Subscriber("video_source/raw", Image, camera_callback)

    #Run the node
    while not rospy.is_shutdown():
        instructor_pub.publish(instruction)
        rate.sleep()

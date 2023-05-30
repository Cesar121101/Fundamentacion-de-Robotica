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

instruction = ""

# Add mesage to callback
def camera_callback(msg):

    global instruction
    global video_writer

    global cv_image
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    # image_file = "pista1.jpg" 
    # package_path = rospkg.RosPack().get_path('line_tracking')

    # image_path = package_path + "/images/" + image_file

    # try:
    #     image = cv2.imread(image_path)
    # except Exception as e:
    #     print("Error al leer imagen")

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

    # # Find the contours (Jetson)
    # contours, _= cv2.findContours(masked_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # # Find the contours
    # # _, contours, hierarchy = cv2.findContours(masked_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # widest_line_width = 0
    # highest_line_height = 0
    # widest_contours = []
    # contour_size = []
    # sec_filter = []
    
    # #Find the widthest and longest contour
    # for contour in contours:
    #     x, y, w, h = cv2.boundingRect(contour)

    #     # Calculate line width and height
    #     line_width = w
    #     line_height = h

    #     # Check if the contour meets the criteria
    #     if line_height > highest_line_height:
    #         widest_line_width = line_width
    #         highest_line_height = line_height
    
    # # print("Ancho: " + str(widest_line_width))
    # # print("Alto: " + str(highest_line_height))

    # # # Draw all widest line contours
    # # cv2.drawContours(image, contours, -1, (0, 255, 0), 2)

    # #Find the contours similar to the biggest contour
    # for contour in contours:
    #     x, y, w, h = cv2.boundingRect(contour)

    #     # Calculate line width and height
    #     line_width = w
    #     line_height = h
        
    #     # Collect all contours that meet the criteria
    #     if ((line_width >= widest_line_width * 0.05 and line_height >= highest_line_height * 0.7) or (line_width == widest_line_width and line_height == highest_line_height)):
            
    #         centroid_x = (x + w/2) 
    #         centroid_y = (y + h/2)

    #         # Guarda las coordenadas del contorno y su centroide
    #         contour_size.append([x, y, centroid_x, centroid_y, w, h])
    #         widest_contours.append(contour)

    #         # Calcula los momentos del contorno
    #         # moments = cv2.moments(contour)
    #         # print(moments)
    #         # if(moments['m00'] > 0):
    #         #     # print(str(line_width) + " " + str(line_height))
    #         #     # Calcula los centroides del contorno
    #         #     centroid_x = int(moments['m10'] / moments['m00'])
    #         #     centroid_y = int(moments['m01'] / moments['m00'])

    # delete_index = []
    # index2 = 0

    # # print(contour_size)

    # for i in contour_size: 
    #     index = contour_size.index(i)
    #     if((index in delete_index) != True):
    #         for j in contour_size:
    #             index2 = contour_size.index(j)
    #             if((index2 in delete_index) != True):
    #                 # print("I: " + str(i) + "J: " + str(j))
    #                 if(i != j):
    #                     if(abs(i[2] - j[2]) > 40):
    #                         if((i in sec_filter) != True):
    #                             if(i[5] > j[5]):
    #                                 if ((i in sec_filter) != True): 
    #                                     sec_filter.append(i)
    #                                     # print("Agregar I")
    #                             else:
    #                                 if ((j in sec_filter) != True):
    #                                     sec_filter.append(j)
    #                                     # print("Agregar J") 
    #                         else:
    #                             if((j in sec_filter) != True):
    #                                 sec_filter.append(j)
    #                                 # print("Agregar J") 
    #                     else:
    #                         if(i[5] > j[5]):
    #                             # print("Borrar J")
    #                             try: 
    #                                 index = contour_size.index(j)
    #                                 delete_index.append(index)
    #                             except Exception as e:
    #                                 index = 0
    #                             try: 
    #                                 index = sec_filter.index(j)
    #                                 sec_filter.pop(index)
    #                             except Exception as e:
    #                                 index = 0
    #                         else: 
    #                             # print("Borrar I")
    #                             try:
    #                                 index = contour_size.index(i)
    #                                 delete_index.append(index)
    #                             except Exception as e:
    #                                 index = 0
    #                             try: 
    #                                 index = sec_filter.index(i)
    #                                 sec_filter.pop(index)
    #                             except Exception as e:
    #                                 index = 0
    #     # print(delete_index)
    # cont = 0

    # for i in delete_index:       
    #     contour_size.pop(i-cont)
    #     widest_contours.pop(i-cont)
    #     cont += 1
        
    # # print(sec_filter)
    # # print(contour_size)

    # highest = 0
    # highest2 = 0
    # cont = 0

    # #Finda largest
    # for i in sec_filter:
    #     if(i[5] > highest2): 
    #         if(i[5] > highest):
    #             highest = i[5]
    #         else: 
    #             highest2 = i[5]

    # for i in sec_filter: 
    #     if(i[5] == highest or i[5] == highest2):
    #         cont+=1
    #     else: 
    #         sec_filter.pop(cont)
    #         contour_size.pop(cont)
    #         widest_contours.pop(cont)
    #         if(len(sec_filter) == 2):
    #             break

    # print("Contornos F: "+ str(len(sec_filter)))
    # # print("Contornos I: " + str(len(contour_size)))
    # # print("Iniciales: " + str(len(widest_contours)))

    # # print(sec_filter)
    # # print("C: " + str(sec_filter))
    # for c in sec_filter: 
    #     cv2.rectangle(image, (c[0],c[1]), (c[0]+c[4], c[1]+c[5]), (255, 255, 255), 2)
    #     cv2.circle(image, (int(c[2]), int(c[3])), 5, (0, 255, 255), -1)

    # # Draw all widest line contours
    # cv2.drawContours(image, widest_contours, -1, (0, 255, 0), 2)
    # if(len(sec_filter) == 2):
    #     # Center of the line
    #     center_xl = (sec_filter[0][2] + sec_filter[1][2])/2
    #     center_yl = (sec_filter[0][3] + sec_filter[1][3])/2
    # else: 
    #     instruction = "stop"
    
    # # Obtiene las dimensiones de la imagen
    # height, width, _ = image.shape

    # # Calcula las coordenadas del centro
    # center_x = int(width / 2)

    # # cv2.circle(image, (int(center_xl), int(center_yl)), 5, (255, 0, 255), -1)
    # # cv2.circle(image, (int(center_x), int(center_yl)), 5, (255, 255, 0), -1)

    # if(abs(center_xl - center_x) < 55):
    #     instruction = "forward"
    # elif(center_xl - center_x > 55): 
    #     instruction = "right"
    # elif(center_xl - center_x < 55): 
    #     instruction = "left"
    # else: 
    #     instruction = "stop"

    # Left rectangle
    # Obtener las dimensiones de la imagen
    height, width, _ = image.shape

    # Definir las coordenadas de la region central
    x1_centro = int(width / 2 + 80)
    x2_centro = int(width / 2 + 170)
    y1_centro = int(height / 2 + 100)
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

    # Calcular el color promedio de la region centxral
    color_promedio_centro = np.mean(roi_centro, axis=(0, 1))
    color_promedio_rgb_centro = cv2.cvtColor(np.uint8([[color_promedio_centro]]), cv2.COLOR_BGR2RGB)

    # Calcular el color promedio de la region central
    color_promedio_centro2 = np.mean(roi_centro2, axis=(0, 1))
    color_promedio_rgb_centro2 = cv2.cvtColor(np.uint8([[color_promedio_centro2]]), cv2.COLOR_BGR2RGB)

    # Imprimir el color promedio
    print("Derecho: " + str(color_promedio_rgb_centro.mean()))
    # Imprimir el color promedio
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

    # # Guardar la imagen
    # cv2.imwrite("imagen_guardada.jpg", image)

    # if(color_promedio_rgb1.mean() > 20):
    #     color_promedio_rgb1 = [255, 255, 255]
    # else: 
    #     color_promedio_rgb1 = [0, 0, 0]

    # # Right rectangle
    # x1_r2, x2_r2, y1_r2, y2_r2 = 620, 670, 650, 700
    # cv2.rectangle(image, (x1_r2,y1_r2), (x2_r2, y2_r2), (0, 255, 0), 2)
    # roi2 = image[y1_r2:y2_r2, x1_r2:x2_r2]
    # color_promedio2 = np.mean(roi2)
    # color_promedio_rgb2 = cv2.cvtColor(np.uint8([[color_promedio2]]), cv2.COLOR_BGR2RGB)
    # print(color_promedio_rgb2.mean())
    # if(color_promedio_rgb2.mean() > 20):
    #     color_promedio_rgb2 = [255, 255, 255]
    # else: 
    #     color_promedio_rgb2 = [0, 0, 0]

    # print("Color promedio 1: ", color_promedio_rgb1)
    # print("Color promedio 2: ", color_promedio_rgb2)

    # #Continue forward
    # if(color_promedio_rgb1 == color_promedio_rgb2 and color_promedio_rgb1 == [0, 0, 0]):
    #     instruction = "forward"
    # elif(color_promedio_rgb2 == [255, 255, 255]): #Turn left
    #     instruction = "left"
    # elif(color_promedio_rgb1 == [255, 255, 255]): #Turn right
    #     instruction = "right"
    # else: 
    #     instruction = "stop"

    # Show the image with the enclosed line
    # cv2.imshow('Enclosed Line', image)

    # print("Instrction: " + str(instruction))

    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    
    video_writer.write(image)
    

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
    rospy.Subscriber("video_source/raw", Image, camera_callback)
    
    # camera_callback()


       #Run the node
    while not rospy.is_shutdown():
        instructor_pub.publish(instruction)
        rate.sleep()

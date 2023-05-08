#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Variables globales para el seguimiento de objetos
object_lower_color = np.array([0, 100, 100])
object_upper_color = np.array([10, 255, 255])
object_center = None
contador = 0

# Callback function
def image_callback(img_msg):
    global object_center
    
    bridge = CvBridge()
    # Convertir el mensaje de imagen a una imagen de OpenCV
    cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")

    # Redimensionar la imagen para reducir el tiempo de procesamiento
    small_frame = cv2.resize(cv_image, (0, 0), fx=0.25, fy=0.25)

    # Convertir la imagen a espacio de color HSV para el seguimiento de objetos
    hsv_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2HSV)

    # Crear una mascara para detectar el objeto de interes en la imagen
    mask = cv2.inRange(hsv_frame, object_lower_color, object_upper_color)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # Encontrar contornos en la mascara y determinar el centro del objeto de interes
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)
        if M["m00"] != 0:
            object_center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        else:
            object_center = None
    else:
        object_center = None

    # Dibujar un circulo en el centro del objeto de interes (si se encuentra)
    if object_center is not None:
        cv2.circle(small_frame, object_center, 5, (0, 0, 255), -1)

    # Mostrar la imagen con el objeto de interes destacado
    cv2.imshow("Hand Tracking", small_frame)
    cv2.waitKey(1)

    # Controlar el robot simulado en Gazebo con los movimientos detectados
    # Rvis controller

# Stop Condition
def stop():
    print("Stopping")

if __name__=='__main__':
    # Inicializar el nodo ROS
    rospy.init_node('hand_tracking')

    # Configurar la suscripcion a la imagen de la camara
    rospy.Subscriber("/usb_cam/image_raw", Image, image_callback)

    #Run the node
    while not rospy.is_shutdown():
        print("contador: \n" + str(contador))
        contador += 1
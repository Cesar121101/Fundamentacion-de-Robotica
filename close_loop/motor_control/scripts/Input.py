#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from motor_control.msg import motor_input

# Setup Variables, parameters and messages to be used

#Stop Condition
def stop():
  print("Stopping")

if __name__=='__main__':
    print("The Input Genertor is Running")
    
    #Initialise and Setup node
    rospy.init_node("input")
    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    #Setup Publisher
    pwm_pub = rospy.Publisher("cmd_pwm", Float32, queue_size=1) 

    previoustime = rospy.get_time() #Set Previous Time
    valoractual = 0.0 #Variable para guardar el valor del setpoint
    flag = 1 #Bandera para hacer cambiar el incremento del setpoint (Aumenta o disminuir valor)

	#Run the node
    while not rospy.is_shutdown():
        
        if rospy.get_param("Wave", "No param found") == 1.0:
            if(rospy.get_time() - previoustime >= 0.05):
                if(flag == 1): 
                    valoractual += 1
                    if(valoractual >= 255):
                        flag = 0
                elif(flag == 0):
                    valoractual -= 1
                    if(valoractual <= -255):
                        flag = 1
                previoustime = rospy.get_time() 
        
        elif rospy.get_param("Wave", "No param found") == 2.0:
            if(rospy.get_time() - previoustime >= 10):
                if(flag == 1): 
                    valoractual = 255
                    flag = 0
                elif(flag == 0):
                    valoractual = -255
                    flag = 1
                previoustime = rospy.get_time()
        
        else:
            valoractual = rospy.get_param("Step", "No step found")
            
        msg = valoractual
        pwm_pub.publish(msg)
        rate.sleep()
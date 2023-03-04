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


    #Set previous time 
    previoustime = rospy.get_time()
    valoractual = 0.0
    flag = 1
    dir = True

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

        msg= valoractual
        #msg = np.sin(rospy.get_time() * 0.05 * np.pi)+1
		# Publish message
        pwm_pub.publish(msg)
        rate.sleep()
#!/usr/bin/env python
import rospy
import numpy as np
from pid_control.msg import set_point

# Setup Variables, parameters and messages to be used (if required)

#Stop Condition
def stop():
 #Setup the stop message (can be the same as the control message)
  print("Stopping")

if __name__=='__main__':
    print("The Set Point Genertor is Running")

    #Initialise and Setup node
    rospy.init_node("Set_Point_Generator")
    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    #Setup Publishers and subscribers here
    setpoint_pub = rospy.Publisher("set_point", set_point , queue_size=1) 

    # Set the message
    msg = set_point()
    msg.setpoint = 0.0
    msg.time = rospy.get_time()

    #Set previous time 
    previoustime = rospy.get_time()
    flag = 1
    valoractual = 0.0

	#Run the node
    while not rospy.is_shutdown():
        msg = set_point()
        #msg.setpoint = rospy.get_param("Setpoint", "No setpoint found")
        #msg.setpoint = np.sin(rospy.get_time()*0.8)*4
        if(rospy.get_time() - previoustime >= 5):
            if(flag == 1): 
                valoractual = 5.0
                flag = 0
            elif(flag == 0):
                valoractual = -5.0
                flag = 1
            previoustime = rospy.get_time()
        msg.setpoint = valoractual
        msg.time = rospy.get_time()
		#Write your code here
        setpoint_pub.publish(msg)
        rate.sleep()
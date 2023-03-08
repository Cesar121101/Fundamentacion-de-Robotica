#!/usr/bin/env python
import rospy
import numpy as np
from challengeFinal.msg import set_point

#Stop Condition
def stop():
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

        # Change set_point every 5 seconds (for testing)
        if(rospy.get_time() - previoustime >= 5):
            if(flag == 1): 
                valoractual = 75.0
                flag = 0
            elif(flag == 0):
                valoractual = -75.0
                flag = 1
            previoustime = rospy.get_time()
        msg.setpoint = valoractual

        msg.time = rospy.get_time()
        # Publish the message
        setpoint_pub.publish(msg)
        rate.sleep()
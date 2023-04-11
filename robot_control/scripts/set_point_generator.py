#!/usr/bin/env python
import rospy
import numpy as np
from robot_control.msg import set_point

#Stop Condition
def stop():
  print("Stopping")

if __name__=='__main__':
    print("The Set Point Genertor is Running")

    #Initialise and Setup node with rate of 100Hz
    rospy.init_node("Set_Point_Generator")
    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    #Setup Publishers and subscribers here
    setpoint_pub = rospy.Publisher("set_point", set_point , queue_size=1) 

    # Create the message
    msg = set_point()
    msg.setpoint = 0.0
    msg.type = rospy.get_param("/type", "No type found")

    #Set previous time 
    previoustime = rospy.get_time()
    flag = 1
    valoractual = 0.0
    buffer = ""

	#Run the node loop
    while not rospy.is_shutdown():

        # Get caracteristics of input (of the setpoint function)
        amplitude = rospy.get_param("/Amplitude", "No step found") # Amplitude of the setpoint
        period = rospy.get_param("/Period", "No step found") # Period over which the Sine and Square functions run
        type = rospy.get_param("/type", "No type found") # Type of function, either Step, Square or Sine

        # Step
        if type == 1.0:
            valoractual = amplitude
        
        # Square
        elif type == 2.0:
            if(rospy.get_time() - previoustime >= period):
                if(flag == 1): 
                    valoractual = amplitude
                    flag = 0
                elif(flag == 0):
                    valoractual = -amplitude
                    flag = 1
                previoustime = rospy.get_time()

        # Sine
        else:
            valoractual = np.sin(rospy.get_time()*2*np.pi/period)*amplitude


        # Speed limit to prevent asking for more speed than it can hanndle
        if valoractual > 27.5:
            valoractual = 27.5

        elif valoractual < -27.5:
            valoractual = -27.5
        
        # Set message
        msg.setpoint = valoractual
        msg.type = type

        # Publish the message to topic 'set_point'
        setpoint_pub.publish(msg)
        rate.sleep()
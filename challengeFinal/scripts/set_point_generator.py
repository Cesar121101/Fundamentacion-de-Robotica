#!/usr/bin/env python
import rospy
import numpy as np
from challengeFinal.msg import set_point
from getkey import getkey, keys


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
    msg.type = rospy.get_param("type", "No type found")

    #Set previous time 
    previoustime = rospy.get_time()
    flag = 1
    valoractual = 0.0
    buffer = ""
	#Run the node
    while not rospy.is_shutdown():
        key = getkey(blocking=False)

        # EXTRA Keys
        if key == keys.UP:
            print("Amplitude:")
            print(rospy.get_param("/Amplitude", "..."))
            rospy.set_param("/Amplitude", (rospy.get_param("/Amplitude", "No step found") + 0.5))
         
        elif key == keys.DOWN:
            print("Amplitude:")
            print(rospy.get_param("/Amplitude", "..."))
            rospy.set_param("/Amplitude", (rospy.get_param("/Amplitude", "No step found") - 0.5))

        elif key == keys.LEFT:
            print("Period:")
            print(rospy.get_param("/Period", "..."))
            rospy.set_param("/Period", (rospy.get_param("/Period", "No period found") - 1))

        elif key == keys.RIGHT:
            print("Period:")
            print(rospy.get_param("/Period", "..."))
            rospy.set_param("/Period", (rospy.get_param("/Period", "No period found") + 1))

        elif key == "a":
            rospy.set_param("/type", 1.0)

        elif key == "s":
            rospy.set_param("/type", 2.0)

        elif key == "d":
            rospy.set_param("/type", 3.0)
        else:
            buffer += key



        # Get caracteristics of input
        amplitude = rospy.get_param("/Amplitude", "No step found")
        period = rospy.get_param("/Period", "No step found")
        type = rospy.get_param("/type", "No type found")

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


        # Speed limit
        if valoractual > 27.5:
            valoractual = 27.5

        elif valoractual < -27.5:
            valoractual = -27.5
        
        # Set message
        msg.setpoint = valoractual
        msg.type = type

        # Publish the message
        setpoint_pub.publish(msg)
        rate.sleep()
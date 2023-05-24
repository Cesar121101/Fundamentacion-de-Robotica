#!/usr/bin/env python3.7
import rospy
from std_msgs.msg import String

instruction = ""

def instruction_callback(msg):
    global instruction
    
    instruction = msg

# Stop Condition
def stop():
  print("Stopping")

if __name__=='__main__':
    print("Listener is Running")
    # Initialize and Setup node at 100Hz
    rospy.init_node("Listener")
    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    # Setup Publishers and Suscribers
    rospy.Subscriber("Instructor", String , instruction_callback)
    
    #Run the node
    while not rospy.is_shutdown():
       print("Instruction: " + str(instruction))
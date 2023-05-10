#!/usr/bin/env python3.7
import rospy
from hand_tracking.msg import hand_cords

handCords = hand_cords()

def hand_callback(msg):
    global handCords
    handCords.x = msg.x
    handCords.y = msg.y
    handCords.status = msg.status

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
    rospy.Subscriber("hand_coordinates", hand_cords, hand_callback)
    
    #Run the node
    while not rospy.is_shutdown():
       print("Hand X,Y: " + str(handCords.x) + " " + str(handCords.y))
       print("Status: " + str(handCords.status))
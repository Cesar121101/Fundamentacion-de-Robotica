#!/usr/bin/env python
import rospy
from hand_tracking.msg import hand_cords
from std_msgs.msg import String

msgrec = "0.0,0.0,open"
handC = hand_cords()

def callback(msg):
    global msgrec
    msgrec = msg.data 


if __name__ == '__main__':
    # Subscribe to the contact sensor topic
    sub = rospy.Subscriber('/wl', String, callback)
    hand_pub = rospy.Publisher("hand_coordinates", hand_cords, queue_size=1)

    rospy.init_node("talkertest")
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        handC.x = float(msgrec.split(",")[0])
        handC.y = float(msgrec.split(",")[1])
        handC.status = msgrec.split(",")[2]

        hand_pub.publish(handC)

#!/usr/bin/env python
import rospy
from std_msgs.msg import String

command = String()
index = 0
commands = ["460.0,147.0,open",
            "460.0,147.0,close",
            "180.0,411.0,close",
            "180.0,411.0,open",
            "460.0,260.0,open",
            "460.0,260.0,close",
            "180.0,270.0,close",
            "180.0,270.0,open",
            "460.0,401.0,open",
            "460.0,401.0,close",
            "180.0,157.0,close",
            "180.0,157.0,open",
            "460.0,147.0,open"]

if __name__ == '__main__':
    # Subscribe to the contact sensor topic
    pub = rospy.Publisher('/wl', String, queue_size=1)

    rospy.init_node("commands3cubes")
    rospy.Rate(100)

    while not rospy.is_shutdown():
        command.data = commands[index]
        pub.publish(command)
        print("Sending: ", command.data)
        rospy.sleep(15.)

        index += 1
        

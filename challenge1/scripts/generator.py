#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import Float32

if __name__ == '__main__':
    pub = rospy.Publisher("signal", Float32 , queue_size=10)
    pub2 = rospy.Publisher("time", Float32 , queue_size=10)
    rospy.init_node("signal_generator")
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        signal = np.sin(rospy.get_time())
        time = rospy.get_time()
        pub.publish(signal)
        pub2.publish(time)
        rate.sleep()
#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import Float32

signal_process = 0.0
time = 0.0

def callback_signal(msg):
    rospy.loginfo("Signal: %s", msg.data)
    global signal_process
    signal_process = ((msg.data*np.cos(np.pi) + np.cos(time)*np.sin(np.pi)) + 1)*0.5
    

def callback_time(msg):
    rospy.loginfo("Time: %s", msg.data)
    global time
    time = msg.data
    
if __name__ == '__main__':  
    pub = rospy.Publisher("proc_signal", Float32 , queue_size=10) 
    rospy.init_node("process")
    rospy.Subscriber("signal", Float32, callback_signal)
    rospy.Subscriber("time", Float32,callback_time)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        pub.publish(signal_process)
        rospy.loginfo("Process: %s", signal_process)
        rate.sleep()

    rospy.spin()
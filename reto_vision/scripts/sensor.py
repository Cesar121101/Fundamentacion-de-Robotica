#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ContactsState

def callback(data):
    print("...")
    # Process the contact sensor data
    for contact in data.states:
        print("Collision detected between", contact.collision1_name, "and", contact.collision2_name)

def listener():
    # Initialize the ROS node
    rospy.init_node('contact_sensor_listener')

    # Subscribe to the contact sensor topic
    sub = rospy.Subscriber('/gripper_1_vals', ContactsState, callback)

    rate = rospy.Rate(100)


    while not rospy.is_shutdown():
        print("")
        rate.sleep()

if __name__ == '__main__':
    listener()

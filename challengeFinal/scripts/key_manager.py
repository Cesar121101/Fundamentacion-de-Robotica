#!/usr/bin/env python
import rospy
from getkey import getkey, keys

if __name__ == '__main__':
    rospy.init_node("key_manager")
    rate = rospy.Rate(100)

    buffer = []
    print("-- Key Manager --")

    while not rospy.is_shutdown():
        key = getkey()

        # UP Amplitude ++
        if key == keys.UP:
            rospy.set_param("/Amplitude", (rospy.get_param("/Amplitude", "No step found") + 0.5))

        # DOWN Amplitude --
        elif key == keys.DOWN:
            rospy.set_param("/Amplitude", (rospy.get_param("/Amplitude", "No step found") - 0.5))

        # Period ++
        elif key == keys.LEFT:
            rospy.set_param("/Period", (rospy.get_param("/Period", "No period found") - 1))

        # Period --
        elif key == keys.RIGHT:
            rospy.set_param("/Period", (rospy.get_param("/Period", "No period found") + 1))

        # Types
        elif key == "a":
            rospy.set_param("/type", 1.0)

        elif key == "s":
            rospy.set_param("/type", 2.0)

        elif key == "d":
            rospy.set_param("/type", 3.0)


        else:
            buffer += key
        
        print("Amplitude:")
        print(rospy.get_param("/Amplitude", "..."))
        print("Period:")
        print(rospy.get_param("/Period", "..."))
        print("Type:")
        if rospy.get_param("/type", "...") == 1.0:
            print("Type Input: Step")
        elif rospy.get_param("/type", "...") == 2.0:
            print("Type Input: Square")
        else:
            print("Type Input: Sine")


        rate.sleep()
    
        
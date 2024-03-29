#!/usr/bin/env python
import rospy

if __name__ == '__main__':
    # Initialize the node 'input' at 100 Hz
    rospy.init_node("input")
    rate = rospy.Rate(100)

    print("-- User input --")

    selection = False   # To know if a selection was made

    while not rospy.is_shutdown():

        # Validate if the distance or time is not yet defined
        if selection == False:

            # The user choose square or points
            print("Choose 0 for square or 1 for points: ")
            type = input("Your choice: ")
            rospy.set_param("/type", int(type))

            # If the user wants square
            if type == 0:
                # The user choose to define distance of time
                print("\nChoose 0 to set the distance or 1 to set the time")
                var = input("Your choice: ")

                # If the user wants to define distance
                if var == 0:
                    user_dist = input("\nWrite the distance: ") # Get distance from user
                    rospy.set_param("/user_dist", float(user_dist)) # Set the distance
                    selection = True # The user already set a value

                elif var == 1:
                    user_time = input("\nWrite the time: ") # Get time from user
                    rospy.set_param("/user_time", float(user_time)) # Set the time
                    selection = True # The user already set a value
            
            # If the user wants points
            elif type == 1:
                num_points = input("\nHow many points do you want to set: ")
                points = rospy.get_param("/points", "No param found")
                for i in range(num_points):
                    print("Write the point %s", i)
                    x = input("X coordinate: ")
                    y = input("Y coordinate: ")
                    points.append[(float(x), float(y))]
                rospy.set_param("/points", points)
                selection = True

        # If the distance or time is already defined
        elif selection == True:

            # Validate the value of the parameters
            print("User distance: %s", rospy.get_param("/user_dist", "No param found"))
            print("User time: %s", rospy.get_param("/user_time", "No param found"))

            # Ask the user if he/she wants to provide another selection
            print("\n\nHave you finish setting the parameters? 0 for No, 1 for Yes")
            another = input("Your choice: ")

            # If the user wants to provide another selection
            if another == 0:
                selection = False   # Go to the beginning

            # If the user does not want to provide another selection
            elif another == 1:
                rospy.set_param("/user_finish", 1.0) # Set the user finish
                break   # Finish program
       
        rate.sleep()
    
        
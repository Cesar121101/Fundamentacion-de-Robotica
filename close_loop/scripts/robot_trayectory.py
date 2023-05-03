#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from control_msgs import msg

#Setup global variables
out = 0.0
currentTime = 0.0
msgRobot = Twist()
r = 0.05
l = 0.191
distance = 0
rotation = 0
commands = []
isPoints = False
velocity = 0.0
error = 0.0
robot_angle = 0.0
vectorL = 1
user_finish = 0
user_dist = 0
user_time = 0
type = 0
points = [[0.0,0.0]]
wr = 0
wl = 0
prevTime = 0.0
superError = 0.0
Kp = 0.5
Ki = 0.0
Kd = 0
errorDistance = 0.0
errorRotation = 0.0
prevError = 0.0
d_real = 0.0
omega_real = 0.0
isFinishedT = False

# PID function 
def PID(error):
    global currentTime
    global prevTime
    global superError
    global prevError

    dt = currentTime-prevTime
    
    # P
    P = Kp*error

    # I
    superError += error * dt
    I = superError*Ki

    # D
    D = Kd*((error-prevError)/dt)

    prevError = error

    return (P + I + D)

def get_inputs():
    # Global variables
    global user_finish
    global user_dist
    global user_time
    global type
    global points
    selection = False   # To know if a selection was made

    print("-- User input --")

    while (selection == False):
        # The user choose square or points
        print("Choose 0 for square or 1 for points: ")
        type = input("Your choice: ")
        type = int(type)

        # If the user wants square
        if type == 0:
            # The user choose to define distance of time
            print("\nChoose 0 to set the distance or 1 to set the time")
            var = input("Your choice: ")

            # If the user wants to define distance
            if var == 0:
                user_dist = input("\nWrite the distance: ") # Get distance from user
                user_dist = float(user_dist) # Set the distance
                selection = True # The user already set a value

            # If the user wants to define time
            elif var == 1:
                user_time = input("\nWrite the time: ") # Get time from user
                user_time = float(user_time) # Set the time
                selection = True # The user already set a value
        
        # If the user wants points
        elif type == 1:

            # Get the number of points
            num_points = input("\nHow many points do you want to set: ")

            # Ask the user for each point
            for i in range(int(num_points)):
                print("Write the point " + str(i))
                x = input("X coordinate: ")     # Get X coordinate
                y = input("Y coordinate: ")     # Get Y coordinate
                points.append((float(x), float(y))) # Add values to array of points
            selection = True    # The user already set a value

        # Validate the value of the parameters
        print("User distance: " + str(user_dist))
        print("User time: " + str(user_time))
        print("Points: ")
        print(points)

        # Ask the user if he/she wants to provide another selection
        print("\n\nHave you finish setting the parameters? 0 for No, 1 for Yes")
        another = input("Your choice: ")

        # If the user wants to provide another selection
        if another == 0:
            selection = False   # Go to the beginning

        # If the user does not want to provide another selection
        elif another == 1:
            user_finish = 1.0
            selection = True

# Make points
def calculate_points():
    # Get global variables
    global isPoints
    global velocity
    global type
    global user_dist
    global user_time
    global points
    global robot_angle

    # If the user decide a square
    if type == 0:
        # If the user defined the distance
        if user_dist > 0.0 and float(user_time) == 0.0:
            # Make a square with the distance of the user
            commands.append((float(user_dist), 0.0)) # Move 
            commands.append((0.0, 1.6))              # Turn 90 deg
            commands.append((float(user_dist), 0.0)) # ...
            commands.append((0.0, 1.6))
            commands.append((float(user_dist), 0.0))
            commands.append((0.0, 1.6))
            commands.append((float(user_dist), 0.0))
            commands.append((0.0, 1.6))
            velocity = 1.0

        # If the user defined the time
        else: 
            # Make a square of 2m
            commands.append((2.0, 0.0)) # Move 2 m
            commands.append((0.0, 0.5)) # Turn 90 deg
            commands.append((2.0, 0.0)) # ...
            commands.append((0.0, 0.5))
            commands.append((2.0, 0.0))
            commands.append((0.0, 0.5))
            commands.append((2.0, 0.0))
            commands.append((0.0, 0.5))

            # Calculate the velocity
            velocity = 8.0/float(user_time)

            # If the velocity exceeds the max velocity
            if velocity > 1:
                velocity = 1

    # If the user decide points
    elif type == 1:
        print("POINTS")
        print(points)

        # --- Follow a set of points ---
        for i in range(len(points)-1):
            # Distance
            distx = points[i+1][0] - points[i][0]
            disty = points[i+1][1] - points[i][1]
            b = np.sqrt(distx*distx+disty*disty)

            robotx = points[i][0]
            roboty = points[i][1]

            newpointx = vectorL*np.cos(robot_angle) + robotx
            newpointy = vectorL*np.sin(robot_angle) + roboty

            print("New point: x(" + str(newpointx) + "), y(" + str(newpointy)+ ")")

            distx2 = newpointx-points[i+1][0]
            disty2 = newpointy-points[i+1][1]
            a = np.sqrt(distx2*distx2 + disty2*disty2)

            print("a: "+str(a))
            c = vectorL
            print("c: " + str(c))
            print("b: " + str(b))

            op = (b*b+c*c-a*a)/(2*b*c)
            print(round(op,14))
            op = round(op,14)
            #val = 0
            #val = round(op,8)
            #print("val: " + str(val))
            angle = np.arccos(op)
            print("angle: "+ str(angle))
            print("robotangle: "+str(robot_angle))

            # Check which side to choose
            testPointX = b*np.cos(robot_angle)*np.cos(angle) - b*vectorL*np.sin(robot_angle)*np.sin(angle) + robotx
            testPointY = b*np.cos(robot_angle)*np.sin(angle) + b*vectorL*np.sin(robot_angle)*np.cos(angle) + roboty

            print("Test point: x(" + str(testPointX) + "), y(" + str(testPointY)+ ")")

            if (testPointX < points[i+1][0]+1 and testPointX > points[i+1][0]-1 )and (testPointY < points[i+1][1]+1 and testPointY > points[i+1][1]-1):
                print("First try :)")      
            else:
                print("Not right one")
                angle = 2*np.pi-angle
                
            commands.append((0.0,((angle)/np.pi)))
            commands.append((b, 0.0))
            print("Sent Angle: " + str(angle/np.pi))
            print("Sent Distance: " + str(b))
            print("")
            robot_angle += angle

            print(commands)

        # Set velocity to 1
        velocity = 1
    isPoints = True

# This function handles the info inside of the 'motor_output' topic
def motor_output_callback(msg):
    global motorOut
    motorOut = msg.data

def wr_callback(msg):
    global wr
    wr = msg.data

def wl_callback(msg):
    global wl
    wl = msg.data

# Stop Condition
def stop():
  print("Stopping")

if __name__=='__main__':
    print("The Controller is Running")
    
    # Initialize and Setup node at 100Hz
    rospy.init_node("controller")
    rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    # Setup Publishers
    input_pub = rospy.Publisher("motor_input", Float32 , queue_size=1)
    error_pub = rospy.Publisher("error", Float32 , queue_size=1) 
    cmd_vel = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    sub_wr = rospy.Subscriber("wr", Float32, wr_callback)
    sub_wl = rospy.Subscriber("wl", Float32, wl_callback)

    # Get information from the user
    get_inputs()

    # Set the current time
    startTime = rospy.get_time()

    #Run the node
    while not rospy.is_shutdown():
        
        # If we have information from the user
        if user_finish == 1.0 and isPoints == False:
            calculate_points()      # Calculate the points
            isPoints = True         # Flag to calculate only one time
            point = 0              # Manages which point we will focus on
            # error = user_dist       # Set initial error
            msgRobot.linear.x = 1   # Set initial velocity
            msgRobot.angular.z = 0  # Set initial rotation

            # Get the working Distance and Rotation for each command (point)
            distance = commands[point][0]
            rotation = commands[point][1]
            
            rate.sleep()            # Time for all the variables to change

        # If the calculated points exists
        if isPoints:
            rate.sleep() # make a wait for making sure previous movements had stopped
            prevTime = currentTime
            currentTime = rospy.get_time()  # Obtain the time

            #Verify that we just use the existing points
            if(point > len(commands)-1):
                point = len(commands)-1
                isFinishedT = True          #Flag of finished trayectory

            # # Get the working Distance and Rotation for each command (point)
            distance = commands[point][0]
            rotation = commands[point][1]

            # dt = currentTime-startTime
            dt = currentTime-prevTime

            #Calculate real distance and the real rotation
            d_real += r*((wr + wl)/2.0)*dt
            omega_real += ((r*((wr-wl)/l))/np.pi)*dt
            # omega_real += msgRobot.angular.z * dt

            #Calculate the distance and rotation error
            errorDistance = distance - d_real
            errorRotation = rotation - omega_real
            
            #If the trayectory is not finished the control value is obtain from the pid
            if(not(isFinishedT)):
                linearVelocity = PID(errorDistance)
                angularVelocity = 5*PID(errorRotation)
            else:   #Else we set the linear and angular velocity to 0
                linearVelocity = 0.0
                angularVelocity = 0.0

            #If we reach the point we reset real distance, real rotation and find the new point
            if errorDistance <= 0.01 and errorRotation <= 0.01:
                d_real = 0.0
                omega_real = 0.0
                point += 1
                rate.sleep()

            #The value of the linear and angular velocity is obtained from the PID
            msgRobot.linear.x = linearVelocity
            msgRobot.angular.z = angularVelocity

            #print("WR: " + str(wr))
            #print("WL: " + str(wl))
            #print("Distance: " + str(distance))
            #print("Rotation: " + str(rotation))
            #print("Real distance: " + str(d_real))
            #print("Real omega: " + str(omega_real))
            #print("Linear Velocity: " + str(linearVelocity))
            #print("Angular Velocity: " + str(angularVelocity))
            #print("Error Distance: " + str(errorDistance))
            #print("Error Rotation: " + str(errorRotation))
            #print("Point: " + str(point))
            #print("Commnads" + str(commands))
            #print(" ")
               
        # We handle the error as a topic in order to able to plot it
        error_pub.publish(error)
        input_pub.publish(out)
        cmd_vel.publish(msgRobot)
        rate.sleep()

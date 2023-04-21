#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from close_loop.msg import goals

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
isFinishMovement = 0.0
prevPoint = -1
point = -1

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

# Callback function of motor output
def motor_output_callback(msg):
    global motorOut
    motorOut = msg.data

def wr_callback(msg):
    global wr
    wr = msg.data

def wl_callback(msg):
    global wl
    wl = msg.data

def goals_callback(msg):
    global point
    global distance
    global rotation

    point = msg.point
    distance = msg.distance
    rotation = msg.rotation

def isFinish_callback(msg):
    global isFinishMovement
    isFinishMovement = msg.data

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
    cmd_vel = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    isFinish_pub = rospy.Publisher("isFinish", Float32, queue_size=1)
    rospy.Subscriber("isFinish", Float32, isFinish_callback)
    rospy.Subscriber("wr", Float32, wr_callback)
    rospy.Subscriber("wl", Float32, wl_callback)
    rospy.Subscriber("goals", goals, goals_callback)

    # Set the current time
    startTime = rospy.get_time()

    #Run the node
    while not rospy.is_shutdown():

        # If the calculated points exists
        if prevPoint != point and isFinishMovement == 0.0:
            rate.sleep() # make a wait for making sure previous movements had stopped
            prevTime = currentTime
            currentTime = rospy.get_time()  # Obtain the time

            dt = currentTime-prevTime

            #Calculate real distance and the real rotation
            d_real += r*((wr + wl)/2.0)*dt
            omega_real += ((r*((wr-wl)/l))/np.pi)*dt

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
                prevPoint = point
                point += 1
                isFinishMovement = 1.0
                rate.sleep()
            else:
                isFinishMovement = 0.0

            #The value of the linear and angular velocity is obtained from the PID
            msgRobot.linear.x = linearVelocity
            msgRobot.angular.z = angularVelocity

            # Print information
            print("WR: " + str(wr))
            print("WL: " + str(wl))
            print("Distance: " + str(distance))
            print("Rotation: " + str(rotation))
            print("Real distance: " + str(d_real))
            print("Real omega: " + str(omega_real))
            print("Linear Velocity: " + str(linearVelocity))
            print("Angular Velocity: " + str(angularVelocity))
            print("Error Distance: " + str(errorDistance))
            print("Error Rotation: " + str(errorRotation))
            print("Point: " + str(point))
            print(" ")
               
        # Publish all the topics
        input_pub.publish(out)
        cmd_vel.publish(msgRobot)
        isFinish_pub.publish(isFinishMovement)
        rate.sleep()
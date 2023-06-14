#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats 
from std_msgs.msg import Float32MultiArray
import time

#Setup global variables
msgRobot = Twist()
signal = []
len_signal = 0
error = 500
linearVelocity = 0.0
angularVelocity = 0.0
currentTime = 0.0
prevTime = 0.0
superError = 0.0
prevError = 0.0
prevAngularVel = 0.0
prevLinearVel = 0.0
right = False
left = False
current = 0
maxVel = 0.04
flag = True
semaforo = False
go = True

# PID function 
def PID(error):
    global currentTime
    global prevTime
    global superError
    global prevError

    dt = currentTime-prevTime
    
    # P
    P = 0.0001*error

    # I
    superError += error * dt
    I = superError*0

    # D
    D = 0.0001*((error-prevError)/dt)

    prevError = error

    print("PID: " + str(P+I+D))

    return (P + I + D)


# Callback function of motor output
def signals_callback(msg):
    global signal
    global len_signal
    signal = msg.data
    len_signal = signal[0]

def instruction_callback(msg):
    global error
    error = msg.data

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
    cmd_vel = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    rospy.Subscriber("Instructor", Float32 , instruction_callback)
    rospy.Subscriber("signal", Float32MultiArray , signals_callback)

    #Run the node
    while not rospy.is_shutdown():
        prevTime = currentTime
        currentTime = rospy.get_time()

        # New Code

        #SEMAFORO
        if (2.0 in signal[1:]) or (4.0 in signal[1:]) or ( 9.0 in signal[1:]):
            semaforo = True
            if (2.0 in signal[1:] or 9.0 in signal[1:]):
                go = True
                semaforo = False
            else:
                go = False

        
        print(current)
        if (0.0 in signal[1:]) or (3.0 in signal[1:]) or ( 5.0 in signal[1:]) :
            if (flag == True):
                current += 1
            flag = False
        
        if 6.0 in signal[1:]: #Stop or red
            print("1...")
            linearVelocity = 0.0
            angularVelocity = 0.0
            flag = True
        elif 8.0 in signal[1:]: #Working or yellow
            print("3...")
            maxVel = 0.02
            linearVelocity = maxVel
            angularVelocity = 0.0
            flag = True
        elif (error < 45 and error > -45): #Forward or green
            if current== 3: #Forward or green
                print("2...")
                if semaforo == True and go == False:
                    linearVelocity = 0.0
                elif semaforo== True and go==True:
                    maxVel = 0.04
                    linearVelocity = 0.06
                    angularVelocity = 0.0
                    msgRobot.linear.x = linearVelocity
                    msgRobot.angular.z = angularVelocity
                    cmd_vel.publish(msgRobot)
                    time.sleep(15)
                    linearVelocity = 0.0
                    angularVelocity = 0.0
                    msgRobot.linear.x = linearVelocity
                    msgRobot.angular.z = angularVelocity
                    cmd_vel.publish(msgRobot)
                    flag = True
                    semaforo = False
            else:
                linearVelocity = maxVel
                angularVelocity = 0.0

                #flag = True
        elif current == 1: #Left
            print("4...")
            if semaforo == True and go == False:
                linearVelocity = 0.0
            elif semaforo== True and go==True:
                left = True
                linearVelocity = 0.06
                angularVelocity = 0.0
                msgRobot.linear.x = linearVelocity
                msgRobot.angular.z = angularVelocity
                cmd_vel.publish(msgRobot)
                time.sleep(6)
                linearVelocity = 0.0
                angularVelocity = 0.5
                msgRobot.linear.x = linearVelocity
                msgRobot.angular.z = angularVelocity
                cmd_vel.publish(msgRobot)
                time.sleep(0.6)
                flag = True
                semaforo = False
        elif current == 2: #Right
            print("5...")
            if semaforo == True and go == False:
                linearVelocity = 0.0
            elif semaforo== True and go==True:
                maxVel = 0.04
                right = True
                linearVelocity = 0.06
                angularVelocity = 0.0
                msgRobot.linear.x = linearVelocity
                msgRobot.angular.z = angularVelocity
                cmd_vel.publish(msgRobot)
                time.sleep(12)
                linearVelocity = 0.0
                angularVelocity = -0.5
                msgRobot.linear.x = linearVelocity
                msgRobot.angular.z = angularVelocity
                cmd_vel.publish(msgRobot)
                time.sleep(0.6)
                semaforo = False
                flag = True
        else: #Track curve
            print("6...")
            linearVelocity = 0.017
            angularVelocity = PID(error)

        if(len_signal) > 0:
            print("Signal 1: " + str(signal[6]))
        if(len_signal) > 5:
            print("Signal 2: " + str(signal[12]))
        print("Linear Velocity: " + str(linearVelocity))
        print("Angular Velocity: " + str(angularVelocity))
        print(" ")
        print("Error: " + str(error))
        print("Sem: "+ str(semaforo) + ", Go: " + str(go))
        print("Flag: "+ str(flag))

        if linearVelocity > 0.4 or linearVelocity < -0.4: 
            linearVelocity = prevLinearVel
        if angularVelocity > 0.3 or angularVelocity < -0.3: 
            angularVelocity = prevAngularVel

        prevAngularVel = angularVelocity
        prevLinearVel = linearVelocity
        
        msgRobot.linear.x = linearVelocity
        msgRobot.angular.z = angularVelocity

        # Publish all the topics
        cmd_vel.publish(msgRobot)
        rate.sleep()
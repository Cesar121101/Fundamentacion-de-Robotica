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
maxVel = 0.05
slowVel = 0.02
state = "line"
detects = []
cont = 3
flag = True

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

    return (P + I + D)

def line(error):
    global maxVel
    global slowVel
    if error < 45 and error > -45: 
        return maxVel, 0.0
    else:
        return 0.017, PID(error)
    
def stop():
    return 0.0, 0.0

def slow(error):
    global slowVel
    if error < 45 and error > -45: 
        return slowVel, 0.0
    else:
        return 0.017, PID(error)

def signal_stop():
    return 0.0, 0.0

def semaforo():
    return 0.0, 0.0

def blue_sign_process(cont, prev):
    global msgRobot
    if "forward" in prev or "right" in prev or "left" in prev:
        return cont
    else:
        if cont == 1:
            print("TURNING LEFT")
            linearVelocity = 0.06
            angularVelocity = 0.0
            msgRobot.linear.x = linearVelocity
            msgRobot.angular.z = angularVelocity
            cmd_vel.publish(msgRobot)
            time.sleep(6.5)
            linearVelocity = 0.0
            angularVelocity = 0.5
            msgRobot.linear.x = linearVelocity
            msgRobot.angular.z = angularVelocity
            cmd_vel.publish(msgRobot)
            time.sleep(0.6)
        elif cont == 2:
            print("TURNING RIGHT")
            linearVelocity = 0.06
            angularVelocity = 0.0
            msgRobot.linear.x = linearVelocity
            msgRobot.angular.z = angularVelocity
            cmd_vel.publish(msgRobot)
            time.sleep(13)
            linearVelocity = 0.0
            angularVelocity = -0.5
            msgRobot.linear.x = linearVelocity
            msgRobot.angular.z = angularVelocity
            cmd_vel.publish(msgRobot)
            time.sleep(0.6)
        elif cont == 3:
            print("GOING FORWARD")
            linearVelocity = 0.06
            angularVelocity = 0.0
            msgRobot.linear.x = linearVelocity
            msgRobot.angular.z = angularVelocity
            cmd_vel.publish(msgRobot)
            time.sleep(14)
            linearVelocity = 0.0
            angularVelocity = 0.0
            msgRobot.linear.x = linearVelocity
            msgRobot.angular.z = angularVelocity
            cmd_vel.publish(msgRobot)

        else:
            print("ERROR")
        
    cont += 1
    return cont

# Change to readable values
def changevals(clas):
    if clas == 0.0:
        return "forward"
    elif clas == 1.0:
        return "giveway"
    elif clas == 2.0:
        return "green"
    elif clas == 3.0:
        return "left"
    elif clas == 4.0:
        return "red"
    elif clas == 5.0:
        return "right"
    elif clas == 6.0:
        return "stop"
    elif clas == 7.0:
        return "turnaround"
    elif clas == 8.0:
        return "working"
    elif clas == 9.0:
        return "yellow"


# Callback function of motor output
def signals_callback(msg):
    global signal
    global len_signal
    global detects
    global prev
    global flag
    flag = True
    prev = detects
    signal = msg.data
    len_signal = signal[0]
    len_signal = int(len_signal)
    detects = []
    if len_signal > 0:
        for senal in range(1,len_signal+1):
            detects.append(changevals(signal[senal*6]))
    
    print("DETECTS:")
    print(detects)


def instruction_callback(msg):
    global error
    error = msg.data

# Stop Condition
def stop():
    print("..")

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
        

        if(state == "line"):
            linearVelocity, angularVelocity = line(error)
            if "working" in detects:
                state = "slow"
            elif "stop" in detects:
                state = "stop"
            elif ("left" in detects or "right" in detects or "forward" in detects) and flag == True:
                state = "semaforo"
            else:
                state = "line"
        
        elif state == "slow":
            linearVelocity, angularVelocity = slow(error)
            if len(detects) > 0 and ("working" in detects) == False:
                state = "line"
            else:
                state = "slow"

        elif state == "stop":
            linearVelocity, angularVelocity = signal_stop()

        elif state == "semaforo":
            linearVelocity, angularVelocity = semaforo()
            if "rojo" in detects:
                state = "semaforo"
            elif "green" in detects or "yellow" in detects:
                state = "blue_sign"

        elif state == "blue_sign":
            cont = blue_sign_process(cont, prev)
            state = "line"
            flag = False
            
        if(len_signal) > 0:
            print("Signal 1: " + str(signal[6]))
        if(len_signal) > 5:
            print("Signal 2: " + str(signal[12]))
        
        print("State: ", state)
        print("Cont: " + str(cont))
        print("Linear Velocity: " + str(linearVelocity))
        print("Angular Velocity: " + str(angularVelocity))
        print("")

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

#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

# rostopic pub 
# /arm_controller/command 
# trajectory_msgs/JointTrajectory 
# '{joint_names: ["p0_joint","p1_joint","p2_joint","p3_joint","gripper_1","gripper_2"], points: [{positions: [0.0,2.3,-1.0,1.57,0.0627,-0.0627],time_from_start:[1.0,0.0]}]}' -1

recvmsg = ""
message = JointTrajectory()
points = JointTrajectoryPoint()
req = AttachRequest()

def activate_callback(msg):
    global recvmsg
    print(msg.data)
    recvmsg = msg.data


if __name__ == '__main__':
    pub = rospy.Publisher("/arm_controller/command", JointTrajectory, queue_size=10)
    sub_wl = rospy.Subscriber("wl", String, activate_callback)
    rospy.init_node("wobot")
    rate = rospy.Rate(100)


    while not rospy.is_shutdown():
        rospy.wait_for_service('/link_attacher_node/attach')
        rospy.wait_for_service('/link_attacher_node/detach')

        attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)


        message.joint_names = ["p0_joint","p1_joint","p2_joint","p3_joint","gripper_1","gripper_2"]

        if recvmsg == "a":
            points.positions = [0.0,2.3,-1.0,1.57,0.0,0.0]

        elif recvmsg == "b":
            points.positions = [0.0,2.3,-1.0,1.57,0.0627,-0.0627]

        elif recvmsg == "d":
            req.model_name_1 = "poke_arm"
            req.link_name_1 = "p3_link"
            req.model_name_2 = "my_box_1"
            req.link_name_2 = "my_box_link"

            attach_srv.call(req)

        elif recvmsg == "e":
            req.model_name_1 = "poke_arm"
            req.link_name_1 = "p3_link"
            req.model_name_2 = "my_box_1"
            req.link_name_2 = "my_box_link"

            detach_srv.call(req)

        elif recvmsg == "f":
            # Do nothing
            print("...")

        else:
            points.positions = [0.0,0.0,0.0,0.0,0.0,0.0]

        points.time_from_start = rospy.Duration(2.0)
        message.points = [points]
        recvmsg = "f"
        pub.publish(message)
        rate.sleep()

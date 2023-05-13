#!/usr/bin/env python
# KILL ME.
# This is one of the most complex and learning heavy scripts I'd have to do
# Armida
import rospy
import sys
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from gazebo_msgs.msg import ContactsState
from tf.transformations import quaternion_from_euler
from hand_tracking.msg import hand_cords

# rostopic pub 
# /arm_controller/command 
# trajectory_msgs/JointTrajectory 
# '{joint_names: ["p0_joint","p1_joint","p2_joint","p3_joint","gripper_1","gripper_2"], points: [{positions: [0.0,2.3,-1.0,1.57,0.0627,-0.0627],time_from_start:[1.0,0.0]}]}' -1

recvmsg = ""
message = JointTrajectory()
points = JointTrajectoryPoint()
req = AttachRequest()
gripper_1_coll_contact = ContactsState()
gripper_2_coll_contact = ContactsState()
holding = False
handCords = hand_cords()

# Grippers configs
grippers_open_config = [0.27,-0.27]
grippers_close_config = [0.084,-0.084]
grippers_closeMagnet_config = [0.094,-0.094]

# Current holding
current_model = ""
current_link = ""

# Get the names of the models we collided with
def get_contacts_names(contact):
    return [contact.collision1_name.split("::")[0], contact.collision2_name.split("::")[0]]

# Get the link of a contact
def get_contacts_links(contact):
    return contact.collision1_name.split("::")[1]

# Check for object collision (anything that touches the 2 gripper hands)
def check_object(gripper_1_coll_contact, gripper_2_coll_contact, attach_srv):
    global current_model
    global current_link
    global holding
    if not(holding):
        for contact1 in gripper_1_coll_contact.states:
            contacts1 = get_contacts_names(contact1)
            contact_1_1 = contacts1[0]
            contact_1_2 = contacts1[1]
            for contact2 in gripper_2_coll_contact.states:
                contacts2 = get_contacts_names(contact2)
                contact_2_1 = contacts2[0]
                contact_2_2 = contacts2[1]

                print("CHECKING")
                print(contact_1_1)
                print(contact_1_2)
                print(contact_2_1)
                print(contact_2_2)

                # Check if they are the same
                if contact_1_1 == contact_2_1:
                    req.model_name_1 = "robot"
                    req.link_name_1 = "p3_link"
                    req.model_name_2 = contact_1_1
                    req.link_name_2 = get_contacts_links(contact1)

                    current_model = req.model_name_2
                    current_link = req.link_name_2

                    attach_srv.call(req)
                    print("HOLDING A CUBE")
                    return True
    return False

def hand_callback(msg):
    global handCords
    handCords.x = msg.x*7.0/3200.0-0.7
    handCords.y = msg.y*(-7.0/4800.0)+0.7
    handCords.status = msg.status
    handCords.orientation = msg.orientation
    #print("X :",handCords.x, ", Y :",handCords.y) 

def activate_callback(msg):
    global recvmsg
    print(msg.data)
    recvmsg = msg.data

# Gripper 1 collided
def gripper_1_coll_callback(msg):
    global gripper_1_coll_contact
    gripper_1_coll_contact = msg
    #print("Gripper 1 Coll detected")

# Gripper 2 collided
def gripper_2_coll_callback(msg):
    global gripper_2_coll_contact
    gripper_2_coll_contact = msg
    #print("Gripper 2 Coll detected")


if __name__ == '__main__':

    # Pub to manage the gripper
    gripper = rospy.Publisher("/gripper/command", JointTrajectory, queue_size=10)

    # Pub to manage the path
    trajectory = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=20)
    
    gripper_1 = rospy.Subscriber('/gripper_1_vals', ContactsState, gripper_1_coll_callback)
    gripper_2 = rospy.Subscriber('/gripper_2_vals', ContactsState, gripper_2_coll_callback)
    
    # This will be input
    sub_wl = rospy.Subscriber("hand_coordinates", hand_cords, hand_callback)
    moveit_commander.roscpp_initialize(sys.argv)

    rospy.init_node("wobot")
    rate = rospy.Rate(100)

    # Moveit config
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "arm"
    group = moveit_commander.MoveGroupCommander(group_name)
    group.set_num_planning_attempts(10000000)
    group.set_planning_time(2)

    # Initial position
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 0.08726
    joint_goal[1] = 0.1745
    joint_goal[2] = 0.3490
    joint_goal[3] = 1.5708
    group.go(joint_goal, wait=True)
    group.stop()

    while not rospy.is_shutdown():
        # Service for magnet
        rospy.wait_for_service('/link_attacher_node/attach')
        rospy.wait_for_service('/link_attacher_node/detach')
        attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)

        # Message for closing and opening
        message.joint_names = ["gripper_1","gripper_2"]

        print(holding)

        # Manage a cube picking up
        if check_object(gripper_1_coll_contact, gripper_2_coll_contact, attach_srv):
            points.positions = grippers_closeMagnet_config
            gripper.publish(message)
            holding = True

        # Close
        if not(holding) and handCords.status == "close":
            points.positions = grippers_close_config
            gripper.publish(message)
            print("CLOSING")

        # Detach
        if handCords.status == "open":
            points.positions = grippers_open_config
            gripper.publish(message)

            if holding:
                req.model_name_1 = "robot"
                req.link_name_1 = "p3_link"
                req.model_name_2 = current_model
                req.link_name_2 = current_link

                detach_srv.call(req)
                holding = False
            print("OPENING")

        # Move to a location
        if handCords.x > 0:
            roll = 1.57
        else:
            roll = -1.57
        #roll = float(recvmsg.split(",")[3])
        pitch = 0.0
        yaw = 1.57
        quaternion = quaternion_from_euler(roll, pitch, yaw)
        
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = handCords.x
        pose_goal.position.y = 0.0
        if holding:
            pose_goal.position.z = handCords.y+0.05
        else:
            pose_goal.position.z = handCords.y
        pose_goal.orientation = geometry_msgs.msg.Quaternion(*quaternion)
        group.set_pose_target(pose_goal)
        plan = group.go(wait=True)
        group.stop()
        #group.clear_pose_targets()

        print("SUCCESS")

        points.time_from_start = rospy.Duration(2.0)
        message.points = [points]
        recvmsg = "f"
        rate.sleep()

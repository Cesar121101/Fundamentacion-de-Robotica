import rospy
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.srv import ApplyPlanningScene
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import roscpp_initialize, RobotCommander, PlanningSceneInterface, MoveGroupCommander

rospy.init_node("move_arm_node")

robot = RobotCommander()
group = MoveGroupCommander('arm')

target_pose = PoseStamped()
target_pose.header.frame_id = "baseMount_link"
target_pose.pose.position.x = 2.0
target_pose.pose.position.y = 0.0
target_pose.pose.position.z = 2.0
target_pose.pose.orientation.x = 0.0
target_pose.pose.orientation.y = 0.0
target_pose.pose.orientation.z = 0.0
target_pose.pose.orientation.w = 1.0

scene = PlanningSceneInterface()
rospy.wait_for_service('/apply_planning_scene')
apply_scene = rospy.ServiceProxy('/apply_planning_scene', ApplyPlanningScene)
apply_scene(PlanningScene(is_diff=True))

group.set_pose_target(target_pose)
plan = group.plan()
group.go(wait=True)

scene.remove_world_object()
rospy.signal_shutdown('Done!')
import rospy
from std_msgs.msg import Float64
import sys
from numpy import linspace
from math import pi
import math
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64
from std_msgs.msg import Header
from std_msgs.msg import Bool
from std_srvs.srv import Empty
import time

def perform_trajectory(pose):
	end_effector_pose_0 = [0.0,0.0,0.0,0.0,0.0,0.0]
	end_effector_pose_1 = [0,-0.6278 ,1.2265,0.9720 ,-1.5708 +math.pi, 0.3603]
	end_effector_pose_2 = [4.3521-(math.pi),-0.6278 ,1.2265,0.9720 ,-1.5708 +math.pi, 0.3603]
	end_effector_pose_3 =   [ 4.1351-math.pi,-1.7643, 2.2657, 1.0694,-1.5708+math.pi,0.5773]
	end_effector_pose_4 = [5.9108 -math.pi,-1.4173, 2.0805, 0.9077,-1.5708 +math.pi,-1.1984]
	vector = [end_effector_pose_0, end_effector_pose_1,end_effector_pose_2,end_effector_pose_3,end_effector_pose_4]

	rospy.init_node('publisher')
	contoller_name='/arrowbot_manipulator/manipulator_controller/command'
	trajectory_publihser = rospy.Publisher(contoller_name,JointTrajectory, queue_size=10)                        
	joint_names = ['ur5_shoulder_pan_joint','ur5_shoulder_lift_joint','ur5_elbow_joint','ur5_wrist_1_joint','ur5_wrist_2_joint','ur5_wrist_3_joint']
	#goal_positions = [joint_angles_final[0],joint_angles_final[1],joint_angles_final[2],joint_angles_final[3],joint_angles_final[4],joint_angles_final[5]]
	#goal_positions =[3.530803804574691, -0.45885124275501404, 3.141592653589793, -0.0010024297792483167, 0.001, 0.0]
	goal_positions = vector[pose]
	rospy.loginfo("The goal to be reached for given joint angles")
	rospy.loginfo(goal_positions)
	rospy.sleep(1)
	trajectory_msg = JointTrajectory()
	trajectory_msg.joint_names = joint_names
	trajectory_msg.points.append(JointTrajectoryPoint())
	trajectory_msg.points[0].positions = goal_positions
	trajectory_msg.points[0].velocities = [0.0 for i in joint_names]
	trajectory_msg.points[0].accelerations = [0.0 for i in joint_names]
	trajectory_msg.points[0].time_from_start = rospy.Duration(2)
	trajectory_publihser.publish(trajectory_msg)
	rospy.sleep(1)
def gripper_on():
	# Wait till the srv is available
    
    rospy.loginfo("gripperon")
    rospy.wait_for_service("/arrowbot_manipulator/on")
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/arrowbot_manipulator/on', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException as e:
        print ("Service call failed: %s" % e)
def gripper_off():
	# Wait till the srv is available
    rospy.loginfo("gripperoff")
    rospy.wait_for_service("/arrowbot_manipulator/off")
    try:
        # Create a handle for the calling the srv
        turn_on = rospy.ServiceProxy('/arrowbot_manipulator/off', Empty)
        # Use this handle just like a normal function and call it
        resp = turn_on()
        return resp
    except rospy.ServiceException as e:
        print ("Service call failed: %s" % e)
if __name__ == '__main__':
    perform_trajectory(0)
    time.sleep(1)
    perform_trajectory(1)
    time.sleep(1)
    perform_trajectory(2)
    time.sleep(1)
    gripper_on()
    time.sleep(1)
    perform_trajectory(3)
    time.sleep(1)
    perform_trajectory(4)
    time.sleep(5)
    gripper_off()

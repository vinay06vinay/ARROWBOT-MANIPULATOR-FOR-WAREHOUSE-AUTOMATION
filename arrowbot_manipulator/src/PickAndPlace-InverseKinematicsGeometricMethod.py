import rospy
from std_msgs.msg import Float64
import sys
from functools import partial
from sympy import symbols,cos,sin,simplify,Matrix,pprint,evalf,diff
import math
import numpy as np
from numpy import linspace
from math import pi
from std_msgs.msg import Float64
from std_msgs.msg import Header
from std_msgs.msg import Bool
from std_srvs.srv import Empty
import actionlib
#Messages on which Trajectory goals are published
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def forward_inverse_kinematics(final_pose):
	theta1,theta2,theta3,theta4,theta5,theta6,d1,a2,a3,d4,d5,d6 = symbols('theta1 theta2 theta3 theta4 theta5 theta6 d1 a2 a3 d4 d5 d6', real=True)
	round3 = partial(round, ndigits=3)
	round1 = partial(round, ndigits=0)
	#Transformation for A1
	R_z_1 = Matrix(([cos(theta1),-sin(theta1),0,0],[sin(theta1),cos(theta1),0,0],[0,0,1,0],[0,0,0,1]))
	T_z_1 = Matrix(([1,0,0,0],[0,1,0,0],[0,0,1,d1],[0,0,0,1]))
	R_x_1 = Matrix(([1,0,0,0],[0,round(math.cos(math.radians(90))),-round(math.sin(math.radians(90))),0],[0,round(math.sin(math.radians(90))),round(math.cos(math.radians(90))),0],[0,0,0,1]))
	A1 = R_z_1*T_z_1*R_x_1

	#Transformation for A2
	R_z_2 = Matrix(([cos(theta2),-sin(theta2),0,0],[sin(theta2),cos(theta2),0,0],[0,0,1,0],[0,0,0,1]))
	T_x_2 = Matrix(([1,0,0,a2],[0,1,0,0],[0,0,1,0],[0,0,0,1]))
	A2 = R_z_2*T_x_2

	# Transformation for A3
	theta3,a3 = symbols('theta3 a3', real=True)
	R_z_3 = Matrix(([cos(theta3),-sin(theta3),0,0],[sin(theta3),cos(theta3),0,0],[0,0,1,0],[0,0,0,1]))
	T_x_3 = Matrix(([1,0,0,a3],[0,1,0,0],[0,0,1,0],[0,0,0,1]))
	A3 = R_z_3*T_x_3

	#Transformation for A4
	theta4,d4 = symbols('theta4 d4', real=True)
	R_z_4 = Matrix(([cos(theta4),-sin(theta4),0,0],[sin(theta4),cos(theta4),0,0],[0,0,1,0],[0,0,0,1]))
	T_z_4 = Matrix(([1,0,0,0],[0,1,0,0],[0,0,1,d4],[0,0,0,1]))
	R_x_4 = Matrix(([1,0,0,0],[0,round(math.cos(math.radians(90))),-round(math.sin(math.radians(90))),0],[0,round(math.sin(math.radians(90))),round(math.cos(math.radians(90))),0],[0,0,0,1]))
	A4 = R_z_4*T_z_4*R_x_4

	#Transformation for A5
	theta5,d5 = symbols('theta5 d5', real=True)
	R_z_5 = Matrix(([cos(theta5),-sin(theta5),0,0],[sin(theta5),cos(theta5),0,0],[0,0,1,0],[0,0,0,1]))
	T_z_5 = Matrix(([1,0,0,0],[0,1,0,0],[0,0,1,d5],[0,0,0,1]))
	R_x_5 = Matrix(([1,0,0,0],[0,round(math.cos(math.radians(-90))),-round(math.sin(math.radians(-90))),0],[0,round(math.sin(math.radians(-90))),round(math.cos(math.radians(-90))),0],[0,0,0,1]))
	A5 = R_z_5*T_z_5*R_x_5

	#Transformation for A6
	theta6,d6 = symbols('theta6 d6', real=True)
	R_z_6 = Matrix(([cos(theta6),-sin(theta6),0,0],[sin(theta6),cos(theta6),0,0],[0,0,1,0],[0,0,0,1]))
	T_z_6 = Matrix(([1,0,0,0],[0,1,0,0],[0,0,1,d6],[0,0,0,1]))
	A6 = R_z_6*T_z_6

	#Final Transformation matrix 
	x1,x2,x3,y1,y2,y3,z1,z2,z3,p1,p2,p3 = symbols('x1 x2 x3 y1 y2 y3 z1 z2 z3 p1 p2 p3 ', real=True)
	A= A1*A2*A3*A4*A5*A6
	A_Final = A.subs([(a3,-0.39225),(a2,-0.42500),(d1,0.089159),(d4,0.10915),(d5,0.09465),(d6,0.0823)])
	A_Final
	#Ab = A_Final.subs([(theta1,math.radians(0)),(theta2,math.radians(-90)),(theta4,math.radians(0)),(theta5,math.radians(0)),(theta6,math.radians(0)),(theta3,math.radians(0))])
	#Ab.applyfunc(round3)
	
	'''
	Inverse Kinematics calculations
	Calculations of joint angles using geometric approach
	'''
	final_thetas=[]
	#Finding position of point 5 and calculation of theta1
	d4_1 = 0.10915
	d6_1  = 0.0823
	p5 = final_pose * Matrix([0,0,-d6_1,1])
	#phi1 = symbols('phi1', real=True)

	#Calculation of theta1 in reference to frame 5
	p5_x = p5[0]
	p5_y = p5[1]
	p5_z = p5[2]
	p5_xy = math.sqrt(((p5_x)**2)+((p5_y)**2))
	phi1 = math.atan2(p5_y,p5_x)
	phi2 = math.acos(d4_1/p5_xy) 
	#Shoulder left so positive phi2
	theta1_final = phi1 + phi2 + (math.pi/2)
	
	#Calculation of theta5 in reference to theta1 calculated above
	p6_x = final_pose[3]
	p6_y = final_pose[7]
	p6_z = final_pose[11]
	a1= p6_x*(math.sin(theta1_final))
	a2= -p6_y*(math.cos(theta1_final))
	if(abs((a1-a2-d4_1))<=d6_1):
	    theta5_final = -math.acos(((a1-a2-d4_1)/d6_1))
	else:
	    theta5_final=0.05

	#Calculation of theta6 using theta1, theta5 and inverse of final pose
	final_pose_inverse = final_pose.inv()
	x6_0y = final_pose_inverse[4]
	y6_0y = final_pose_inverse[1]
	x6_0x = final_pose_inverse[0]
	y6_0x = final_pose_inverse[5]
	if(math.sin(theta5_final) == 0):
	    theta6_final = 0.2
	else:
	    theta6_final_1 = ((-x6_0y*math.sin(theta1_final))+(y6_0y*math.cos(theta1_final))) / math.sin(theta5_final)
	    theta6_final_2 = ((x6_0x*math.sin(theta1_final))-(y6_0x*math.cos(theta1_final))) / math.sin(theta5_final)
	    theta6_final = math.atan2(theta6_final_1,theta6_final_2)
	
	#Calculation of Theta3 by equating transformation matrix from Link 2 to Link4 on right hand side. And All other transformation 
	#brought on left hand side of final transformation and performed inverse.
	a2_1 = -0.42500
	a3_1 = -0.39225 

	theta3_1 = (A6.inv()*A5.inv()*A1.inv()*final_pose).subs([(theta1,theta1_final),(theta5,theta5_final),(theta6,theta6_final),(d5,0.09465),(d6,0.0823),(d1,0.089159)])
	p4_x = theta3_1[3]
	p4_z = theta3_1[11]
	p4_xz = math.sqrt((theta3_1[3]**2)+(theta3_1[11]**2))
	abs_a23_1  = abs(a2_1 - a3_1)
	abs_a23_2 = abs(a2_1 + a3_1)
	if(p4_xz > abs_a23_1 and p4_xz<abs_a23_2 ):
	    theta3_final = math.acos(round1(((p4_xz**2) - (a2_1**2) - (a3_1**2))/(2*a2_1*a3_1)))
	else:
	    theta3_final=0.01
	
	#theta2
	theta2_1 = math.atan2(-p4_z,-p4_x)
	theta2_2 = math.asin((-a3_1*math.sin(theta3_final))/abs(p4_xz))
	theta2_final = theta2_1-theta2_2
	
	#Calculation of theta4 from all the above thetas calculated and equating all individual inverse transform matrices with
	#transformation matrix A4
	theta4_1= (A1.inv()*A2.inv()*A3.inv()*A5.inv()*A6.inv()*final_pose).subs([(theta1,theta1_final),(theta5,theta5_final),(theta3,theta3_final),(theta2,theta2_final),(theta6,theta6_final),(d5,0.09465),(d6,0.0823),(d1,0.089159)])
	theta4_final = math.atan2(theta4_1[4],theta4_1[0])
	final_thetas=[theta1_final,theta2_final,theta3_final,theta4_final,theta5_final,theta6_final]
	return final_thetas

def move_straight():
	turn_right = rospy.Publisher('/arrowbot_manipulator/front_right_steering/command', Float64, queue_size=15)
	turn_left = rospy.Publisher('/arrowbot_manipulator/front_left_steering/command', Float64, queue_size=15)
	move_rear_left_motor = rospy.Publisher('/arrowbot_manipulator/rear_left_motor/command', Float64, queue_size=15) 
	move_rear_right_motor = rospy.Publisher('/arrowbot_manipulator/rear_right_motor/command', Float64, queue_size=15)
	rate = rospy.Rate(100)
	control_left_speed = 10
	control_right_speed =10
	control_turn = 0
	time1 = rospy.Time.now().to_sec()
	distance_moved = 0
	i = 0
	while not rospy.is_shutdown():
		turn_right.publish(control_turn) # publish the turn command.
		turn_left.publish(control_turn) # publish the turn command.
		move_rear_left_motor.publish(control_left_speed) # publish he control speed. 
		move_rear_right_motor.publish(control_right_speed)
		print ("Robot moving straight for a distance of 10 and stopping")
		time2 = rospy.Time.now().to_sec()       
		#Calculation of distance the robot moved
		distance_moved = (time2 - time1)*10.0
		print(distance_moved)
		#If robot reaches the goal point it will be stopped by exiting the loop and making velocity of robot zero.
		if(distance_moved > 10.0):
			break
	turn_right.publish(0) # publish the turn command.
	turn_left.publish(0) # publish the turn command.
	move_rear_left_motor.publish(0) # publish he control speed. 
	move_rear_right_motor.publish(0)

def perform_trajectory(pose):
	end_effector_pose_1 = Matrix(([1,0,0,-0.22],[0,1,0,-0.57],[0,0,1,0.15],[0,0,0,1]))
	end_effector_pose_2 = Matrix(([1,0,0,0.6],[0,1,0,0.2],[0,0,1,0.2],[0,0,0,1]))
	vector = [end_effector_pose_1,end_effector_pose_2]
	joint_angles_final = forward_inverse_kinematics(vector[pose])
	#Initialising a Publisher node and creating an a Trajectory Message
	rospy.init_node('publisher')
	contoller_name='/arrowbot_manipulator/manipulator_controller/command'
	trajectory_publihser = rospy.Publisher(contoller_name,JointTrajectory, queue_size=10)      
               
	joint_names = ['ur5_shoulder_pan_joint','ur5_shoulder_lift_joint','ur5_elbow_joint','ur5_wrist_1_joint','ur5_wrist_2_joint','ur5_wrist_3_joint']
	goal_positions = joint_angles_final
	rospy.loginfo("Goal to be recached in terms of Joint Angles(rads) for a given position and orientation.")
	rospy.loginfo(joint_angles_final)
	rospy.sleep(1)

	#Creating a Joint Tracjectory Message to which joint names, Trajectory point which contains position velocity and acceleration 
	#are given. Even though we are dealing with joint positions are zero we need to intialise velocity and acceleration as per the
	#Joint Trajectory Point Structure.
	trajectory_msg = JointTrajectory()
	trajectory_msg.joint_names = joint_names
	trajectory_msg.points.append(JointTrajectoryPoint())
	trajectory_msg.points[0].positions =  joint_angles_final
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
    rospy.loginfo("gripperon")
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
    gripper_on()
    perform_trajectory(1)
    gripper_off()
    move_straight()	

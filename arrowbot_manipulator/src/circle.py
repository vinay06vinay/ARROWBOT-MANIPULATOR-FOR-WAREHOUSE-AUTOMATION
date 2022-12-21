# %%
"""
Problem 1
1)Forware kinematics of Frank Emika Robot is found out use DH parameter table.
The Indiviudal transformation matrix is calculated from DH table presented in the report
For 5 different geometrical configurations , final transformation matrix is calculated for geometric validation
2)With the updated DH table, theta3 is made zero as it is assumed to be fixed.
"""
#! /usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import actionlib
#These are required to Create Action Client Interface and send trajectories to Goal
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sympy import symbols,cos,sin,simplify,Matrix,pprint,evalf,diff,sqrt
import math
from functools import partial
round3 = partial(round, ndigits=3)
#Transformation for A1
theta1,d1 = symbols('theta1 d1', real=True)
R_z_1 = Matrix(([cos(theta1),-sin(theta1),0,0],[sin(theta1),cos(theta1),0,0],[0,0,1,0],[0,0,0,1]))
T_z_1 = Matrix(([1,0,0,0],[0,1,0,0],[0,0,1,d1],[0,0,0,1]))
R_x_1 = Matrix(([1,0,0,0],[0,round(math.cos(math.radians(90))),-round(math.sin(math.radians(90))),0],[0,round(math.sin(math.radians(90))),round(math.cos(math.radians(90))),0],[0,0,0,1]))
print("Transformation matrix for A1:")
A1 = R_z_1*T_z_1*R_x_1
pprint(A1)
print("\n")

#Transformation for A2
theta2,a2 = symbols('theta2 a2', real=True)
R_z_2 = Matrix(([cos(theta2),-sin(theta2),0,0],[sin(theta2),cos(theta2),0,0],[0,0,1,0],[0,0,0,1]))
T_x_2 = Matrix(([1,0,0,a2],[0,1,0,0],[0,0,1,0],[0,0,0,1]))
print("Transformation matrix for A2:")
A2 = R_z_2*T_x_2
pprint(A2)
print("\n")

# Transformation for A3
theta3,a3 = symbols('theta3 a3', real=True)
R_z_3 = Matrix(([cos(theta3),-sin(theta3),0,0],[sin(theta3),cos(theta3),0,0],[0,0,1,0],[0,0,0,1]))
T_x_3 = Matrix(([1,0,0,a3],[0,1,0,0],[0,0,1,0],[0,0,0,1]))
print("Transformation matrix for A3:")
A3 = R_z_3*T_x_3
pprint(A3)
print("\n")

#Transformation for A4
theta4,d4 = symbols('theta4 d4', real=True)
R_z_4 = Matrix(([cos(theta4),-sin(theta4),0,0],[sin(theta4),cos(theta4),0,0],[0,0,1,0],[0,0,0,1]))
T_z_4 = Matrix(([1,0,0,0],[0,1,0,0],[0,0,1,d4],[0,0,0,1]))
R_x_4 = Matrix(([1,0,0,0],[0,round(math.cos(math.radians(90))),-round(math.sin(math.radians(90))),0],[0,round(math.sin(math.radians(90))),round(math.cos(math.radians(90))),0],[0,0,0,1]))
print("Transformation matrix for A4:")
A4 = R_z_4*T_z_4*R_x_4
pprint(A4)
print("\n")

#Transformation for A5
theta5,d5 = symbols('theta5 d5', real=True)
R_z_5 = Matrix(([cos(theta5),-sin(theta5),0,0],[sin(theta5),cos(theta5),0,0],[0,0,1,0],[0,0,0,1]))
T_z_5 = Matrix(([1,0,0,0],[0,1,0,0],[0,0,1,d5],[0,0,0,1]))
R_x_5 = Matrix(([1,0,0,0],[0,round(math.cos(math.radians(-90))),-round(math.sin(math.radians(-90))),0],[0,round(math.sin(math.radians(-90))),round(math.cos(math.radians(-90))),0],[0,0,0,1]))
print("Transformation matrix for A5:")
A5 = R_z_5*T_z_5*R_x_5
pprint(A5)
print("\n")


#Transformation for A6
theta6,d6 = symbols('theta6 d6', real=True)
R_z_6 = Matrix(([cos(theta6),-sin(theta6),0,0],[sin(theta6),cos(theta6),0,0],[0,0,1,0],[0,0,0,1]))
T_z_6 = Matrix(([1,0,0,0],[0,1,0,0],[0,0,1,d6],[0,0,0,1]))

print("Transformation matrix for A6:")
A6 = R_z_6*T_z_6
pprint(A6)
print("\n")



#Final Transformation matrix 
x1,x2,x3,y1,y2,y3,z1,z2,z3,p1,p2,p3 = symbols('x1 x2 x3 y1 y2 y3 z1 z2 z3 p1 p2 p3 ', real=True)
A= A1*A2*A3*A4*A5*A6
A_Final = A.subs([(a3,-0.39225),(a2,-0.42500),(d1,0.089159),(d4,0.10915),(d5,0.09465),(d6,0.0823)])
print("The Final Transformation matrix in terms of joint angles as variables:")
# A_Final
               
Ab = A_Final.subs([(theta1, 4.3521),(theta2,-0.6278),(theta4,0.9720),(theta5,-1.5708),(theta6,0.3603),(theta3, 1.2265)])
Ab.applyfunc(round3)
A_1=simplify(A)
A_1
# Qans = (A2*A3*A4*A5*A6).subs([(theta1,math.radians(0)),(theta2,math.radians(0)),(theta4,math.radians(0)),(theta3,math.radians(0))])
# simplify(Qans.inv())
#pose = Matrix(([1,0,0,p1],[0,1,0,p2],[0,0,1,p3],[0,0,0,1]))
#pose
#((A2*A3*A4*A5*A6).T).subs([(theta1,math.radians(0)),(theta2,math.radians(0)),(theta4,math.radians(0)),(theta3,math.radians(0))])

# %%
A10 = A1
A20 = A1*A2
A30 = A1*A2*A3
A40 = A1*A2*A3*A4
A50 = A1*A2*A3*A4*A5
A60 = A1*A2*A2*A3*A4*A5

# %%
import time
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D
start = time.time() #start time


# %%
Xp0 = A60.col(3)
Xp0.row_del(3)
pprint(Xp0)

# %%
#Calculating Partial Differentials for the jacobian
dele1 = diff(Xp0, theta1)
dele2 = diff(Xp0, theta2)
dele3 = diff(Xp0, theta3)
dele4 = diff(Xp0, theta4)
dele5 = diff(Xp0, theta5)
dele6 = diff(Xp0, theta6)

# %%
#Calculating Z column vectors for Jacobian
Z1 = A10.col(2)
Z1.row_del(3)
Z2 = A20.col(2)
Z2.row_del(3)
Z3 = A30.col(2)
Z3.row_del(3)
Z4 = A40.col(2)
Z4.row_del(3)
Z5 = A50.col(2)
Z5.row_del(3)
Z6 = A60.col(2)
Z6.row_del(3)

# %%
#Defining the Jacobian Matrix
Jacob = Matrix([[dele1,dele2,dele3,dele4,dele5,dele6],[Z1,Z2,Z3,Z4,Z5,Z6]])
J=Jacob.evalf()
pprint(J)


# %%
#Writing Geometric dimensions of robot in Jacobian and End effector transformation wrt Base
J =J.subs([(a3,-0.39225),(a2,-0.42500),(d1,0.089159),(d4,0.10915),(d5,0.09465),(d6,0.0823)])
A07 =A60.subs([(a3,-0.39225),(a2,-0.42500),(d1,0.089159),(d4,0.10915),(d5,0.09465),(d6,0.0823)])

#Defining the initial value of joint angles given in question
q =  Matrix([[0.0], [-math.pi/2], [-math.pi/2], [0.0], [0.0000000000000001], [0.001]])

#Defining the list for storing the values of x, y, and z of end effectors
y = []
z = []
x = []

i=0 #Defining the iterators

# %%
#Printing the initial Jacobian Matrix
J2 = J
J2 = J2.subs({theta1:q[0,0],theta2:q[1,0],theta3:q[2,0],theta4:q[3,0],theta5:q[4,0],theta6:q[5,0]}).evalf()
pprint(J2)

# %%
#Define time step for loop
T = 5 #Time required to complete one revolution
N = 100 #No. of Iterations
dt = T / N #Time Step
i = 0
q_pub1 = []
q_pub2 = []
q_pub3 = []
q_pub4 = []
q_pub5 = []
q_pub6 = []

# %%
while (i<=100):  ## run this for loop for more than 1000 time to make sure that circle is being perfactly drawn
    ## divide the entire circle to be drawing in 1000 points
    print(i)
    x_dot = -0.435987207*0.4*math.pi*sin((2*math.pi/100)*i)
    y_dot = 0.435987207*0.4*math.pi*cos((2*math.pi/100)*i)

    V = Matrix([x_dot,y_dot,0.0, 0.0, 0.0, 0.0]).evalf()
    
    ## find the transformation matrix from base to end effector in each iteration
    ## take theta3 = 0
    A = A07.subs({theta1:q[0,0],theta2:q[1,0],theta3:q[2,0],theta4:q[3,0],theta5:q[4,0],theta6:q[5,0]}).evalf()

    # position of ball point in 7th frame
    P7 = Matrix([0,0,0,1])

    ##Storing q values
    q_pub1.append(q[0,0])
    q_pub2.append(q[1,0])
    q_pub3.append(q[2,0])
    q_pub4.append(q[3,0])
    q_pub5.append(q[4,0])
    q_pub6.append(q[5,0])
    
    ## position of ball point in the origin frame
    P07 = A*P7
    ## add the value of x,y and z in the list at each iteration
    ## This will be used for plotting the circle later
    y.append(P07[1,0])
    z.append(P07[2,0])
    x.append(P07[0,0])
    ## jacobian to find the jacobian matrix for each iteration
    ## take theta3 = 0
    J1 = J.subs({theta1:q[0,0],theta2:q[1,0],theta3:q[2,0],theta4:q[3,0],theta5:q[4,0],theta6:q[5,0]}).evalf()
    J_inv = J1.inv('LU')
    # # find the difference in the position of end effector in each iteration
    # ## update the value of q 
    q = q + (J_inv*V*dt)
    i+=1



# %%
# plot the circle after finding x and z value at each iteration
plt.plot(x,y)
plt.xlabel("X coordinate")
plt.ylabel("Y coordinate")
plt.axis("equal")
plt.pause(0.05)
plt.show()
end = time.time()
print(end-start) # print time taken to run the code


# %%
from mpl_toolkits.mplot3d import axes3d, Axes3D
ax = plt.axes(projection='3d')

# Data for a three-dimensional line
# ax.axes.set_xlim3d(left=500, right=700) 
# ax.axes.set_ylim3d(bottom=-200, top=200) 
# ax.axes.set_zlim3d(bottom=0, top=1000) 
ax.set_xlabel("X coordinate")
ax.set_ylabel("Y coordinate")
ax.set_zlabel("Z coordinate")


ax.plot3D(x, y, z, 'red' ,linewidth=2.5)


# %%
max(x)

# %%
min(x)

# %%
min(y)

# %%
max(y)

# %%
max(z)

# %%
min(z)

# %%

rospy.init_node('node_circle')
#A node called publisher is initalised
#rospy.init_node('publisher')
#A simple Action client object is started to follow a trajectory and is waiting for the server.
client = actionlib.SimpleActionClient('/arrowbot_manipulator/manipulator_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
client.wait_for_server()
#Joint names same as in urdf           
joint_names = ['ur5_shoulder_pan_joint','ur5_shoulder_lift_joint','ur5_elbow_joint','ur5_wrist_1_joint','ur5_wrist_2_joint','ur5_wrist_3_joint']
r = rospy.Rate(10) # 10hz
#pub1.publish(0.0)
#rospy.sleep(1.0)
#pub2.publish(-1.57)
#rospy.sleep(1.0)
#pub3.publish(-1.57)
#rospy.sleep(1.0)
#pub4.publish(0.0)
#rospy.sleep(1.0)
#pub5.publish(0.0)
#rospy.sleep(1.0)
#pub6.publish(0.0)
#rospy.sleep(1.0)
pprint("here")
j=0

while (j<=100):
    qpub = [q_pub1[j],q_pub2[j],q_pub3[j],q_pub4[j],q_pub5[j],q_pub6[j]]
    rospy.loginfo("The joint angles in this run are :")
    rospy.loginfo(i)
    trajectory_msg = JointTrajectory()
    trajectory_msg.joint_names = joint_names
    trajectory_msg.points.append(JointTrajectoryPoint())
    trajectory_msg.points[0].positions = qpub
    trajectory_msg.points[0].velocities =  [0.0 for i in joint_names]
    trajectory_msg.points[0].accelerations = [0.0 for i in joint_names]
    trajectory_msg.points[0].time_from_start = rospy.Duration(2)
    goal_positions = FollowJointTrajectoryGoal()
    goal_positions.trajectory = trajectory_msg
    goal_positions.goal_time_tolerance = rospy.Duration(0)
    client.send_goal(goal_positions)
    rospy.sleep(0.05)
    j+=1

while not rospy.is_shutdown():

        r.sleep()

    


# %%




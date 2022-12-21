# ARROWBOT-MANIPULATOR-FOR-WAREHOUSE-AUTOMATION
Arrowbot is the name of the wheeled manipulator designed and simulated to perform autonomous tasks of pick and place in warehouse. 

The project has been designed as part of the coursework. Refer to the report for details on implementation and detailed description of the manipulator applications.  

How to Run?
To run the package:
--> Create a catkin_ws and build it, then source it
--> Download the ros package named arrowbot_manipulator
--> Paste the folder inside the source directory of catkin_ws (i.e., ~/catkin_ws/src)
--> Build and source the arrowbot_manipulator package
--> Commands to run the arrowbot_manipulator for different tasks (Note: Multiple terminal instances are required and run in the same order)
     Instructions to perform circle trajectory:
	-> Open terminal and type in 'roslaunch arrowbot_manipulator manipulator.launch'
	-> Open another terminal and input command 'rosrun arrowbot_manipulator circle.py'
       -> The robot will make a circle trajectory and transformation matrices , jacobian, joint angles are printed on terminal.
	-> Ctrl + C to kill the windows and stop simulation
     Instructions to perform pick and place using Inverse Kinematics solved by geometric approach:
	-> Open terminal and type in 'roslaunch arrowbot_manipulator manipulator.launch'
	-> Open another terminal and input command 'rosrun arrowbot_manipulator PickAndPlace-InverseKinematicsGeometricMethod.py '
       -> The code will start and the robot moves to two different predefined points given for which inverse is calcualated in the code.
       -> After the first task is performed, the robot will move forward to next goal.
	-> Ctrl + C to kill the windows and stop simulation
     Instructions to perform pick and place using Inverse Kinematics solved by IK Solver integrated with camera:
	-> Open terminal and type in 'roslaunch arrowbot_manipulator manipulator.launch'
       -> Open another terminal and input command 'roslaunch arrowbot_manipulator display.launch'.
	-> Select base_link from the fixed frame field.
	-> Select Add command to and select Robot Model command to see the model
	-> Select Add command again and select Laser Scan to see the Lidar
		-> Select topic /arrowbot_manipulator/scan from the topic list and change Size of the beam to 0.1 mtrs
	-> Select Add command again and select Image to see the The live feed of the image.
		-> Select topic /mybot/camera/image_raw from the topic list 
	-> Resize the window so that it can be viewed with gazebo and rviz windows
	-> Open another terminal and input command 'rosrun arrowbot_manipulator PickAndPlaceUsingKnownJointAngles.py'
       -> The code will start and the robot moves along different positions to pick and place the block on the cart. The live image feed   		is visible on rviz window.
	-> Ctrl + C to kill the windows and stop simulation


To open the CAD files:
	->Open the folder Cad Models
	->It should load all the parts required for completing the assembly

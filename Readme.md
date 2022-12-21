To run the package:
1) Create a catkin_ws and build it, then source it
2) Download the ros package named arrowbot_manipulator
3)Paste the folder inside the source directory of catkin_ws (i.e., ~/catkin_ws/src)
4) Build and source the arrowbot_manipulator package
5) Commands to run the arrowbot_manipulator for different tasks (Note: Multiple terminal instances are required and run in the same order)
     a) Instructions to perform circle trajectory:
	 1) Open terminal and type in 'roslaunch arrowbot_manipulator manipulator.launch'
	 2) Open another terminal and input command 'rosrun arrowbot_manipulator circle.py'
         3) The robot will make a circle trajectory and transformation matrices , jacobian, joint angles are printed on terminal.
	 4) Ctrl + C to kill the windows and stop simulation
     b) Instructions to perform pick and place using Inverse Kinematics solved by geometric approach:
	1) Open terminal and type in 'roslaunch arrowbot_manipulator manipulator.launch'
	2) Open another terminal and input command 'rosrun arrowbot_manipulator PickAndPlace-InverseKinematicsGeometricMethod.py '
        3) The code will start and the robot moves to two different predefined points given for which inverse is calcualated in the code.
        4) After the first task is performed, the robot will move forward to next goal.
	5) Ctrl + C to kill the windows and stop simulation
     c) Instructions to perform pick and place using Inverse Kinematics solved by IK Solver integrated with camera:
	1) Open terminal and type in 'roslaunch arrowbot_manipulator manipulator.launch'
        2) Open another terminal and input command 'roslaunch arrowbot_manipulator display.launch'.
	3) Select base_link from the fixed frame field.
	4) Select Add command to and select Robot Model command to see the model
	5) Select Add command again and select Laser Scan to see the Lidar
		-> Select topic /arrowbot_manipulator/scan from the topic list and change Size of the beam to 0.1 mtrs
	6) Select Add command again and select Image to see the The live feed of the image.
		-> Select topic /mybot/camera/image_raw from the topic list 
	7) Resize the window so that it can be viewed with gazebo and rviz windows
	8) Open another terminal and input command 'rosrun arrowbot_manipulator PickAndPlaceUsingKnownJointAngles.py'
        9) The code will start and the robot moves along different positions to pick and place the block on the cart. The live image feed   		is visible on rviz window.
	10) Ctrl + C to kill the windows and stop simulation


6) To open the CAD files:
	->Open the folder Cad Models
	->It should load all the parts required for completing the assembly


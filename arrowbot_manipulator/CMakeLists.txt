cmake_minimum_required(VERSION 2.8.3)

project(arrowbot_manipulator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
 
)

catkin_package()

find_package(roslaunch)
catkin_install_python(PROGRAMS src/PickAndPlaceUsingKnownJointAngles.py 
 DESTINATION 
${CATKIN_PACKAGE_BIN_DESTINATION}
)

find_package(roslaunch)
catkin_install_python(PROGRAMS src/PickAndPlace-InverseKinematicsGeometricMethod.py 
 DESTINATION 
${CATKIN_PACKAGE_BIN_DESTINATION}
)
find_package(roslaunch)
catkin_install_python(PROGRAMS src/InverseKinematicsCircleTrajectory.py 
 DESTINATION 
${CATKIN_PACKAGE_BIN_DESTINATION}
)



foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

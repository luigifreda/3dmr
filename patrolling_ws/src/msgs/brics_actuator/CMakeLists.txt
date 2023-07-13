cmake_minimum_required(VERSION 2.8.3)
project(brics_actuator)

find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs)

add_message_files(FILES 
	CartesianPose.msg
	CartesianTwist.msg
	CartesianVector.msg
	CartesianWrench.msg
	JointAccelerations.msg
	JointConstraint.msg
	JointImpedances.msg
	JointPositions.msg
	JointTorques.msg
	JointValue.msg
	JointVelocities.msg
	Poison.msg
)

generate_messages(DEPENDENCIES geometry_msgs)

find_package(catkin REQUIRED)

catkin_package()


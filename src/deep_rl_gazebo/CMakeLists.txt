cmake_minimum_required(VERSION 3.0.2)
project(deep_rl_gazebo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

################################################################################
# Install
################################################################################
catkin_install_python(PROGRAMS
  nodes/train_model.py
  nodes/test/test_model_sim.py
  nodes/test/test_model_irl.py
  nodes/models/drl_agent.py
  nodes/models/q_network.py
  nodes/common/result_graph
  nodes/common/plotting.py
  nodes/common/action_graph
  src/environment.py
  src/respawnGoal.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

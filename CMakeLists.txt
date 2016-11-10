cmake_minimum_required(VERSION 2.8.3)
project(unified_joint_trajectory_controller)

find_package(catkin REQUIRED COMPONENTS
  rospy
  actionlib_msgs
)

catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  JointsToPosition.msg
)


## Generate actions in the 'action' folder
add_action_files(
  FILES
  GoJointsToPosition.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
)

# Maybe use ddynamic_reconfigure_python if there
# appear parameters to be tuned dynamically


catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)


#############
## Install ##
#############

# Mark executable scripts (Python etc.) for installation
# in contrast to setup.py, you can choose the destination
install(PROGRAMS
  scripts/command_joints.py
  scripts/fake_joint_traj_cnt.py
  scripts/get_current_joint_pos.py
  scripts/get_joint_limits.py
  scripts/unified_interface.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation
foreach (dir launch config)
    install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "franka_interface_msgs: 13 messages, 2 services")

set(MSG_I_FLAGS "-Ifranka_interface_msgs:/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg;-Ifranka_interface_msgs:/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg;-Iactionlib_msgs:/opt/ros/noetic/share/actionlib_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(franka_interface_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/Errors.msg" NAME_WE)
add_custom_target(_franka_interface_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "franka_interface_msgs" "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/Errors.msg" ""
)

get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/FrankaInterfaceStatus.msg" NAME_WE)
add_custom_target(_franka_interface_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "franka_interface_msgs" "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/FrankaInterfaceStatus.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/RobotState.msg" NAME_WE)
add_custom_target(_franka_interface_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "franka_interface_msgs" "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/RobotState.msg" "franka_interface_msgs/Errors:std_msgs/Header"
)

get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/RunLoopProcessInfoState.msg" NAME_WE)
add_custom_target(_franka_interface_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "franka_interface_msgs" "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/RunLoopProcessInfoState.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/SensorData.msg" NAME_WE)
add_custom_target(_franka_interface_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "franka_interface_msgs" "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/SensorData.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/SensorDataGroup.msg" NAME_WE)
add_custom_target(_franka_interface_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "franka_interface_msgs" "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/SensorDataGroup.msg" "std_msgs/Header:franka_interface_msgs/SensorData"
)

get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillAction.msg" NAME_WE)
add_custom_target(_franka_interface_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "franka_interface_msgs" "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillAction.msg" "franka_interface_msgs/ExecuteSkillActionFeedback:franka_interface_msgs/ExecuteSkillResult:franka_interface_msgs/ExecuteSkillGoal:franka_interface_msgs/ExecuteSkillActionResult:franka_interface_msgs/ExecuteSkillFeedback:std_msgs/Header:franka_interface_msgs/ExecuteSkillActionGoal:actionlib_msgs/GoalID:actionlib_msgs/GoalStatus"
)

get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionGoal.msg" NAME_WE)
add_custom_target(_franka_interface_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "franka_interface_msgs" "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionGoal.msg" "franka_interface_msgs/ExecuteSkillGoal:std_msgs/Header:actionlib_msgs/GoalID"
)

get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionResult.msg" NAME_WE)
add_custom_target(_franka_interface_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "franka_interface_msgs" "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionResult.msg" "std_msgs/Header:actionlib_msgs/GoalID:actionlib_msgs/GoalStatus:franka_interface_msgs/ExecuteSkillResult"
)

get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionFeedback.msg" NAME_WE)
add_custom_target(_franka_interface_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "franka_interface_msgs" "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionFeedback.msg" "franka_interface_msgs/ExecuteSkillFeedback:std_msgs/Header:actionlib_msgs/GoalID:actionlib_msgs/GoalStatus"
)

get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillGoal.msg" NAME_WE)
add_custom_target(_franka_interface_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "franka_interface_msgs" "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillGoal.msg" ""
)

get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillResult.msg" NAME_WE)
add_custom_target(_franka_interface_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "franka_interface_msgs" "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillResult.msg" ""
)

get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillFeedback.msg" NAME_WE)
add_custom_target(_franka_interface_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "franka_interface_msgs" "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillFeedback.msg" ""
)

get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/srv/GetCurrentRobotStateCmd.srv" NAME_WE)
add_custom_target(_franka_interface_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "franka_interface_msgs" "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/srv/GetCurrentRobotStateCmd.srv" "franka_interface_msgs/Errors:franka_interface_msgs/RobotState:std_msgs/Header"
)

get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/srv/GetCurrentFrankaInterfaceStatusCmd.srv" NAME_WE)
add_custom_target(_franka_interface_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "franka_interface_msgs" "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/srv/GetCurrentFrankaInterfaceStatusCmd.srv" "std_msgs/Header:franka_interface_msgs/FrankaInterfaceStatus"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/Errors.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_cpp(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/FrankaInterfaceStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_cpp(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/RobotState.msg"
  "${MSG_I_FLAGS}"
  "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/Errors.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_cpp(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/RunLoopProcessInfoState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_cpp(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/SensorData.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_cpp(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/SensorDataGroup.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/SensorData.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_cpp(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillAction.msg"
  "${MSG_I_FLAGS}"
  "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionFeedback.msg;/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillResult.msg;/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillGoal.msg;/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionResult.msg;/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_cpp(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillGoal.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_cpp(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_cpp(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_cpp(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_cpp(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_cpp(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/franka_interface_msgs
)

### Generating Services
_generate_srv_cpp(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/srv/GetCurrentRobotStateCmd.srv"
  "${MSG_I_FLAGS}"
  "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/Errors.msg;/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/RobotState.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/franka_interface_msgs
)
_generate_srv_cpp(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/srv/GetCurrentFrankaInterfaceStatusCmd.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/FrankaInterfaceStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/franka_interface_msgs
)

### Generating Module File
_generate_module_cpp(franka_interface_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/franka_interface_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(franka_interface_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(franka_interface_msgs_generate_messages franka_interface_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/Errors.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_cpp _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/FrankaInterfaceStatus.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_cpp _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/RobotState.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_cpp _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/RunLoopProcessInfoState.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_cpp _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/SensorData.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_cpp _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/SensorDataGroup.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_cpp _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillAction.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_cpp _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionGoal.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_cpp _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionResult.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_cpp _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionFeedback.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_cpp _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillGoal.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_cpp _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillResult.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_cpp _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillFeedback.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_cpp _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/srv/GetCurrentRobotStateCmd.srv" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_cpp _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/srv/GetCurrentFrankaInterfaceStatusCmd.srv" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_cpp _franka_interface_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(franka_interface_msgs_gencpp)
add_dependencies(franka_interface_msgs_gencpp franka_interface_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS franka_interface_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/Errors.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_eus(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/FrankaInterfaceStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_eus(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/RobotState.msg"
  "${MSG_I_FLAGS}"
  "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/Errors.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_eus(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/RunLoopProcessInfoState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_eus(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/SensorData.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_eus(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/SensorDataGroup.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/SensorData.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_eus(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillAction.msg"
  "${MSG_I_FLAGS}"
  "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionFeedback.msg;/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillResult.msg;/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillGoal.msg;/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionResult.msg;/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_eus(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillGoal.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_eus(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_eus(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_eus(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_eus(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_eus(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/franka_interface_msgs
)

### Generating Services
_generate_srv_eus(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/srv/GetCurrentRobotStateCmd.srv"
  "${MSG_I_FLAGS}"
  "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/Errors.msg;/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/RobotState.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/franka_interface_msgs
)
_generate_srv_eus(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/srv/GetCurrentFrankaInterfaceStatusCmd.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/FrankaInterfaceStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/franka_interface_msgs
)

### Generating Module File
_generate_module_eus(franka_interface_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/franka_interface_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(franka_interface_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(franka_interface_msgs_generate_messages franka_interface_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/Errors.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_eus _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/FrankaInterfaceStatus.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_eus _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/RobotState.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_eus _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/RunLoopProcessInfoState.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_eus _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/SensorData.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_eus _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/SensorDataGroup.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_eus _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillAction.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_eus _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionGoal.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_eus _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionResult.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_eus _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionFeedback.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_eus _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillGoal.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_eus _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillResult.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_eus _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillFeedback.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_eus _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/srv/GetCurrentRobotStateCmd.srv" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_eus _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/srv/GetCurrentFrankaInterfaceStatusCmd.srv" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_eus _franka_interface_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(franka_interface_msgs_geneus)
add_dependencies(franka_interface_msgs_geneus franka_interface_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS franka_interface_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/Errors.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_lisp(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/FrankaInterfaceStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_lisp(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/RobotState.msg"
  "${MSG_I_FLAGS}"
  "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/Errors.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_lisp(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/RunLoopProcessInfoState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_lisp(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/SensorData.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_lisp(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/SensorDataGroup.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/SensorData.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_lisp(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillAction.msg"
  "${MSG_I_FLAGS}"
  "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionFeedback.msg;/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillResult.msg;/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillGoal.msg;/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionResult.msg;/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_lisp(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillGoal.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_lisp(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_lisp(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_lisp(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_lisp(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_lisp(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/franka_interface_msgs
)

### Generating Services
_generate_srv_lisp(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/srv/GetCurrentRobotStateCmd.srv"
  "${MSG_I_FLAGS}"
  "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/Errors.msg;/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/RobotState.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/franka_interface_msgs
)
_generate_srv_lisp(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/srv/GetCurrentFrankaInterfaceStatusCmd.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/FrankaInterfaceStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/franka_interface_msgs
)

### Generating Module File
_generate_module_lisp(franka_interface_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/franka_interface_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(franka_interface_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(franka_interface_msgs_generate_messages franka_interface_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/Errors.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_lisp _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/FrankaInterfaceStatus.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_lisp _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/RobotState.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_lisp _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/RunLoopProcessInfoState.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_lisp _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/SensorData.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_lisp _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/SensorDataGroup.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_lisp _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillAction.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_lisp _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionGoal.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_lisp _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionResult.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_lisp _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionFeedback.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_lisp _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillGoal.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_lisp _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillResult.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_lisp _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillFeedback.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_lisp _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/srv/GetCurrentRobotStateCmd.srv" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_lisp _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/srv/GetCurrentFrankaInterfaceStatusCmd.srv" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_lisp _franka_interface_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(franka_interface_msgs_genlisp)
add_dependencies(franka_interface_msgs_genlisp franka_interface_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS franka_interface_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/Errors.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_nodejs(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/FrankaInterfaceStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_nodejs(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/RobotState.msg"
  "${MSG_I_FLAGS}"
  "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/Errors.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_nodejs(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/RunLoopProcessInfoState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_nodejs(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/SensorData.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_nodejs(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/SensorDataGroup.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/SensorData.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_nodejs(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillAction.msg"
  "${MSG_I_FLAGS}"
  "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionFeedback.msg;/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillResult.msg;/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillGoal.msg;/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionResult.msg;/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_nodejs(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillGoal.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_nodejs(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_nodejs(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_nodejs(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_nodejs(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_nodejs(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/franka_interface_msgs
)

### Generating Services
_generate_srv_nodejs(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/srv/GetCurrentRobotStateCmd.srv"
  "${MSG_I_FLAGS}"
  "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/Errors.msg;/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/RobotState.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/franka_interface_msgs
)
_generate_srv_nodejs(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/srv/GetCurrentFrankaInterfaceStatusCmd.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/FrankaInterfaceStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/franka_interface_msgs
)

### Generating Module File
_generate_module_nodejs(franka_interface_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/franka_interface_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(franka_interface_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(franka_interface_msgs_generate_messages franka_interface_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/Errors.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_nodejs _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/FrankaInterfaceStatus.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_nodejs _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/RobotState.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_nodejs _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/RunLoopProcessInfoState.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_nodejs _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/SensorData.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_nodejs _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/SensorDataGroup.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_nodejs _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillAction.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_nodejs _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionGoal.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_nodejs _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionResult.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_nodejs _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionFeedback.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_nodejs _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillGoal.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_nodejs _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillResult.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_nodejs _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillFeedback.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_nodejs _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/srv/GetCurrentRobotStateCmd.srv" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_nodejs _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/srv/GetCurrentFrankaInterfaceStatusCmd.srv" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_nodejs _franka_interface_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(franka_interface_msgs_gennodejs)
add_dependencies(franka_interface_msgs_gennodejs franka_interface_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS franka_interface_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/Errors.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_py(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/FrankaInterfaceStatus.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_py(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/RobotState.msg"
  "${MSG_I_FLAGS}"
  "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/Errors.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_py(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/RunLoopProcessInfoState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_py(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/SensorData.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_py(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/SensorDataGroup.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/SensorData.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_py(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillAction.msg"
  "${MSG_I_FLAGS}"
  "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionFeedback.msg;/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillResult.msg;/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillGoal.msg;/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionResult.msg;/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionGoal.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_py(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillGoal.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_py(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillResult.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_py(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillFeedback.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/noetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_py(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_py(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/franka_interface_msgs
)
_generate_msg_py(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/franka_interface_msgs
)

### Generating Services
_generate_srv_py(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/srv/GetCurrentRobotStateCmd.srv"
  "${MSG_I_FLAGS}"
  "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/Errors.msg;/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/RobotState.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/franka_interface_msgs
)
_generate_srv_py(franka_interface_msgs
  "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/srv/GetCurrentFrankaInterfaceStatusCmd.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/FrankaInterfaceStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/franka_interface_msgs
)

### Generating Module File
_generate_module_py(franka_interface_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/franka_interface_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(franka_interface_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(franka_interface_msgs_generate_messages franka_interface_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/Errors.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_py _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/FrankaInterfaceStatus.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_py _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/RobotState.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_py _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/RunLoopProcessInfoState.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_py _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/SensorData.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_py _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/SensorDataGroup.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_py _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillAction.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_py _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionGoal.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_py _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionResult.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_py _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionFeedback.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_py _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillGoal.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_py _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillResult.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_py _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillFeedback.msg" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_py _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/srv/GetCurrentRobotStateCmd.srv" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_py _franka_interface_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/srv/GetCurrentFrankaInterfaceStatusCmd.srv" NAME_WE)
add_dependencies(franka_interface_msgs_generate_messages_py _franka_interface_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(franka_interface_msgs_genpy)
add_dependencies(franka_interface_msgs_genpy franka_interface_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS franka_interface_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/franka_interface_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/franka_interface_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_cpp)
  add_dependencies(franka_interface_msgs_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/franka_interface_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/franka_interface_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_eus)
  add_dependencies(franka_interface_msgs_generate_messages_eus actionlib_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/franka_interface_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/franka_interface_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_lisp)
  add_dependencies(franka_interface_msgs_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/franka_interface_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/franka_interface_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_nodejs)
  add_dependencies(franka_interface_msgs_generate_messages_nodejs actionlib_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/franka_interface_msgs)
  install(CODE "execute_process(COMMAND \"/home/zhouxian/anaconda3/envs/franka/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/franka_interface_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/franka_interface_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_py)
  add_dependencies(franka_interface_msgs_generate_messages_py actionlib_msgs_generate_messages_py)
endif()

# Install script for directory: /home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/zhouxian/git/franka/frankapy/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/zhouxian/git/franka/frankapy/catkin_ws/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/zhouxian/git/franka/frankapy/catkin_ws/install" TYPE PROGRAM FILES "/home/zhouxian/git/franka/frankapy/catkin_ws/build/franka_interface_msgs/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/zhouxian/git/franka/frankapy/catkin_ws/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/zhouxian/git/franka/frankapy/catkin_ws/install" TYPE PROGRAM FILES "/home/zhouxian/git/franka/frankapy/catkin_ws/build/franka_interface_msgs/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/zhouxian/git/franka/frankapy/catkin_ws/install/setup.bash;/home/zhouxian/git/franka/frankapy/catkin_ws/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/zhouxian/git/franka/frankapy/catkin_ws/install" TYPE FILE FILES
    "/home/zhouxian/git/franka/frankapy/catkin_ws/build/franka_interface_msgs/catkin_generated/installspace/setup.bash"
    "/home/zhouxian/git/franka/frankapy/catkin_ws/build/franka_interface_msgs/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/zhouxian/git/franka/frankapy/catkin_ws/install/setup.sh;/home/zhouxian/git/franka/frankapy/catkin_ws/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/zhouxian/git/franka/frankapy/catkin_ws/install" TYPE FILE FILES
    "/home/zhouxian/git/franka/frankapy/catkin_ws/build/franka_interface_msgs/catkin_generated/installspace/setup.sh"
    "/home/zhouxian/git/franka/frankapy/catkin_ws/build/franka_interface_msgs/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/zhouxian/git/franka/frankapy/catkin_ws/install/setup.zsh;/home/zhouxian/git/franka/frankapy/catkin_ws/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/zhouxian/git/franka/frankapy/catkin_ws/install" TYPE FILE FILES
    "/home/zhouxian/git/franka/frankapy/catkin_ws/build/franka_interface_msgs/catkin_generated/installspace/setup.zsh"
    "/home/zhouxian/git/franka/frankapy/catkin_ws/build/franka_interface_msgs/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/zhouxian/git/franka/frankapy/catkin_ws/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/zhouxian/git/franka/frankapy/catkin_ws/install" TYPE FILE FILES "/home/zhouxian/git/franka/frankapy/catkin_ws/build/franka_interface_msgs/catkin_generated/installspace/.rosinstall")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/franka_interface_msgs/msg" TYPE FILE FILES
    "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/Errors.msg"
    "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/FrankaInterfaceStatus.msg"
    "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/RobotState.msg"
    "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/RunLoopProcessInfoState.msg"
    "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/SensorData.msg"
    "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/msg/SensorDataGroup.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/franka_interface_msgs/srv" TYPE FILE FILES
    "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/srv/GetCurrentRobotStateCmd.srv"
    "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/srv/GetCurrentFrankaInterfaceStatusCmd.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/franka_interface_msgs/action" TYPE FILE FILES "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/action/ExecuteSkill.action")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/franka_interface_msgs/msg" TYPE FILE FILES
    "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillAction.msg"
    "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionGoal.msg"
    "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionResult.msg"
    "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillActionFeedback.msg"
    "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillGoal.msg"
    "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillResult.msg"
    "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/franka_interface_msgs/msg/ExecuteSkillFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/franka_interface_msgs/cmake" TYPE FILE FILES "/home/zhouxian/git/franka/frankapy/catkin_ws/build/franka_interface_msgs/catkin_generated/installspace/franka_interface_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/include/franka_interface_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/roseus/ros/franka_interface_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/common-lisp/ros/franka_interface_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/share/gennodejs/ros/franka_interface_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/home/zhouxian/anaconda3/envs/franka/bin/python3" -m compileall "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/lib/python3/dist-packages/franka_interface_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/zhouxian/git/franka/frankapy/catkin_ws/devel/.private/franka_interface_msgs/lib/python3/dist-packages/franka_interface_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/zhouxian/git/franka/frankapy/catkin_ws/build/franka_interface_msgs/catkin_generated/installspace/franka_interface_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/franka_interface_msgs/cmake" TYPE FILE FILES "/home/zhouxian/git/franka/frankapy/catkin_ws/build/franka_interface_msgs/catkin_generated/installspace/franka_interface_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/franka_interface_msgs/cmake" TYPE FILE FILES
    "/home/zhouxian/git/franka/frankapy/catkin_ws/build/franka_interface_msgs/catkin_generated/installspace/franka_interface_msgsConfig.cmake"
    "/home/zhouxian/git/franka/frankapy/catkin_ws/build/franka_interface_msgs/catkin_generated/installspace/franka_interface_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/franka_interface_msgs" TYPE FILE FILES "/home/zhouxian/git/franka/frankapy/catkin_ws/src/franka-interface-msgs/package.xml")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/zhouxian/git/franka/frankapy/catkin_ws/build/franka_interface_msgs/gtest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/zhouxian/git/franka/frankapy/catkin_ws/build/franka_interface_msgs/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")

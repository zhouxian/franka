;; Auto-generated. Do not edit!


(when (boundp 'franka_interface_msgs::GetCurrentRobotStateCmd)
  (if (not (find-package "FRANKA_INTERFACE_MSGS"))
    (make-package "FRANKA_INTERFACE_MSGS"))
  (shadow 'GetCurrentRobotStateCmd (find-package "FRANKA_INTERFACE_MSGS")))
(unless (find-package "FRANKA_INTERFACE_MSGS::GETCURRENTROBOTSTATECMD")
  (make-package "FRANKA_INTERFACE_MSGS::GETCURRENTROBOTSTATECMD"))
(unless (find-package "FRANKA_INTERFACE_MSGS::GETCURRENTROBOTSTATECMDREQUEST")
  (make-package "FRANKA_INTERFACE_MSGS::GETCURRENTROBOTSTATECMDREQUEST"))
(unless (find-package "FRANKA_INTERFACE_MSGS::GETCURRENTROBOTSTATECMDRESPONSE")
  (make-package "FRANKA_INTERFACE_MSGS::GETCURRENTROBOTSTATECMDRESPONSE"))

(in-package "ROS")





(defclass franka_interface_msgs::GetCurrentRobotStateCmdRequest
  :super ros::object
  :slots ())

(defmethod franka_interface_msgs::GetCurrentRobotStateCmdRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass franka_interface_msgs::GetCurrentRobotStateCmdResponse
  :super ros::object
  :slots (_robot_state ))

(defmethod franka_interface_msgs::GetCurrentRobotStateCmdResponse
  (:init
   (&key
    ((:robot_state __robot_state) (instance franka_interface_msgs::RobotState :init))
    )
   (send-super :init)
   (setq _robot_state __robot_state)
   self)
  (:robot_state
   (&rest __robot_state)
   (if (keywordp (car __robot_state))
       (send* _robot_state __robot_state)
     (progn
       (if __robot_state (setq _robot_state (car __robot_state)))
       _robot_state)))
  (:serialization-length
   ()
   (+
    ;; franka_interface_msgs/RobotState _robot_state
    (send _robot_state :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; franka_interface_msgs/RobotState _robot_state
       (send _robot_state :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; franka_interface_msgs/RobotState _robot_state
     (send _robot_state :deserialize buf ptr-) (incf ptr- (send _robot_state :serialization-length))
   ;;
   self)
  )

(defclass franka_interface_msgs::GetCurrentRobotStateCmd
  :super ros::object
  :slots ())

(setf (get franka_interface_msgs::GetCurrentRobotStateCmd :md5sum-) "b518dbe0aba9b1925588d18e42fb85fa")
(setf (get franka_interface_msgs::GetCurrentRobotStateCmd :datatype-) "franka_interface_msgs/GetCurrentRobotStateCmd")
(setf (get franka_interface_msgs::GetCurrentRobotStateCmd :request) franka_interface_msgs::GetCurrentRobotStateCmdRequest)
(setf (get franka_interface_msgs::GetCurrentRobotStateCmd :response) franka_interface_msgs::GetCurrentRobotStateCmdResponse)

(defmethod franka_interface_msgs::GetCurrentRobotStateCmdRequest
  (:response () (instance franka_interface_msgs::GetCurrentRobotStateCmdResponse :init)))

(setf (get franka_interface_msgs::GetCurrentRobotStateCmdRequest :md5sum-) "b518dbe0aba9b1925588d18e42fb85fa")
(setf (get franka_interface_msgs::GetCurrentRobotStateCmdRequest :datatype-) "franka_interface_msgs/GetCurrentRobotStateCmdRequest")
(setf (get franka_interface_msgs::GetCurrentRobotStateCmdRequest :definition-)
      "---
RobotState robot_state

================================================================================
MSG: franka_interface_msgs/RobotState
# Franka robot state
# For more information about each field, look at robot_state.h here: 
# https://github.com/frankaemika/libfranka/blob/master/include/franka/robot_state.h
std_msgs/Header header
float64[16] pose_desired
float64[16] O_T_EE 
float64[16] O_T_EE_d
float64[16] F_T_EE
float64[16] EE_T_K
float64 m_ee
float64[9] I_ee
float64[3] F_x_Cee
float64 m_load
float64[9] I_load
float64[3] F_x_Cload
float64 m_total
float64[9] I_total
float64[3] F_x_Ctotal
float64[2] elbow
float64[2] elbow_d
float64[2] elbow_c
float64[2] delbow_c
float64[2] ddelbow_c
float64[7] tau_J
float64[7] tau_J_d
float64[7] dtau_J
float64[7] q
float64[7] q_d
float64[7] dq
float64[7] dq_d
float64[7] ddq_d
float64[7] joint_contact
float64[6] cartesian_contact
float64[7] joint_collision
float64[6] cartesian_collision
float64[7] tau_ext_hat_filtered
float64[6] O_F_ext_hat_K
float64[6] K_F_ext_hat_K
float64[6] O_dP_EE_d
float64[16] O_T_EE_c
float64[6] O_dP_EE_c
float64[6] O_ddP_EE_c
float64[7] theta
float64[7] dtheta
Errors current_errors
Errors last_motion_errors
float64 control_command_success_rate
uint8 ROBOT_MODE_OTHER=0
uint8 ROBOT_MODE_IDLE=1
uint8 ROBOT_MODE_MOVE=2
uint8 ROBOT_MODE_GUIDING=3
uint8 ROBOT_MODE_REFLEX=4
uint8 ROBOT_MODE_USER_STOPPED=5
uint8 ROBOT_MODE_AUTOMATIC_ERROR_RECOVERY=6
uint8 robot_mode
float64 robot_time
float64 gripper_width
float64 gripper_max_width
bool gripper_is_grasped
uint16 gripper_temperature
float64 gripper_time
bool is_fresh
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: franka_interface_msgs/Errors
bool joint_position_limits_violation
bool cartesian_position_limits_violation
bool self_collision_avoidance_violation
bool joint_velocity_violation
bool cartesian_velocity_violation
bool force_control_safety_violation
bool joint_reflex
bool cartesian_reflex
bool max_goal_pose_deviation_violation
bool max_path_pose_deviation_violation
bool cartesian_velocity_profile_safety_violation
bool joint_position_motion_generator_start_pose_invalid
bool joint_motion_generator_position_limits_violation
bool joint_motion_generator_velocity_limits_violation
bool joint_motion_generator_velocity_discontinuity
bool joint_motion_generator_acceleration_discontinuity
bool cartesian_position_motion_generator_start_pose_invalid
bool cartesian_motion_generator_elbow_limit_violation
bool cartesian_motion_generator_velocity_limits_violation
bool cartesian_motion_generator_velocity_discontinuity
bool cartesian_motion_generator_acceleration_discontinuity
bool cartesian_motion_generator_elbow_sign_inconsistent
bool cartesian_motion_generator_start_elbow_invalid
bool cartesian_motion_generator_joint_position_limits_violation
bool cartesian_motion_generator_joint_velocity_limits_violation
bool cartesian_motion_generator_joint_velocity_discontinuity
bool cartesian_motion_generator_joint_acceleration_discontinuity
bool cartesian_position_motion_generator_invalid_frame
bool force_controller_desired_force_tolerance_violation
bool controller_torque_discontinuity
bool start_elbow_sign_inconsistent
bool communication_constraints_violation
bool power_limit_violation
bool joint_p2p_insufficient_torque_for_planning
bool tau_j_range_violation
bool instability_detected
")

(setf (get franka_interface_msgs::GetCurrentRobotStateCmdResponse :md5sum-) "b518dbe0aba9b1925588d18e42fb85fa")
(setf (get franka_interface_msgs::GetCurrentRobotStateCmdResponse :datatype-) "franka_interface_msgs/GetCurrentRobotStateCmdResponse")
(setf (get franka_interface_msgs::GetCurrentRobotStateCmdResponse :definition-)
      "---
RobotState robot_state

================================================================================
MSG: franka_interface_msgs/RobotState
# Franka robot state
# For more information about each field, look at robot_state.h here: 
# https://github.com/frankaemika/libfranka/blob/master/include/franka/robot_state.h
std_msgs/Header header
float64[16] pose_desired
float64[16] O_T_EE 
float64[16] O_T_EE_d
float64[16] F_T_EE
float64[16] EE_T_K
float64 m_ee
float64[9] I_ee
float64[3] F_x_Cee
float64 m_load
float64[9] I_load
float64[3] F_x_Cload
float64 m_total
float64[9] I_total
float64[3] F_x_Ctotal
float64[2] elbow
float64[2] elbow_d
float64[2] elbow_c
float64[2] delbow_c
float64[2] ddelbow_c
float64[7] tau_J
float64[7] tau_J_d
float64[7] dtau_J
float64[7] q
float64[7] q_d
float64[7] dq
float64[7] dq_d
float64[7] ddq_d
float64[7] joint_contact
float64[6] cartesian_contact
float64[7] joint_collision
float64[6] cartesian_collision
float64[7] tau_ext_hat_filtered
float64[6] O_F_ext_hat_K
float64[6] K_F_ext_hat_K
float64[6] O_dP_EE_d
float64[16] O_T_EE_c
float64[6] O_dP_EE_c
float64[6] O_ddP_EE_c
float64[7] theta
float64[7] dtheta
Errors current_errors
Errors last_motion_errors
float64 control_command_success_rate
uint8 ROBOT_MODE_OTHER=0
uint8 ROBOT_MODE_IDLE=1
uint8 ROBOT_MODE_MOVE=2
uint8 ROBOT_MODE_GUIDING=3
uint8 ROBOT_MODE_REFLEX=4
uint8 ROBOT_MODE_USER_STOPPED=5
uint8 ROBOT_MODE_AUTOMATIC_ERROR_RECOVERY=6
uint8 robot_mode
float64 robot_time
float64 gripper_width
float64 gripper_max_width
bool gripper_is_grasped
uint16 gripper_temperature
float64 gripper_time
bool is_fresh
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: franka_interface_msgs/Errors
bool joint_position_limits_violation
bool cartesian_position_limits_violation
bool self_collision_avoidance_violation
bool joint_velocity_violation
bool cartesian_velocity_violation
bool force_control_safety_violation
bool joint_reflex
bool cartesian_reflex
bool max_goal_pose_deviation_violation
bool max_path_pose_deviation_violation
bool cartesian_velocity_profile_safety_violation
bool joint_position_motion_generator_start_pose_invalid
bool joint_motion_generator_position_limits_violation
bool joint_motion_generator_velocity_limits_violation
bool joint_motion_generator_velocity_discontinuity
bool joint_motion_generator_acceleration_discontinuity
bool cartesian_position_motion_generator_start_pose_invalid
bool cartesian_motion_generator_elbow_limit_violation
bool cartesian_motion_generator_velocity_limits_violation
bool cartesian_motion_generator_velocity_discontinuity
bool cartesian_motion_generator_acceleration_discontinuity
bool cartesian_motion_generator_elbow_sign_inconsistent
bool cartesian_motion_generator_start_elbow_invalid
bool cartesian_motion_generator_joint_position_limits_violation
bool cartesian_motion_generator_joint_velocity_limits_violation
bool cartesian_motion_generator_joint_velocity_discontinuity
bool cartesian_motion_generator_joint_acceleration_discontinuity
bool cartesian_position_motion_generator_invalid_frame
bool force_controller_desired_force_tolerance_violation
bool controller_torque_discontinuity
bool start_elbow_sign_inconsistent
bool communication_constraints_violation
bool power_limit_violation
bool joint_p2p_insufficient_torque_for_planning
bool tau_j_range_violation
bool instability_detected
")



(provide :franka_interface_msgs/GetCurrentRobotStateCmd "b518dbe0aba9b1925588d18e42fb85fa")



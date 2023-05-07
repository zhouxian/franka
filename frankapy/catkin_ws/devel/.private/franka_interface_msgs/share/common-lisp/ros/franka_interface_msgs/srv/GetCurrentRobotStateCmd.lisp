; Auto-generated. Do not edit!


(cl:in-package franka_interface_msgs-srv)


;//! \htmlinclude GetCurrentRobotStateCmd-request.msg.html

(cl:defclass <GetCurrentRobotStateCmd-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetCurrentRobotStateCmd-request (<GetCurrentRobotStateCmd-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetCurrentRobotStateCmd-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetCurrentRobotStateCmd-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name franka_interface_msgs-srv:<GetCurrentRobotStateCmd-request> is deprecated: use franka_interface_msgs-srv:GetCurrentRobotStateCmd-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetCurrentRobotStateCmd-request>) ostream)
  "Serializes a message object of type '<GetCurrentRobotStateCmd-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetCurrentRobotStateCmd-request>) istream)
  "Deserializes a message object of type '<GetCurrentRobotStateCmd-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetCurrentRobotStateCmd-request>)))
  "Returns string type for a service object of type '<GetCurrentRobotStateCmd-request>"
  "franka_interface_msgs/GetCurrentRobotStateCmdRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCurrentRobotStateCmd-request)))
  "Returns string type for a service object of type 'GetCurrentRobotStateCmd-request"
  "franka_interface_msgs/GetCurrentRobotStateCmdRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetCurrentRobotStateCmd-request>)))
  "Returns md5sum for a message object of type '<GetCurrentRobotStateCmd-request>"
  "b518dbe0aba9b1925588d18e42fb85fa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetCurrentRobotStateCmd-request)))
  "Returns md5sum for a message object of type 'GetCurrentRobotStateCmd-request"
  "b518dbe0aba9b1925588d18e42fb85fa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetCurrentRobotStateCmd-request>)))
  "Returns full string definition for message of type '<GetCurrentRobotStateCmd-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetCurrentRobotStateCmd-request)))
  "Returns full string definition for message of type 'GetCurrentRobotStateCmd-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetCurrentRobotStateCmd-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetCurrentRobotStateCmd-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetCurrentRobotStateCmd-request
))
;//! \htmlinclude GetCurrentRobotStateCmd-response.msg.html

(cl:defclass <GetCurrentRobotStateCmd-response> (roslisp-msg-protocol:ros-message)
  ((robot_state
    :reader robot_state
    :initarg :robot_state
    :type franka_interface_msgs-msg:RobotState
    :initform (cl:make-instance 'franka_interface_msgs-msg:RobotState)))
)

(cl:defclass GetCurrentRobotStateCmd-response (<GetCurrentRobotStateCmd-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetCurrentRobotStateCmd-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetCurrentRobotStateCmd-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name franka_interface_msgs-srv:<GetCurrentRobotStateCmd-response> is deprecated: use franka_interface_msgs-srv:GetCurrentRobotStateCmd-response instead.")))

(cl:ensure-generic-function 'robot_state-val :lambda-list '(m))
(cl:defmethod robot_state-val ((m <GetCurrentRobotStateCmd-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-srv:robot_state-val is deprecated.  Use franka_interface_msgs-srv:robot_state instead.")
  (robot_state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetCurrentRobotStateCmd-response>) ostream)
  "Serializes a message object of type '<GetCurrentRobotStateCmd-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'robot_state) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetCurrentRobotStateCmd-response>) istream)
  "Deserializes a message object of type '<GetCurrentRobotStateCmd-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'robot_state) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetCurrentRobotStateCmd-response>)))
  "Returns string type for a service object of type '<GetCurrentRobotStateCmd-response>"
  "franka_interface_msgs/GetCurrentRobotStateCmdResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCurrentRobotStateCmd-response)))
  "Returns string type for a service object of type 'GetCurrentRobotStateCmd-response"
  "franka_interface_msgs/GetCurrentRobotStateCmdResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetCurrentRobotStateCmd-response>)))
  "Returns md5sum for a message object of type '<GetCurrentRobotStateCmd-response>"
  "b518dbe0aba9b1925588d18e42fb85fa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetCurrentRobotStateCmd-response)))
  "Returns md5sum for a message object of type 'GetCurrentRobotStateCmd-response"
  "b518dbe0aba9b1925588d18e42fb85fa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetCurrentRobotStateCmd-response>)))
  "Returns full string definition for message of type '<GetCurrentRobotStateCmd-response>"
  (cl:format cl:nil "RobotState robot_state~%~%================================================================================~%MSG: franka_interface_msgs/RobotState~%# Franka robot state~%# For more information about each field, look at robot_state.h here: ~%# https://github.com/frankaemika/libfranka/blob/master/include/franka/robot_state.h~%std_msgs/Header header~%float64[16] pose_desired~%float64[16] O_T_EE ~%float64[16] O_T_EE_d~%float64[16] F_T_EE~%float64[16] EE_T_K~%float64 m_ee~%float64[9] I_ee~%float64[3] F_x_Cee~%float64 m_load~%float64[9] I_load~%float64[3] F_x_Cload~%float64 m_total~%float64[9] I_total~%float64[3] F_x_Ctotal~%float64[2] elbow~%float64[2] elbow_d~%float64[2] elbow_c~%float64[2] delbow_c~%float64[2] ddelbow_c~%float64[7] tau_J~%float64[7] tau_J_d~%float64[7] dtau_J~%float64[7] q~%float64[7] q_d~%float64[7] dq~%float64[7] dq_d~%float64[7] ddq_d~%float64[7] joint_contact~%float64[6] cartesian_contact~%float64[7] joint_collision~%float64[6] cartesian_collision~%float64[7] tau_ext_hat_filtered~%float64[6] O_F_ext_hat_K~%float64[6] K_F_ext_hat_K~%float64[6] O_dP_EE_d~%float64[16] O_T_EE_c~%float64[6] O_dP_EE_c~%float64[6] O_ddP_EE_c~%float64[7] theta~%float64[7] dtheta~%Errors current_errors~%Errors last_motion_errors~%float64 control_command_success_rate~%uint8 ROBOT_MODE_OTHER=0~%uint8 ROBOT_MODE_IDLE=1~%uint8 ROBOT_MODE_MOVE=2~%uint8 ROBOT_MODE_GUIDING=3~%uint8 ROBOT_MODE_REFLEX=4~%uint8 ROBOT_MODE_USER_STOPPED=5~%uint8 ROBOT_MODE_AUTOMATIC_ERROR_RECOVERY=6~%uint8 robot_mode~%float64 robot_time~%float64 gripper_width~%float64 gripper_max_width~%bool gripper_is_grasped~%uint16 gripper_temperature~%float64 gripper_time~%bool is_fresh~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: franka_interface_msgs/Errors~%bool joint_position_limits_violation~%bool cartesian_position_limits_violation~%bool self_collision_avoidance_violation~%bool joint_velocity_violation~%bool cartesian_velocity_violation~%bool force_control_safety_violation~%bool joint_reflex~%bool cartesian_reflex~%bool max_goal_pose_deviation_violation~%bool max_path_pose_deviation_violation~%bool cartesian_velocity_profile_safety_violation~%bool joint_position_motion_generator_start_pose_invalid~%bool joint_motion_generator_position_limits_violation~%bool joint_motion_generator_velocity_limits_violation~%bool joint_motion_generator_velocity_discontinuity~%bool joint_motion_generator_acceleration_discontinuity~%bool cartesian_position_motion_generator_start_pose_invalid~%bool cartesian_motion_generator_elbow_limit_violation~%bool cartesian_motion_generator_velocity_limits_violation~%bool cartesian_motion_generator_velocity_discontinuity~%bool cartesian_motion_generator_acceleration_discontinuity~%bool cartesian_motion_generator_elbow_sign_inconsistent~%bool cartesian_motion_generator_start_elbow_invalid~%bool cartesian_motion_generator_joint_position_limits_violation~%bool cartesian_motion_generator_joint_velocity_limits_violation~%bool cartesian_motion_generator_joint_velocity_discontinuity~%bool cartesian_motion_generator_joint_acceleration_discontinuity~%bool cartesian_position_motion_generator_invalid_frame~%bool force_controller_desired_force_tolerance_violation~%bool controller_torque_discontinuity~%bool start_elbow_sign_inconsistent~%bool communication_constraints_violation~%bool power_limit_violation~%bool joint_p2p_insufficient_torque_for_planning~%bool tau_j_range_violation~%bool instability_detected~%bool joint_move_in_wrong_direction~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetCurrentRobotStateCmd-response)))
  "Returns full string definition for message of type 'GetCurrentRobotStateCmd-response"
  (cl:format cl:nil "RobotState robot_state~%~%================================================================================~%MSG: franka_interface_msgs/RobotState~%# Franka robot state~%# For more information about each field, look at robot_state.h here: ~%# https://github.com/frankaemika/libfranka/blob/master/include/franka/robot_state.h~%std_msgs/Header header~%float64[16] pose_desired~%float64[16] O_T_EE ~%float64[16] O_T_EE_d~%float64[16] F_T_EE~%float64[16] EE_T_K~%float64 m_ee~%float64[9] I_ee~%float64[3] F_x_Cee~%float64 m_load~%float64[9] I_load~%float64[3] F_x_Cload~%float64 m_total~%float64[9] I_total~%float64[3] F_x_Ctotal~%float64[2] elbow~%float64[2] elbow_d~%float64[2] elbow_c~%float64[2] delbow_c~%float64[2] ddelbow_c~%float64[7] tau_J~%float64[7] tau_J_d~%float64[7] dtau_J~%float64[7] q~%float64[7] q_d~%float64[7] dq~%float64[7] dq_d~%float64[7] ddq_d~%float64[7] joint_contact~%float64[6] cartesian_contact~%float64[7] joint_collision~%float64[6] cartesian_collision~%float64[7] tau_ext_hat_filtered~%float64[6] O_F_ext_hat_K~%float64[6] K_F_ext_hat_K~%float64[6] O_dP_EE_d~%float64[16] O_T_EE_c~%float64[6] O_dP_EE_c~%float64[6] O_ddP_EE_c~%float64[7] theta~%float64[7] dtheta~%Errors current_errors~%Errors last_motion_errors~%float64 control_command_success_rate~%uint8 ROBOT_MODE_OTHER=0~%uint8 ROBOT_MODE_IDLE=1~%uint8 ROBOT_MODE_MOVE=2~%uint8 ROBOT_MODE_GUIDING=3~%uint8 ROBOT_MODE_REFLEX=4~%uint8 ROBOT_MODE_USER_STOPPED=5~%uint8 ROBOT_MODE_AUTOMATIC_ERROR_RECOVERY=6~%uint8 robot_mode~%float64 robot_time~%float64 gripper_width~%float64 gripper_max_width~%bool gripper_is_grasped~%uint16 gripper_temperature~%float64 gripper_time~%bool is_fresh~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: franka_interface_msgs/Errors~%bool joint_position_limits_violation~%bool cartesian_position_limits_violation~%bool self_collision_avoidance_violation~%bool joint_velocity_violation~%bool cartesian_velocity_violation~%bool force_control_safety_violation~%bool joint_reflex~%bool cartesian_reflex~%bool max_goal_pose_deviation_violation~%bool max_path_pose_deviation_violation~%bool cartesian_velocity_profile_safety_violation~%bool joint_position_motion_generator_start_pose_invalid~%bool joint_motion_generator_position_limits_violation~%bool joint_motion_generator_velocity_limits_violation~%bool joint_motion_generator_velocity_discontinuity~%bool joint_motion_generator_acceleration_discontinuity~%bool cartesian_position_motion_generator_start_pose_invalid~%bool cartesian_motion_generator_elbow_limit_violation~%bool cartesian_motion_generator_velocity_limits_violation~%bool cartesian_motion_generator_velocity_discontinuity~%bool cartesian_motion_generator_acceleration_discontinuity~%bool cartesian_motion_generator_elbow_sign_inconsistent~%bool cartesian_motion_generator_start_elbow_invalid~%bool cartesian_motion_generator_joint_position_limits_violation~%bool cartesian_motion_generator_joint_velocity_limits_violation~%bool cartesian_motion_generator_joint_velocity_discontinuity~%bool cartesian_motion_generator_joint_acceleration_discontinuity~%bool cartesian_position_motion_generator_invalid_frame~%bool force_controller_desired_force_tolerance_violation~%bool controller_torque_discontinuity~%bool start_elbow_sign_inconsistent~%bool communication_constraints_violation~%bool power_limit_violation~%bool joint_p2p_insufficient_torque_for_planning~%bool tau_j_range_violation~%bool instability_detected~%bool joint_move_in_wrong_direction~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetCurrentRobotStateCmd-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'robot_state))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetCurrentRobotStateCmd-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetCurrentRobotStateCmd-response
    (cl:cons ':robot_state (robot_state msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetCurrentRobotStateCmd)))
  'GetCurrentRobotStateCmd-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetCurrentRobotStateCmd)))
  'GetCurrentRobotStateCmd-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCurrentRobotStateCmd)))
  "Returns string type for a service object of type '<GetCurrentRobotStateCmd>"
  "franka_interface_msgs/GetCurrentRobotStateCmd")
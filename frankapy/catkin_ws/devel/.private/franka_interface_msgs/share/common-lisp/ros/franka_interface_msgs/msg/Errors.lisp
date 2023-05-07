; Auto-generated. Do not edit!


(cl:in-package franka_interface_msgs-msg)


;//! \htmlinclude Errors.msg.html

(cl:defclass <Errors> (roslisp-msg-protocol:ros-message)
  ((joint_position_limits_violation
    :reader joint_position_limits_violation
    :initarg :joint_position_limits_violation
    :type cl:boolean
    :initform cl:nil)
   (cartesian_position_limits_violation
    :reader cartesian_position_limits_violation
    :initarg :cartesian_position_limits_violation
    :type cl:boolean
    :initform cl:nil)
   (self_collision_avoidance_violation
    :reader self_collision_avoidance_violation
    :initarg :self_collision_avoidance_violation
    :type cl:boolean
    :initform cl:nil)
   (joint_velocity_violation
    :reader joint_velocity_violation
    :initarg :joint_velocity_violation
    :type cl:boolean
    :initform cl:nil)
   (cartesian_velocity_violation
    :reader cartesian_velocity_violation
    :initarg :cartesian_velocity_violation
    :type cl:boolean
    :initform cl:nil)
   (force_control_safety_violation
    :reader force_control_safety_violation
    :initarg :force_control_safety_violation
    :type cl:boolean
    :initform cl:nil)
   (joint_reflex
    :reader joint_reflex
    :initarg :joint_reflex
    :type cl:boolean
    :initform cl:nil)
   (cartesian_reflex
    :reader cartesian_reflex
    :initarg :cartesian_reflex
    :type cl:boolean
    :initform cl:nil)
   (max_goal_pose_deviation_violation
    :reader max_goal_pose_deviation_violation
    :initarg :max_goal_pose_deviation_violation
    :type cl:boolean
    :initform cl:nil)
   (max_path_pose_deviation_violation
    :reader max_path_pose_deviation_violation
    :initarg :max_path_pose_deviation_violation
    :type cl:boolean
    :initform cl:nil)
   (cartesian_velocity_profile_safety_violation
    :reader cartesian_velocity_profile_safety_violation
    :initarg :cartesian_velocity_profile_safety_violation
    :type cl:boolean
    :initform cl:nil)
   (joint_position_motion_generator_start_pose_invalid
    :reader joint_position_motion_generator_start_pose_invalid
    :initarg :joint_position_motion_generator_start_pose_invalid
    :type cl:boolean
    :initform cl:nil)
   (joint_motion_generator_position_limits_violation
    :reader joint_motion_generator_position_limits_violation
    :initarg :joint_motion_generator_position_limits_violation
    :type cl:boolean
    :initform cl:nil)
   (joint_motion_generator_velocity_limits_violation
    :reader joint_motion_generator_velocity_limits_violation
    :initarg :joint_motion_generator_velocity_limits_violation
    :type cl:boolean
    :initform cl:nil)
   (joint_motion_generator_velocity_discontinuity
    :reader joint_motion_generator_velocity_discontinuity
    :initarg :joint_motion_generator_velocity_discontinuity
    :type cl:boolean
    :initform cl:nil)
   (joint_motion_generator_acceleration_discontinuity
    :reader joint_motion_generator_acceleration_discontinuity
    :initarg :joint_motion_generator_acceleration_discontinuity
    :type cl:boolean
    :initform cl:nil)
   (cartesian_position_motion_generator_start_pose_invalid
    :reader cartesian_position_motion_generator_start_pose_invalid
    :initarg :cartesian_position_motion_generator_start_pose_invalid
    :type cl:boolean
    :initform cl:nil)
   (cartesian_motion_generator_elbow_limit_violation
    :reader cartesian_motion_generator_elbow_limit_violation
    :initarg :cartesian_motion_generator_elbow_limit_violation
    :type cl:boolean
    :initform cl:nil)
   (cartesian_motion_generator_velocity_limits_violation
    :reader cartesian_motion_generator_velocity_limits_violation
    :initarg :cartesian_motion_generator_velocity_limits_violation
    :type cl:boolean
    :initform cl:nil)
   (cartesian_motion_generator_velocity_discontinuity
    :reader cartesian_motion_generator_velocity_discontinuity
    :initarg :cartesian_motion_generator_velocity_discontinuity
    :type cl:boolean
    :initform cl:nil)
   (cartesian_motion_generator_acceleration_discontinuity
    :reader cartesian_motion_generator_acceleration_discontinuity
    :initarg :cartesian_motion_generator_acceleration_discontinuity
    :type cl:boolean
    :initform cl:nil)
   (cartesian_motion_generator_elbow_sign_inconsistent
    :reader cartesian_motion_generator_elbow_sign_inconsistent
    :initarg :cartesian_motion_generator_elbow_sign_inconsistent
    :type cl:boolean
    :initform cl:nil)
   (cartesian_motion_generator_start_elbow_invalid
    :reader cartesian_motion_generator_start_elbow_invalid
    :initarg :cartesian_motion_generator_start_elbow_invalid
    :type cl:boolean
    :initform cl:nil)
   (cartesian_motion_generator_joint_position_limits_violation
    :reader cartesian_motion_generator_joint_position_limits_violation
    :initarg :cartesian_motion_generator_joint_position_limits_violation
    :type cl:boolean
    :initform cl:nil)
   (cartesian_motion_generator_joint_velocity_limits_violation
    :reader cartesian_motion_generator_joint_velocity_limits_violation
    :initarg :cartesian_motion_generator_joint_velocity_limits_violation
    :type cl:boolean
    :initform cl:nil)
   (cartesian_motion_generator_joint_velocity_discontinuity
    :reader cartesian_motion_generator_joint_velocity_discontinuity
    :initarg :cartesian_motion_generator_joint_velocity_discontinuity
    :type cl:boolean
    :initform cl:nil)
   (cartesian_motion_generator_joint_acceleration_discontinuity
    :reader cartesian_motion_generator_joint_acceleration_discontinuity
    :initarg :cartesian_motion_generator_joint_acceleration_discontinuity
    :type cl:boolean
    :initform cl:nil)
   (cartesian_position_motion_generator_invalid_frame
    :reader cartesian_position_motion_generator_invalid_frame
    :initarg :cartesian_position_motion_generator_invalid_frame
    :type cl:boolean
    :initform cl:nil)
   (force_controller_desired_force_tolerance_violation
    :reader force_controller_desired_force_tolerance_violation
    :initarg :force_controller_desired_force_tolerance_violation
    :type cl:boolean
    :initform cl:nil)
   (controller_torque_discontinuity
    :reader controller_torque_discontinuity
    :initarg :controller_torque_discontinuity
    :type cl:boolean
    :initform cl:nil)
   (start_elbow_sign_inconsistent
    :reader start_elbow_sign_inconsistent
    :initarg :start_elbow_sign_inconsistent
    :type cl:boolean
    :initform cl:nil)
   (communication_constraints_violation
    :reader communication_constraints_violation
    :initarg :communication_constraints_violation
    :type cl:boolean
    :initform cl:nil)
   (power_limit_violation
    :reader power_limit_violation
    :initarg :power_limit_violation
    :type cl:boolean
    :initform cl:nil)
   (joint_p2p_insufficient_torque_for_planning
    :reader joint_p2p_insufficient_torque_for_planning
    :initarg :joint_p2p_insufficient_torque_for_planning
    :type cl:boolean
    :initform cl:nil)
   (tau_j_range_violation
    :reader tau_j_range_violation
    :initarg :tau_j_range_violation
    :type cl:boolean
    :initform cl:nil)
   (instability_detected
    :reader instability_detected
    :initarg :instability_detected
    :type cl:boolean
    :initform cl:nil)
   (joint_move_in_wrong_direction
    :reader joint_move_in_wrong_direction
    :initarg :joint_move_in_wrong_direction
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Errors (<Errors>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Errors>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Errors)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name franka_interface_msgs-msg:<Errors> is deprecated: use franka_interface_msgs-msg:Errors instead.")))

(cl:ensure-generic-function 'joint_position_limits_violation-val :lambda-list '(m))
(cl:defmethod joint_position_limits_violation-val ((m <Errors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:joint_position_limits_violation-val is deprecated.  Use franka_interface_msgs-msg:joint_position_limits_violation instead.")
  (joint_position_limits_violation m))

(cl:ensure-generic-function 'cartesian_position_limits_violation-val :lambda-list '(m))
(cl:defmethod cartesian_position_limits_violation-val ((m <Errors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:cartesian_position_limits_violation-val is deprecated.  Use franka_interface_msgs-msg:cartesian_position_limits_violation instead.")
  (cartesian_position_limits_violation m))

(cl:ensure-generic-function 'self_collision_avoidance_violation-val :lambda-list '(m))
(cl:defmethod self_collision_avoidance_violation-val ((m <Errors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:self_collision_avoidance_violation-val is deprecated.  Use franka_interface_msgs-msg:self_collision_avoidance_violation instead.")
  (self_collision_avoidance_violation m))

(cl:ensure-generic-function 'joint_velocity_violation-val :lambda-list '(m))
(cl:defmethod joint_velocity_violation-val ((m <Errors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:joint_velocity_violation-val is deprecated.  Use franka_interface_msgs-msg:joint_velocity_violation instead.")
  (joint_velocity_violation m))

(cl:ensure-generic-function 'cartesian_velocity_violation-val :lambda-list '(m))
(cl:defmethod cartesian_velocity_violation-val ((m <Errors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:cartesian_velocity_violation-val is deprecated.  Use franka_interface_msgs-msg:cartesian_velocity_violation instead.")
  (cartesian_velocity_violation m))

(cl:ensure-generic-function 'force_control_safety_violation-val :lambda-list '(m))
(cl:defmethod force_control_safety_violation-val ((m <Errors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:force_control_safety_violation-val is deprecated.  Use franka_interface_msgs-msg:force_control_safety_violation instead.")
  (force_control_safety_violation m))

(cl:ensure-generic-function 'joint_reflex-val :lambda-list '(m))
(cl:defmethod joint_reflex-val ((m <Errors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:joint_reflex-val is deprecated.  Use franka_interface_msgs-msg:joint_reflex instead.")
  (joint_reflex m))

(cl:ensure-generic-function 'cartesian_reflex-val :lambda-list '(m))
(cl:defmethod cartesian_reflex-val ((m <Errors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:cartesian_reflex-val is deprecated.  Use franka_interface_msgs-msg:cartesian_reflex instead.")
  (cartesian_reflex m))

(cl:ensure-generic-function 'max_goal_pose_deviation_violation-val :lambda-list '(m))
(cl:defmethod max_goal_pose_deviation_violation-val ((m <Errors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:max_goal_pose_deviation_violation-val is deprecated.  Use franka_interface_msgs-msg:max_goal_pose_deviation_violation instead.")
  (max_goal_pose_deviation_violation m))

(cl:ensure-generic-function 'max_path_pose_deviation_violation-val :lambda-list '(m))
(cl:defmethod max_path_pose_deviation_violation-val ((m <Errors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:max_path_pose_deviation_violation-val is deprecated.  Use franka_interface_msgs-msg:max_path_pose_deviation_violation instead.")
  (max_path_pose_deviation_violation m))

(cl:ensure-generic-function 'cartesian_velocity_profile_safety_violation-val :lambda-list '(m))
(cl:defmethod cartesian_velocity_profile_safety_violation-val ((m <Errors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:cartesian_velocity_profile_safety_violation-val is deprecated.  Use franka_interface_msgs-msg:cartesian_velocity_profile_safety_violation instead.")
  (cartesian_velocity_profile_safety_violation m))

(cl:ensure-generic-function 'joint_position_motion_generator_start_pose_invalid-val :lambda-list '(m))
(cl:defmethod joint_position_motion_generator_start_pose_invalid-val ((m <Errors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:joint_position_motion_generator_start_pose_invalid-val is deprecated.  Use franka_interface_msgs-msg:joint_position_motion_generator_start_pose_invalid instead.")
  (joint_position_motion_generator_start_pose_invalid m))

(cl:ensure-generic-function 'joint_motion_generator_position_limits_violation-val :lambda-list '(m))
(cl:defmethod joint_motion_generator_position_limits_violation-val ((m <Errors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:joint_motion_generator_position_limits_violation-val is deprecated.  Use franka_interface_msgs-msg:joint_motion_generator_position_limits_violation instead.")
  (joint_motion_generator_position_limits_violation m))

(cl:ensure-generic-function 'joint_motion_generator_velocity_limits_violation-val :lambda-list '(m))
(cl:defmethod joint_motion_generator_velocity_limits_violation-val ((m <Errors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:joint_motion_generator_velocity_limits_violation-val is deprecated.  Use franka_interface_msgs-msg:joint_motion_generator_velocity_limits_violation instead.")
  (joint_motion_generator_velocity_limits_violation m))

(cl:ensure-generic-function 'joint_motion_generator_velocity_discontinuity-val :lambda-list '(m))
(cl:defmethod joint_motion_generator_velocity_discontinuity-val ((m <Errors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:joint_motion_generator_velocity_discontinuity-val is deprecated.  Use franka_interface_msgs-msg:joint_motion_generator_velocity_discontinuity instead.")
  (joint_motion_generator_velocity_discontinuity m))

(cl:ensure-generic-function 'joint_motion_generator_acceleration_discontinuity-val :lambda-list '(m))
(cl:defmethod joint_motion_generator_acceleration_discontinuity-val ((m <Errors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:joint_motion_generator_acceleration_discontinuity-val is deprecated.  Use franka_interface_msgs-msg:joint_motion_generator_acceleration_discontinuity instead.")
  (joint_motion_generator_acceleration_discontinuity m))

(cl:ensure-generic-function 'cartesian_position_motion_generator_start_pose_invalid-val :lambda-list '(m))
(cl:defmethod cartesian_position_motion_generator_start_pose_invalid-val ((m <Errors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:cartesian_position_motion_generator_start_pose_invalid-val is deprecated.  Use franka_interface_msgs-msg:cartesian_position_motion_generator_start_pose_invalid instead.")
  (cartesian_position_motion_generator_start_pose_invalid m))

(cl:ensure-generic-function 'cartesian_motion_generator_elbow_limit_violation-val :lambda-list '(m))
(cl:defmethod cartesian_motion_generator_elbow_limit_violation-val ((m <Errors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:cartesian_motion_generator_elbow_limit_violation-val is deprecated.  Use franka_interface_msgs-msg:cartesian_motion_generator_elbow_limit_violation instead.")
  (cartesian_motion_generator_elbow_limit_violation m))

(cl:ensure-generic-function 'cartesian_motion_generator_velocity_limits_violation-val :lambda-list '(m))
(cl:defmethod cartesian_motion_generator_velocity_limits_violation-val ((m <Errors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:cartesian_motion_generator_velocity_limits_violation-val is deprecated.  Use franka_interface_msgs-msg:cartesian_motion_generator_velocity_limits_violation instead.")
  (cartesian_motion_generator_velocity_limits_violation m))

(cl:ensure-generic-function 'cartesian_motion_generator_velocity_discontinuity-val :lambda-list '(m))
(cl:defmethod cartesian_motion_generator_velocity_discontinuity-val ((m <Errors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:cartesian_motion_generator_velocity_discontinuity-val is deprecated.  Use franka_interface_msgs-msg:cartesian_motion_generator_velocity_discontinuity instead.")
  (cartesian_motion_generator_velocity_discontinuity m))

(cl:ensure-generic-function 'cartesian_motion_generator_acceleration_discontinuity-val :lambda-list '(m))
(cl:defmethod cartesian_motion_generator_acceleration_discontinuity-val ((m <Errors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:cartesian_motion_generator_acceleration_discontinuity-val is deprecated.  Use franka_interface_msgs-msg:cartesian_motion_generator_acceleration_discontinuity instead.")
  (cartesian_motion_generator_acceleration_discontinuity m))

(cl:ensure-generic-function 'cartesian_motion_generator_elbow_sign_inconsistent-val :lambda-list '(m))
(cl:defmethod cartesian_motion_generator_elbow_sign_inconsistent-val ((m <Errors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:cartesian_motion_generator_elbow_sign_inconsistent-val is deprecated.  Use franka_interface_msgs-msg:cartesian_motion_generator_elbow_sign_inconsistent instead.")
  (cartesian_motion_generator_elbow_sign_inconsistent m))

(cl:ensure-generic-function 'cartesian_motion_generator_start_elbow_invalid-val :lambda-list '(m))
(cl:defmethod cartesian_motion_generator_start_elbow_invalid-val ((m <Errors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:cartesian_motion_generator_start_elbow_invalid-val is deprecated.  Use franka_interface_msgs-msg:cartesian_motion_generator_start_elbow_invalid instead.")
  (cartesian_motion_generator_start_elbow_invalid m))

(cl:ensure-generic-function 'cartesian_motion_generator_joint_position_limits_violation-val :lambda-list '(m))
(cl:defmethod cartesian_motion_generator_joint_position_limits_violation-val ((m <Errors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:cartesian_motion_generator_joint_position_limits_violation-val is deprecated.  Use franka_interface_msgs-msg:cartesian_motion_generator_joint_position_limits_violation instead.")
  (cartesian_motion_generator_joint_position_limits_violation m))

(cl:ensure-generic-function 'cartesian_motion_generator_joint_velocity_limits_violation-val :lambda-list '(m))
(cl:defmethod cartesian_motion_generator_joint_velocity_limits_violation-val ((m <Errors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:cartesian_motion_generator_joint_velocity_limits_violation-val is deprecated.  Use franka_interface_msgs-msg:cartesian_motion_generator_joint_velocity_limits_violation instead.")
  (cartesian_motion_generator_joint_velocity_limits_violation m))

(cl:ensure-generic-function 'cartesian_motion_generator_joint_velocity_discontinuity-val :lambda-list '(m))
(cl:defmethod cartesian_motion_generator_joint_velocity_discontinuity-val ((m <Errors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:cartesian_motion_generator_joint_velocity_discontinuity-val is deprecated.  Use franka_interface_msgs-msg:cartesian_motion_generator_joint_velocity_discontinuity instead.")
  (cartesian_motion_generator_joint_velocity_discontinuity m))

(cl:ensure-generic-function 'cartesian_motion_generator_joint_acceleration_discontinuity-val :lambda-list '(m))
(cl:defmethod cartesian_motion_generator_joint_acceleration_discontinuity-val ((m <Errors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:cartesian_motion_generator_joint_acceleration_discontinuity-val is deprecated.  Use franka_interface_msgs-msg:cartesian_motion_generator_joint_acceleration_discontinuity instead.")
  (cartesian_motion_generator_joint_acceleration_discontinuity m))

(cl:ensure-generic-function 'cartesian_position_motion_generator_invalid_frame-val :lambda-list '(m))
(cl:defmethod cartesian_position_motion_generator_invalid_frame-val ((m <Errors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:cartesian_position_motion_generator_invalid_frame-val is deprecated.  Use franka_interface_msgs-msg:cartesian_position_motion_generator_invalid_frame instead.")
  (cartesian_position_motion_generator_invalid_frame m))

(cl:ensure-generic-function 'force_controller_desired_force_tolerance_violation-val :lambda-list '(m))
(cl:defmethod force_controller_desired_force_tolerance_violation-val ((m <Errors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:force_controller_desired_force_tolerance_violation-val is deprecated.  Use franka_interface_msgs-msg:force_controller_desired_force_tolerance_violation instead.")
  (force_controller_desired_force_tolerance_violation m))

(cl:ensure-generic-function 'controller_torque_discontinuity-val :lambda-list '(m))
(cl:defmethod controller_torque_discontinuity-val ((m <Errors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:controller_torque_discontinuity-val is deprecated.  Use franka_interface_msgs-msg:controller_torque_discontinuity instead.")
  (controller_torque_discontinuity m))

(cl:ensure-generic-function 'start_elbow_sign_inconsistent-val :lambda-list '(m))
(cl:defmethod start_elbow_sign_inconsistent-val ((m <Errors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:start_elbow_sign_inconsistent-val is deprecated.  Use franka_interface_msgs-msg:start_elbow_sign_inconsistent instead.")
  (start_elbow_sign_inconsistent m))

(cl:ensure-generic-function 'communication_constraints_violation-val :lambda-list '(m))
(cl:defmethod communication_constraints_violation-val ((m <Errors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:communication_constraints_violation-val is deprecated.  Use franka_interface_msgs-msg:communication_constraints_violation instead.")
  (communication_constraints_violation m))

(cl:ensure-generic-function 'power_limit_violation-val :lambda-list '(m))
(cl:defmethod power_limit_violation-val ((m <Errors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:power_limit_violation-val is deprecated.  Use franka_interface_msgs-msg:power_limit_violation instead.")
  (power_limit_violation m))

(cl:ensure-generic-function 'joint_p2p_insufficient_torque_for_planning-val :lambda-list '(m))
(cl:defmethod joint_p2p_insufficient_torque_for_planning-val ((m <Errors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:joint_p2p_insufficient_torque_for_planning-val is deprecated.  Use franka_interface_msgs-msg:joint_p2p_insufficient_torque_for_planning instead.")
  (joint_p2p_insufficient_torque_for_planning m))

(cl:ensure-generic-function 'tau_j_range_violation-val :lambda-list '(m))
(cl:defmethod tau_j_range_violation-val ((m <Errors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:tau_j_range_violation-val is deprecated.  Use franka_interface_msgs-msg:tau_j_range_violation instead.")
  (tau_j_range_violation m))

(cl:ensure-generic-function 'instability_detected-val :lambda-list '(m))
(cl:defmethod instability_detected-val ((m <Errors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:instability_detected-val is deprecated.  Use franka_interface_msgs-msg:instability_detected instead.")
  (instability_detected m))

(cl:ensure-generic-function 'joint_move_in_wrong_direction-val :lambda-list '(m))
(cl:defmethod joint_move_in_wrong_direction-val ((m <Errors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:joint_move_in_wrong_direction-val is deprecated.  Use franka_interface_msgs-msg:joint_move_in_wrong_direction instead.")
  (joint_move_in_wrong_direction m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Errors>) ostream)
  "Serializes a message object of type '<Errors>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'joint_position_limits_violation) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'cartesian_position_limits_violation) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'self_collision_avoidance_violation) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'joint_velocity_violation) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'cartesian_velocity_violation) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'force_control_safety_violation) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'joint_reflex) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'cartesian_reflex) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'max_goal_pose_deviation_violation) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'max_path_pose_deviation_violation) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'cartesian_velocity_profile_safety_violation) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'joint_position_motion_generator_start_pose_invalid) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'joint_motion_generator_position_limits_violation) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'joint_motion_generator_velocity_limits_violation) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'joint_motion_generator_velocity_discontinuity) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'joint_motion_generator_acceleration_discontinuity) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'cartesian_position_motion_generator_start_pose_invalid) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'cartesian_motion_generator_elbow_limit_violation) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'cartesian_motion_generator_velocity_limits_violation) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'cartesian_motion_generator_velocity_discontinuity) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'cartesian_motion_generator_acceleration_discontinuity) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'cartesian_motion_generator_elbow_sign_inconsistent) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'cartesian_motion_generator_start_elbow_invalid) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'cartesian_motion_generator_joint_position_limits_violation) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'cartesian_motion_generator_joint_velocity_limits_violation) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'cartesian_motion_generator_joint_velocity_discontinuity) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'cartesian_motion_generator_joint_acceleration_discontinuity) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'cartesian_position_motion_generator_invalid_frame) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'force_controller_desired_force_tolerance_violation) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'controller_torque_discontinuity) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'start_elbow_sign_inconsistent) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'communication_constraints_violation) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'power_limit_violation) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'joint_p2p_insufficient_torque_for_planning) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'tau_j_range_violation) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'instability_detected) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'joint_move_in_wrong_direction) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Errors>) istream)
  "Deserializes a message object of type '<Errors>"
    (cl:setf (cl:slot-value msg 'joint_position_limits_violation) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'cartesian_position_limits_violation) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'self_collision_avoidance_violation) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'joint_velocity_violation) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'cartesian_velocity_violation) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'force_control_safety_violation) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'joint_reflex) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'cartesian_reflex) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'max_goal_pose_deviation_violation) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'max_path_pose_deviation_violation) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'cartesian_velocity_profile_safety_violation) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'joint_position_motion_generator_start_pose_invalid) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'joint_motion_generator_position_limits_violation) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'joint_motion_generator_velocity_limits_violation) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'joint_motion_generator_velocity_discontinuity) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'joint_motion_generator_acceleration_discontinuity) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'cartesian_position_motion_generator_start_pose_invalid) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'cartesian_motion_generator_elbow_limit_violation) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'cartesian_motion_generator_velocity_limits_violation) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'cartesian_motion_generator_velocity_discontinuity) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'cartesian_motion_generator_acceleration_discontinuity) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'cartesian_motion_generator_elbow_sign_inconsistent) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'cartesian_motion_generator_start_elbow_invalid) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'cartesian_motion_generator_joint_position_limits_violation) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'cartesian_motion_generator_joint_velocity_limits_violation) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'cartesian_motion_generator_joint_velocity_discontinuity) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'cartesian_motion_generator_joint_acceleration_discontinuity) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'cartesian_position_motion_generator_invalid_frame) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'force_controller_desired_force_tolerance_violation) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'controller_torque_discontinuity) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'start_elbow_sign_inconsistent) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'communication_constraints_violation) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'power_limit_violation) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'joint_p2p_insufficient_torque_for_planning) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'tau_j_range_violation) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'instability_detected) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'joint_move_in_wrong_direction) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Errors>)))
  "Returns string type for a message object of type '<Errors>"
  "franka_interface_msgs/Errors")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Errors)))
  "Returns string type for a message object of type 'Errors"
  "franka_interface_msgs/Errors")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Errors>)))
  "Returns md5sum for a message object of type '<Errors>"
  "b93c14a039f7f69d7321c619961e4708")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Errors)))
  "Returns md5sum for a message object of type 'Errors"
  "b93c14a039f7f69d7321c619961e4708")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Errors>)))
  "Returns full string definition for message of type '<Errors>"
  (cl:format cl:nil "bool joint_position_limits_violation~%bool cartesian_position_limits_violation~%bool self_collision_avoidance_violation~%bool joint_velocity_violation~%bool cartesian_velocity_violation~%bool force_control_safety_violation~%bool joint_reflex~%bool cartesian_reflex~%bool max_goal_pose_deviation_violation~%bool max_path_pose_deviation_violation~%bool cartesian_velocity_profile_safety_violation~%bool joint_position_motion_generator_start_pose_invalid~%bool joint_motion_generator_position_limits_violation~%bool joint_motion_generator_velocity_limits_violation~%bool joint_motion_generator_velocity_discontinuity~%bool joint_motion_generator_acceleration_discontinuity~%bool cartesian_position_motion_generator_start_pose_invalid~%bool cartesian_motion_generator_elbow_limit_violation~%bool cartesian_motion_generator_velocity_limits_violation~%bool cartesian_motion_generator_velocity_discontinuity~%bool cartesian_motion_generator_acceleration_discontinuity~%bool cartesian_motion_generator_elbow_sign_inconsistent~%bool cartesian_motion_generator_start_elbow_invalid~%bool cartesian_motion_generator_joint_position_limits_violation~%bool cartesian_motion_generator_joint_velocity_limits_violation~%bool cartesian_motion_generator_joint_velocity_discontinuity~%bool cartesian_motion_generator_joint_acceleration_discontinuity~%bool cartesian_position_motion_generator_invalid_frame~%bool force_controller_desired_force_tolerance_violation~%bool controller_torque_discontinuity~%bool start_elbow_sign_inconsistent~%bool communication_constraints_violation~%bool power_limit_violation~%bool joint_p2p_insufficient_torque_for_planning~%bool tau_j_range_violation~%bool instability_detected~%bool joint_move_in_wrong_direction~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Errors)))
  "Returns full string definition for message of type 'Errors"
  (cl:format cl:nil "bool joint_position_limits_violation~%bool cartesian_position_limits_violation~%bool self_collision_avoidance_violation~%bool joint_velocity_violation~%bool cartesian_velocity_violation~%bool force_control_safety_violation~%bool joint_reflex~%bool cartesian_reflex~%bool max_goal_pose_deviation_violation~%bool max_path_pose_deviation_violation~%bool cartesian_velocity_profile_safety_violation~%bool joint_position_motion_generator_start_pose_invalid~%bool joint_motion_generator_position_limits_violation~%bool joint_motion_generator_velocity_limits_violation~%bool joint_motion_generator_velocity_discontinuity~%bool joint_motion_generator_acceleration_discontinuity~%bool cartesian_position_motion_generator_start_pose_invalid~%bool cartesian_motion_generator_elbow_limit_violation~%bool cartesian_motion_generator_velocity_limits_violation~%bool cartesian_motion_generator_velocity_discontinuity~%bool cartesian_motion_generator_acceleration_discontinuity~%bool cartesian_motion_generator_elbow_sign_inconsistent~%bool cartesian_motion_generator_start_elbow_invalid~%bool cartesian_motion_generator_joint_position_limits_violation~%bool cartesian_motion_generator_joint_velocity_limits_violation~%bool cartesian_motion_generator_joint_velocity_discontinuity~%bool cartesian_motion_generator_joint_acceleration_discontinuity~%bool cartesian_position_motion_generator_invalid_frame~%bool force_controller_desired_force_tolerance_violation~%bool controller_torque_discontinuity~%bool start_elbow_sign_inconsistent~%bool communication_constraints_violation~%bool power_limit_violation~%bool joint_p2p_insufficient_torque_for_planning~%bool tau_j_range_violation~%bool instability_detected~%bool joint_move_in_wrong_direction~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Errors>))
  (cl:+ 0
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Errors>))
  "Converts a ROS message object to a list"
  (cl:list 'Errors
    (cl:cons ':joint_position_limits_violation (joint_position_limits_violation msg))
    (cl:cons ':cartesian_position_limits_violation (cartesian_position_limits_violation msg))
    (cl:cons ':self_collision_avoidance_violation (self_collision_avoidance_violation msg))
    (cl:cons ':joint_velocity_violation (joint_velocity_violation msg))
    (cl:cons ':cartesian_velocity_violation (cartesian_velocity_violation msg))
    (cl:cons ':force_control_safety_violation (force_control_safety_violation msg))
    (cl:cons ':joint_reflex (joint_reflex msg))
    (cl:cons ':cartesian_reflex (cartesian_reflex msg))
    (cl:cons ':max_goal_pose_deviation_violation (max_goal_pose_deviation_violation msg))
    (cl:cons ':max_path_pose_deviation_violation (max_path_pose_deviation_violation msg))
    (cl:cons ':cartesian_velocity_profile_safety_violation (cartesian_velocity_profile_safety_violation msg))
    (cl:cons ':joint_position_motion_generator_start_pose_invalid (joint_position_motion_generator_start_pose_invalid msg))
    (cl:cons ':joint_motion_generator_position_limits_violation (joint_motion_generator_position_limits_violation msg))
    (cl:cons ':joint_motion_generator_velocity_limits_violation (joint_motion_generator_velocity_limits_violation msg))
    (cl:cons ':joint_motion_generator_velocity_discontinuity (joint_motion_generator_velocity_discontinuity msg))
    (cl:cons ':joint_motion_generator_acceleration_discontinuity (joint_motion_generator_acceleration_discontinuity msg))
    (cl:cons ':cartesian_position_motion_generator_start_pose_invalid (cartesian_position_motion_generator_start_pose_invalid msg))
    (cl:cons ':cartesian_motion_generator_elbow_limit_violation (cartesian_motion_generator_elbow_limit_violation msg))
    (cl:cons ':cartesian_motion_generator_velocity_limits_violation (cartesian_motion_generator_velocity_limits_violation msg))
    (cl:cons ':cartesian_motion_generator_velocity_discontinuity (cartesian_motion_generator_velocity_discontinuity msg))
    (cl:cons ':cartesian_motion_generator_acceleration_discontinuity (cartesian_motion_generator_acceleration_discontinuity msg))
    (cl:cons ':cartesian_motion_generator_elbow_sign_inconsistent (cartesian_motion_generator_elbow_sign_inconsistent msg))
    (cl:cons ':cartesian_motion_generator_start_elbow_invalid (cartesian_motion_generator_start_elbow_invalid msg))
    (cl:cons ':cartesian_motion_generator_joint_position_limits_violation (cartesian_motion_generator_joint_position_limits_violation msg))
    (cl:cons ':cartesian_motion_generator_joint_velocity_limits_violation (cartesian_motion_generator_joint_velocity_limits_violation msg))
    (cl:cons ':cartesian_motion_generator_joint_velocity_discontinuity (cartesian_motion_generator_joint_velocity_discontinuity msg))
    (cl:cons ':cartesian_motion_generator_joint_acceleration_discontinuity (cartesian_motion_generator_joint_acceleration_discontinuity msg))
    (cl:cons ':cartesian_position_motion_generator_invalid_frame (cartesian_position_motion_generator_invalid_frame msg))
    (cl:cons ':force_controller_desired_force_tolerance_violation (force_controller_desired_force_tolerance_violation msg))
    (cl:cons ':controller_torque_discontinuity (controller_torque_discontinuity msg))
    (cl:cons ':start_elbow_sign_inconsistent (start_elbow_sign_inconsistent msg))
    (cl:cons ':communication_constraints_violation (communication_constraints_violation msg))
    (cl:cons ':power_limit_violation (power_limit_violation msg))
    (cl:cons ':joint_p2p_insufficient_torque_for_planning (joint_p2p_insufficient_torque_for_planning msg))
    (cl:cons ':tau_j_range_violation (tau_j_range_violation msg))
    (cl:cons ':instability_detected (instability_detected msg))
    (cl:cons ':joint_move_in_wrong_direction (joint_move_in_wrong_direction msg))
))

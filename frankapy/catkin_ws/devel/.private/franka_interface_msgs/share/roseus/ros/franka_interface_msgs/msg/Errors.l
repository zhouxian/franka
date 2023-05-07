;; Auto-generated. Do not edit!


(when (boundp 'franka_interface_msgs::Errors)
  (if (not (find-package "FRANKA_INTERFACE_MSGS"))
    (make-package "FRANKA_INTERFACE_MSGS"))
  (shadow 'Errors (find-package "FRANKA_INTERFACE_MSGS")))
(unless (find-package "FRANKA_INTERFACE_MSGS::ERRORS")
  (make-package "FRANKA_INTERFACE_MSGS::ERRORS"))

(in-package "ROS")
;;//! \htmlinclude Errors.msg.html


(defclass franka_interface_msgs::Errors
  :super ros::object
  :slots (_joint_position_limits_violation _cartesian_position_limits_violation _self_collision_avoidance_violation _joint_velocity_violation _cartesian_velocity_violation _force_control_safety_violation _joint_reflex _cartesian_reflex _max_goal_pose_deviation_violation _max_path_pose_deviation_violation _cartesian_velocity_profile_safety_violation _joint_position_motion_generator_start_pose_invalid _joint_motion_generator_position_limits_violation _joint_motion_generator_velocity_limits_violation _joint_motion_generator_velocity_discontinuity _joint_motion_generator_acceleration_discontinuity _cartesian_position_motion_generator_start_pose_invalid _cartesian_motion_generator_elbow_limit_violation _cartesian_motion_generator_velocity_limits_violation _cartesian_motion_generator_velocity_discontinuity _cartesian_motion_generator_acceleration_discontinuity _cartesian_motion_generator_elbow_sign_inconsistent _cartesian_motion_generator_start_elbow_invalid _cartesian_motion_generator_joint_position_limits_violation _cartesian_motion_generator_joint_velocity_limits_violation _cartesian_motion_generator_joint_velocity_discontinuity _cartesian_motion_generator_joint_acceleration_discontinuity _cartesian_position_motion_generator_invalid_frame _force_controller_desired_force_tolerance_violation _controller_torque_discontinuity _start_elbow_sign_inconsistent _communication_constraints_violation _power_limit_violation _joint_p2p_insufficient_torque_for_planning _tau_j_range_violation _instability_detected _joint_move_in_wrong_direction ))

(defmethod franka_interface_msgs::Errors
  (:init
   (&key
    ((:joint_position_limits_violation __joint_position_limits_violation) nil)
    ((:cartesian_position_limits_violation __cartesian_position_limits_violation) nil)
    ((:self_collision_avoidance_violation __self_collision_avoidance_violation) nil)
    ((:joint_velocity_violation __joint_velocity_violation) nil)
    ((:cartesian_velocity_violation __cartesian_velocity_violation) nil)
    ((:force_control_safety_violation __force_control_safety_violation) nil)
    ((:joint_reflex __joint_reflex) nil)
    ((:cartesian_reflex __cartesian_reflex) nil)
    ((:max_goal_pose_deviation_violation __max_goal_pose_deviation_violation) nil)
    ((:max_path_pose_deviation_violation __max_path_pose_deviation_violation) nil)
    ((:cartesian_velocity_profile_safety_violation __cartesian_velocity_profile_safety_violation) nil)
    ((:joint_position_motion_generator_start_pose_invalid __joint_position_motion_generator_start_pose_invalid) nil)
    ((:joint_motion_generator_position_limits_violation __joint_motion_generator_position_limits_violation) nil)
    ((:joint_motion_generator_velocity_limits_violation __joint_motion_generator_velocity_limits_violation) nil)
    ((:joint_motion_generator_velocity_discontinuity __joint_motion_generator_velocity_discontinuity) nil)
    ((:joint_motion_generator_acceleration_discontinuity __joint_motion_generator_acceleration_discontinuity) nil)
    ((:cartesian_position_motion_generator_start_pose_invalid __cartesian_position_motion_generator_start_pose_invalid) nil)
    ((:cartesian_motion_generator_elbow_limit_violation __cartesian_motion_generator_elbow_limit_violation) nil)
    ((:cartesian_motion_generator_velocity_limits_violation __cartesian_motion_generator_velocity_limits_violation) nil)
    ((:cartesian_motion_generator_velocity_discontinuity __cartesian_motion_generator_velocity_discontinuity) nil)
    ((:cartesian_motion_generator_acceleration_discontinuity __cartesian_motion_generator_acceleration_discontinuity) nil)
    ((:cartesian_motion_generator_elbow_sign_inconsistent __cartesian_motion_generator_elbow_sign_inconsistent) nil)
    ((:cartesian_motion_generator_start_elbow_invalid __cartesian_motion_generator_start_elbow_invalid) nil)
    ((:cartesian_motion_generator_joint_position_limits_violation __cartesian_motion_generator_joint_position_limits_violation) nil)
    ((:cartesian_motion_generator_joint_velocity_limits_violation __cartesian_motion_generator_joint_velocity_limits_violation) nil)
    ((:cartesian_motion_generator_joint_velocity_discontinuity __cartesian_motion_generator_joint_velocity_discontinuity) nil)
    ((:cartesian_motion_generator_joint_acceleration_discontinuity __cartesian_motion_generator_joint_acceleration_discontinuity) nil)
    ((:cartesian_position_motion_generator_invalid_frame __cartesian_position_motion_generator_invalid_frame) nil)
    ((:force_controller_desired_force_tolerance_violation __force_controller_desired_force_tolerance_violation) nil)
    ((:controller_torque_discontinuity __controller_torque_discontinuity) nil)
    ((:start_elbow_sign_inconsistent __start_elbow_sign_inconsistent) nil)
    ((:communication_constraints_violation __communication_constraints_violation) nil)
    ((:power_limit_violation __power_limit_violation) nil)
    ((:joint_p2p_insufficient_torque_for_planning __joint_p2p_insufficient_torque_for_planning) nil)
    ((:tau_j_range_violation __tau_j_range_violation) nil)
    ((:instability_detected __instability_detected) nil)
    ((:joint_move_in_wrong_direction __joint_move_in_wrong_direction) nil)
    )
   (send-super :init)
   (setq _joint_position_limits_violation __joint_position_limits_violation)
   (setq _cartesian_position_limits_violation __cartesian_position_limits_violation)
   (setq _self_collision_avoidance_violation __self_collision_avoidance_violation)
   (setq _joint_velocity_violation __joint_velocity_violation)
   (setq _cartesian_velocity_violation __cartesian_velocity_violation)
   (setq _force_control_safety_violation __force_control_safety_violation)
   (setq _joint_reflex __joint_reflex)
   (setq _cartesian_reflex __cartesian_reflex)
   (setq _max_goal_pose_deviation_violation __max_goal_pose_deviation_violation)
   (setq _max_path_pose_deviation_violation __max_path_pose_deviation_violation)
   (setq _cartesian_velocity_profile_safety_violation __cartesian_velocity_profile_safety_violation)
   (setq _joint_position_motion_generator_start_pose_invalid __joint_position_motion_generator_start_pose_invalid)
   (setq _joint_motion_generator_position_limits_violation __joint_motion_generator_position_limits_violation)
   (setq _joint_motion_generator_velocity_limits_violation __joint_motion_generator_velocity_limits_violation)
   (setq _joint_motion_generator_velocity_discontinuity __joint_motion_generator_velocity_discontinuity)
   (setq _joint_motion_generator_acceleration_discontinuity __joint_motion_generator_acceleration_discontinuity)
   (setq _cartesian_position_motion_generator_start_pose_invalid __cartesian_position_motion_generator_start_pose_invalid)
   (setq _cartesian_motion_generator_elbow_limit_violation __cartesian_motion_generator_elbow_limit_violation)
   (setq _cartesian_motion_generator_velocity_limits_violation __cartesian_motion_generator_velocity_limits_violation)
   (setq _cartesian_motion_generator_velocity_discontinuity __cartesian_motion_generator_velocity_discontinuity)
   (setq _cartesian_motion_generator_acceleration_discontinuity __cartesian_motion_generator_acceleration_discontinuity)
   (setq _cartesian_motion_generator_elbow_sign_inconsistent __cartesian_motion_generator_elbow_sign_inconsistent)
   (setq _cartesian_motion_generator_start_elbow_invalid __cartesian_motion_generator_start_elbow_invalid)
   (setq _cartesian_motion_generator_joint_position_limits_violation __cartesian_motion_generator_joint_position_limits_violation)
   (setq _cartesian_motion_generator_joint_velocity_limits_violation __cartesian_motion_generator_joint_velocity_limits_violation)
   (setq _cartesian_motion_generator_joint_velocity_discontinuity __cartesian_motion_generator_joint_velocity_discontinuity)
   (setq _cartesian_motion_generator_joint_acceleration_discontinuity __cartesian_motion_generator_joint_acceleration_discontinuity)
   (setq _cartesian_position_motion_generator_invalid_frame __cartesian_position_motion_generator_invalid_frame)
   (setq _force_controller_desired_force_tolerance_violation __force_controller_desired_force_tolerance_violation)
   (setq _controller_torque_discontinuity __controller_torque_discontinuity)
   (setq _start_elbow_sign_inconsistent __start_elbow_sign_inconsistent)
   (setq _communication_constraints_violation __communication_constraints_violation)
   (setq _power_limit_violation __power_limit_violation)
   (setq _joint_p2p_insufficient_torque_for_planning __joint_p2p_insufficient_torque_for_planning)
   (setq _tau_j_range_violation __tau_j_range_violation)
   (setq _instability_detected __instability_detected)
   (setq _joint_move_in_wrong_direction __joint_move_in_wrong_direction)
   self)
  (:joint_position_limits_violation
   (&optional (__joint_position_limits_violation :null))
   (if (not (eq __joint_position_limits_violation :null)) (setq _joint_position_limits_violation __joint_position_limits_violation)) _joint_position_limits_violation)
  (:cartesian_position_limits_violation
   (&optional (__cartesian_position_limits_violation :null))
   (if (not (eq __cartesian_position_limits_violation :null)) (setq _cartesian_position_limits_violation __cartesian_position_limits_violation)) _cartesian_position_limits_violation)
  (:self_collision_avoidance_violation
   (&optional (__self_collision_avoidance_violation :null))
   (if (not (eq __self_collision_avoidance_violation :null)) (setq _self_collision_avoidance_violation __self_collision_avoidance_violation)) _self_collision_avoidance_violation)
  (:joint_velocity_violation
   (&optional (__joint_velocity_violation :null))
   (if (not (eq __joint_velocity_violation :null)) (setq _joint_velocity_violation __joint_velocity_violation)) _joint_velocity_violation)
  (:cartesian_velocity_violation
   (&optional (__cartesian_velocity_violation :null))
   (if (not (eq __cartesian_velocity_violation :null)) (setq _cartesian_velocity_violation __cartesian_velocity_violation)) _cartesian_velocity_violation)
  (:force_control_safety_violation
   (&optional (__force_control_safety_violation :null))
   (if (not (eq __force_control_safety_violation :null)) (setq _force_control_safety_violation __force_control_safety_violation)) _force_control_safety_violation)
  (:joint_reflex
   (&optional (__joint_reflex :null))
   (if (not (eq __joint_reflex :null)) (setq _joint_reflex __joint_reflex)) _joint_reflex)
  (:cartesian_reflex
   (&optional (__cartesian_reflex :null))
   (if (not (eq __cartesian_reflex :null)) (setq _cartesian_reflex __cartesian_reflex)) _cartesian_reflex)
  (:max_goal_pose_deviation_violation
   (&optional (__max_goal_pose_deviation_violation :null))
   (if (not (eq __max_goal_pose_deviation_violation :null)) (setq _max_goal_pose_deviation_violation __max_goal_pose_deviation_violation)) _max_goal_pose_deviation_violation)
  (:max_path_pose_deviation_violation
   (&optional (__max_path_pose_deviation_violation :null))
   (if (not (eq __max_path_pose_deviation_violation :null)) (setq _max_path_pose_deviation_violation __max_path_pose_deviation_violation)) _max_path_pose_deviation_violation)
  (:cartesian_velocity_profile_safety_violation
   (&optional (__cartesian_velocity_profile_safety_violation :null))
   (if (not (eq __cartesian_velocity_profile_safety_violation :null)) (setq _cartesian_velocity_profile_safety_violation __cartesian_velocity_profile_safety_violation)) _cartesian_velocity_profile_safety_violation)
  (:joint_position_motion_generator_start_pose_invalid
   (&optional (__joint_position_motion_generator_start_pose_invalid :null))
   (if (not (eq __joint_position_motion_generator_start_pose_invalid :null)) (setq _joint_position_motion_generator_start_pose_invalid __joint_position_motion_generator_start_pose_invalid)) _joint_position_motion_generator_start_pose_invalid)
  (:joint_motion_generator_position_limits_violation
   (&optional (__joint_motion_generator_position_limits_violation :null))
   (if (not (eq __joint_motion_generator_position_limits_violation :null)) (setq _joint_motion_generator_position_limits_violation __joint_motion_generator_position_limits_violation)) _joint_motion_generator_position_limits_violation)
  (:joint_motion_generator_velocity_limits_violation
   (&optional (__joint_motion_generator_velocity_limits_violation :null))
   (if (not (eq __joint_motion_generator_velocity_limits_violation :null)) (setq _joint_motion_generator_velocity_limits_violation __joint_motion_generator_velocity_limits_violation)) _joint_motion_generator_velocity_limits_violation)
  (:joint_motion_generator_velocity_discontinuity
   (&optional (__joint_motion_generator_velocity_discontinuity :null))
   (if (not (eq __joint_motion_generator_velocity_discontinuity :null)) (setq _joint_motion_generator_velocity_discontinuity __joint_motion_generator_velocity_discontinuity)) _joint_motion_generator_velocity_discontinuity)
  (:joint_motion_generator_acceleration_discontinuity
   (&optional (__joint_motion_generator_acceleration_discontinuity :null))
   (if (not (eq __joint_motion_generator_acceleration_discontinuity :null)) (setq _joint_motion_generator_acceleration_discontinuity __joint_motion_generator_acceleration_discontinuity)) _joint_motion_generator_acceleration_discontinuity)
  (:cartesian_position_motion_generator_start_pose_invalid
   (&optional (__cartesian_position_motion_generator_start_pose_invalid :null))
   (if (not (eq __cartesian_position_motion_generator_start_pose_invalid :null)) (setq _cartesian_position_motion_generator_start_pose_invalid __cartesian_position_motion_generator_start_pose_invalid)) _cartesian_position_motion_generator_start_pose_invalid)
  (:cartesian_motion_generator_elbow_limit_violation
   (&optional (__cartesian_motion_generator_elbow_limit_violation :null))
   (if (not (eq __cartesian_motion_generator_elbow_limit_violation :null)) (setq _cartesian_motion_generator_elbow_limit_violation __cartesian_motion_generator_elbow_limit_violation)) _cartesian_motion_generator_elbow_limit_violation)
  (:cartesian_motion_generator_velocity_limits_violation
   (&optional (__cartesian_motion_generator_velocity_limits_violation :null))
   (if (not (eq __cartesian_motion_generator_velocity_limits_violation :null)) (setq _cartesian_motion_generator_velocity_limits_violation __cartesian_motion_generator_velocity_limits_violation)) _cartesian_motion_generator_velocity_limits_violation)
  (:cartesian_motion_generator_velocity_discontinuity
   (&optional (__cartesian_motion_generator_velocity_discontinuity :null))
   (if (not (eq __cartesian_motion_generator_velocity_discontinuity :null)) (setq _cartesian_motion_generator_velocity_discontinuity __cartesian_motion_generator_velocity_discontinuity)) _cartesian_motion_generator_velocity_discontinuity)
  (:cartesian_motion_generator_acceleration_discontinuity
   (&optional (__cartesian_motion_generator_acceleration_discontinuity :null))
   (if (not (eq __cartesian_motion_generator_acceleration_discontinuity :null)) (setq _cartesian_motion_generator_acceleration_discontinuity __cartesian_motion_generator_acceleration_discontinuity)) _cartesian_motion_generator_acceleration_discontinuity)
  (:cartesian_motion_generator_elbow_sign_inconsistent
   (&optional (__cartesian_motion_generator_elbow_sign_inconsistent :null))
   (if (not (eq __cartesian_motion_generator_elbow_sign_inconsistent :null)) (setq _cartesian_motion_generator_elbow_sign_inconsistent __cartesian_motion_generator_elbow_sign_inconsistent)) _cartesian_motion_generator_elbow_sign_inconsistent)
  (:cartesian_motion_generator_start_elbow_invalid
   (&optional (__cartesian_motion_generator_start_elbow_invalid :null))
   (if (not (eq __cartesian_motion_generator_start_elbow_invalid :null)) (setq _cartesian_motion_generator_start_elbow_invalid __cartesian_motion_generator_start_elbow_invalid)) _cartesian_motion_generator_start_elbow_invalid)
  (:cartesian_motion_generator_joint_position_limits_violation
   (&optional (__cartesian_motion_generator_joint_position_limits_violation :null))
   (if (not (eq __cartesian_motion_generator_joint_position_limits_violation :null)) (setq _cartesian_motion_generator_joint_position_limits_violation __cartesian_motion_generator_joint_position_limits_violation)) _cartesian_motion_generator_joint_position_limits_violation)
  (:cartesian_motion_generator_joint_velocity_limits_violation
   (&optional (__cartesian_motion_generator_joint_velocity_limits_violation :null))
   (if (not (eq __cartesian_motion_generator_joint_velocity_limits_violation :null)) (setq _cartesian_motion_generator_joint_velocity_limits_violation __cartesian_motion_generator_joint_velocity_limits_violation)) _cartesian_motion_generator_joint_velocity_limits_violation)
  (:cartesian_motion_generator_joint_velocity_discontinuity
   (&optional (__cartesian_motion_generator_joint_velocity_discontinuity :null))
   (if (not (eq __cartesian_motion_generator_joint_velocity_discontinuity :null)) (setq _cartesian_motion_generator_joint_velocity_discontinuity __cartesian_motion_generator_joint_velocity_discontinuity)) _cartesian_motion_generator_joint_velocity_discontinuity)
  (:cartesian_motion_generator_joint_acceleration_discontinuity
   (&optional (__cartesian_motion_generator_joint_acceleration_discontinuity :null))
   (if (not (eq __cartesian_motion_generator_joint_acceleration_discontinuity :null)) (setq _cartesian_motion_generator_joint_acceleration_discontinuity __cartesian_motion_generator_joint_acceleration_discontinuity)) _cartesian_motion_generator_joint_acceleration_discontinuity)
  (:cartesian_position_motion_generator_invalid_frame
   (&optional (__cartesian_position_motion_generator_invalid_frame :null))
   (if (not (eq __cartesian_position_motion_generator_invalid_frame :null)) (setq _cartesian_position_motion_generator_invalid_frame __cartesian_position_motion_generator_invalid_frame)) _cartesian_position_motion_generator_invalid_frame)
  (:force_controller_desired_force_tolerance_violation
   (&optional (__force_controller_desired_force_tolerance_violation :null))
   (if (not (eq __force_controller_desired_force_tolerance_violation :null)) (setq _force_controller_desired_force_tolerance_violation __force_controller_desired_force_tolerance_violation)) _force_controller_desired_force_tolerance_violation)
  (:controller_torque_discontinuity
   (&optional (__controller_torque_discontinuity :null))
   (if (not (eq __controller_torque_discontinuity :null)) (setq _controller_torque_discontinuity __controller_torque_discontinuity)) _controller_torque_discontinuity)
  (:start_elbow_sign_inconsistent
   (&optional (__start_elbow_sign_inconsistent :null))
   (if (not (eq __start_elbow_sign_inconsistent :null)) (setq _start_elbow_sign_inconsistent __start_elbow_sign_inconsistent)) _start_elbow_sign_inconsistent)
  (:communication_constraints_violation
   (&optional (__communication_constraints_violation :null))
   (if (not (eq __communication_constraints_violation :null)) (setq _communication_constraints_violation __communication_constraints_violation)) _communication_constraints_violation)
  (:power_limit_violation
   (&optional (__power_limit_violation :null))
   (if (not (eq __power_limit_violation :null)) (setq _power_limit_violation __power_limit_violation)) _power_limit_violation)
  (:joint_p2p_insufficient_torque_for_planning
   (&optional (__joint_p2p_insufficient_torque_for_planning :null))
   (if (not (eq __joint_p2p_insufficient_torque_for_planning :null)) (setq _joint_p2p_insufficient_torque_for_planning __joint_p2p_insufficient_torque_for_planning)) _joint_p2p_insufficient_torque_for_planning)
  (:tau_j_range_violation
   (&optional (__tau_j_range_violation :null))
   (if (not (eq __tau_j_range_violation :null)) (setq _tau_j_range_violation __tau_j_range_violation)) _tau_j_range_violation)
  (:instability_detected
   (&optional (__instability_detected :null))
   (if (not (eq __instability_detected :null)) (setq _instability_detected __instability_detected)) _instability_detected)
  (:joint_move_in_wrong_direction
   (&optional (__joint_move_in_wrong_direction :null))
   (if (not (eq __joint_move_in_wrong_direction :null)) (setq _joint_move_in_wrong_direction __joint_move_in_wrong_direction)) _joint_move_in_wrong_direction)
  (:serialization-length
   ()
   (+
    ;; bool _joint_position_limits_violation
    1
    ;; bool _cartesian_position_limits_violation
    1
    ;; bool _self_collision_avoidance_violation
    1
    ;; bool _joint_velocity_violation
    1
    ;; bool _cartesian_velocity_violation
    1
    ;; bool _force_control_safety_violation
    1
    ;; bool _joint_reflex
    1
    ;; bool _cartesian_reflex
    1
    ;; bool _max_goal_pose_deviation_violation
    1
    ;; bool _max_path_pose_deviation_violation
    1
    ;; bool _cartesian_velocity_profile_safety_violation
    1
    ;; bool _joint_position_motion_generator_start_pose_invalid
    1
    ;; bool _joint_motion_generator_position_limits_violation
    1
    ;; bool _joint_motion_generator_velocity_limits_violation
    1
    ;; bool _joint_motion_generator_velocity_discontinuity
    1
    ;; bool _joint_motion_generator_acceleration_discontinuity
    1
    ;; bool _cartesian_position_motion_generator_start_pose_invalid
    1
    ;; bool _cartesian_motion_generator_elbow_limit_violation
    1
    ;; bool _cartesian_motion_generator_velocity_limits_violation
    1
    ;; bool _cartesian_motion_generator_velocity_discontinuity
    1
    ;; bool _cartesian_motion_generator_acceleration_discontinuity
    1
    ;; bool _cartesian_motion_generator_elbow_sign_inconsistent
    1
    ;; bool _cartesian_motion_generator_start_elbow_invalid
    1
    ;; bool _cartesian_motion_generator_joint_position_limits_violation
    1
    ;; bool _cartesian_motion_generator_joint_velocity_limits_violation
    1
    ;; bool _cartesian_motion_generator_joint_velocity_discontinuity
    1
    ;; bool _cartesian_motion_generator_joint_acceleration_discontinuity
    1
    ;; bool _cartesian_position_motion_generator_invalid_frame
    1
    ;; bool _force_controller_desired_force_tolerance_violation
    1
    ;; bool _controller_torque_discontinuity
    1
    ;; bool _start_elbow_sign_inconsistent
    1
    ;; bool _communication_constraints_violation
    1
    ;; bool _power_limit_violation
    1
    ;; bool _joint_p2p_insufficient_torque_for_planning
    1
    ;; bool _tau_j_range_violation
    1
    ;; bool _instability_detected
    1
    ;; bool _joint_move_in_wrong_direction
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _joint_position_limits_violation
       (if _joint_position_limits_violation (write-byte -1 s) (write-byte 0 s))
     ;; bool _cartesian_position_limits_violation
       (if _cartesian_position_limits_violation (write-byte -1 s) (write-byte 0 s))
     ;; bool _self_collision_avoidance_violation
       (if _self_collision_avoidance_violation (write-byte -1 s) (write-byte 0 s))
     ;; bool _joint_velocity_violation
       (if _joint_velocity_violation (write-byte -1 s) (write-byte 0 s))
     ;; bool _cartesian_velocity_violation
       (if _cartesian_velocity_violation (write-byte -1 s) (write-byte 0 s))
     ;; bool _force_control_safety_violation
       (if _force_control_safety_violation (write-byte -1 s) (write-byte 0 s))
     ;; bool _joint_reflex
       (if _joint_reflex (write-byte -1 s) (write-byte 0 s))
     ;; bool _cartesian_reflex
       (if _cartesian_reflex (write-byte -1 s) (write-byte 0 s))
     ;; bool _max_goal_pose_deviation_violation
       (if _max_goal_pose_deviation_violation (write-byte -1 s) (write-byte 0 s))
     ;; bool _max_path_pose_deviation_violation
       (if _max_path_pose_deviation_violation (write-byte -1 s) (write-byte 0 s))
     ;; bool _cartesian_velocity_profile_safety_violation
       (if _cartesian_velocity_profile_safety_violation (write-byte -1 s) (write-byte 0 s))
     ;; bool _joint_position_motion_generator_start_pose_invalid
       (if _joint_position_motion_generator_start_pose_invalid (write-byte -1 s) (write-byte 0 s))
     ;; bool _joint_motion_generator_position_limits_violation
       (if _joint_motion_generator_position_limits_violation (write-byte -1 s) (write-byte 0 s))
     ;; bool _joint_motion_generator_velocity_limits_violation
       (if _joint_motion_generator_velocity_limits_violation (write-byte -1 s) (write-byte 0 s))
     ;; bool _joint_motion_generator_velocity_discontinuity
       (if _joint_motion_generator_velocity_discontinuity (write-byte -1 s) (write-byte 0 s))
     ;; bool _joint_motion_generator_acceleration_discontinuity
       (if _joint_motion_generator_acceleration_discontinuity (write-byte -1 s) (write-byte 0 s))
     ;; bool _cartesian_position_motion_generator_start_pose_invalid
       (if _cartesian_position_motion_generator_start_pose_invalid (write-byte -1 s) (write-byte 0 s))
     ;; bool _cartesian_motion_generator_elbow_limit_violation
       (if _cartesian_motion_generator_elbow_limit_violation (write-byte -1 s) (write-byte 0 s))
     ;; bool _cartesian_motion_generator_velocity_limits_violation
       (if _cartesian_motion_generator_velocity_limits_violation (write-byte -1 s) (write-byte 0 s))
     ;; bool _cartesian_motion_generator_velocity_discontinuity
       (if _cartesian_motion_generator_velocity_discontinuity (write-byte -1 s) (write-byte 0 s))
     ;; bool _cartesian_motion_generator_acceleration_discontinuity
       (if _cartesian_motion_generator_acceleration_discontinuity (write-byte -1 s) (write-byte 0 s))
     ;; bool _cartesian_motion_generator_elbow_sign_inconsistent
       (if _cartesian_motion_generator_elbow_sign_inconsistent (write-byte -1 s) (write-byte 0 s))
     ;; bool _cartesian_motion_generator_start_elbow_invalid
       (if _cartesian_motion_generator_start_elbow_invalid (write-byte -1 s) (write-byte 0 s))
     ;; bool _cartesian_motion_generator_joint_position_limits_violation
       (if _cartesian_motion_generator_joint_position_limits_violation (write-byte -1 s) (write-byte 0 s))
     ;; bool _cartesian_motion_generator_joint_velocity_limits_violation
       (if _cartesian_motion_generator_joint_velocity_limits_violation (write-byte -1 s) (write-byte 0 s))
     ;; bool _cartesian_motion_generator_joint_velocity_discontinuity
       (if _cartesian_motion_generator_joint_velocity_discontinuity (write-byte -1 s) (write-byte 0 s))
     ;; bool _cartesian_motion_generator_joint_acceleration_discontinuity
       (if _cartesian_motion_generator_joint_acceleration_discontinuity (write-byte -1 s) (write-byte 0 s))
     ;; bool _cartesian_position_motion_generator_invalid_frame
       (if _cartesian_position_motion_generator_invalid_frame (write-byte -1 s) (write-byte 0 s))
     ;; bool _force_controller_desired_force_tolerance_violation
       (if _force_controller_desired_force_tolerance_violation (write-byte -1 s) (write-byte 0 s))
     ;; bool _controller_torque_discontinuity
       (if _controller_torque_discontinuity (write-byte -1 s) (write-byte 0 s))
     ;; bool _start_elbow_sign_inconsistent
       (if _start_elbow_sign_inconsistent (write-byte -1 s) (write-byte 0 s))
     ;; bool _communication_constraints_violation
       (if _communication_constraints_violation (write-byte -1 s) (write-byte 0 s))
     ;; bool _power_limit_violation
       (if _power_limit_violation (write-byte -1 s) (write-byte 0 s))
     ;; bool _joint_p2p_insufficient_torque_for_planning
       (if _joint_p2p_insufficient_torque_for_planning (write-byte -1 s) (write-byte 0 s))
     ;; bool _tau_j_range_violation
       (if _tau_j_range_violation (write-byte -1 s) (write-byte 0 s))
     ;; bool _instability_detected
       (if _instability_detected (write-byte -1 s) (write-byte 0 s))
     ;; bool _joint_move_in_wrong_direction
       (if _joint_move_in_wrong_direction (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _joint_position_limits_violation
     (setq _joint_position_limits_violation (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _cartesian_position_limits_violation
     (setq _cartesian_position_limits_violation (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _self_collision_avoidance_violation
     (setq _self_collision_avoidance_violation (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _joint_velocity_violation
     (setq _joint_velocity_violation (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _cartesian_velocity_violation
     (setq _cartesian_velocity_violation (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _force_control_safety_violation
     (setq _force_control_safety_violation (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _joint_reflex
     (setq _joint_reflex (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _cartesian_reflex
     (setq _cartesian_reflex (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _max_goal_pose_deviation_violation
     (setq _max_goal_pose_deviation_violation (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _max_path_pose_deviation_violation
     (setq _max_path_pose_deviation_violation (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _cartesian_velocity_profile_safety_violation
     (setq _cartesian_velocity_profile_safety_violation (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _joint_position_motion_generator_start_pose_invalid
     (setq _joint_position_motion_generator_start_pose_invalid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _joint_motion_generator_position_limits_violation
     (setq _joint_motion_generator_position_limits_violation (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _joint_motion_generator_velocity_limits_violation
     (setq _joint_motion_generator_velocity_limits_violation (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _joint_motion_generator_velocity_discontinuity
     (setq _joint_motion_generator_velocity_discontinuity (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _joint_motion_generator_acceleration_discontinuity
     (setq _joint_motion_generator_acceleration_discontinuity (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _cartesian_position_motion_generator_start_pose_invalid
     (setq _cartesian_position_motion_generator_start_pose_invalid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _cartesian_motion_generator_elbow_limit_violation
     (setq _cartesian_motion_generator_elbow_limit_violation (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _cartesian_motion_generator_velocity_limits_violation
     (setq _cartesian_motion_generator_velocity_limits_violation (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _cartesian_motion_generator_velocity_discontinuity
     (setq _cartesian_motion_generator_velocity_discontinuity (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _cartesian_motion_generator_acceleration_discontinuity
     (setq _cartesian_motion_generator_acceleration_discontinuity (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _cartesian_motion_generator_elbow_sign_inconsistent
     (setq _cartesian_motion_generator_elbow_sign_inconsistent (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _cartesian_motion_generator_start_elbow_invalid
     (setq _cartesian_motion_generator_start_elbow_invalid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _cartesian_motion_generator_joint_position_limits_violation
     (setq _cartesian_motion_generator_joint_position_limits_violation (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _cartesian_motion_generator_joint_velocity_limits_violation
     (setq _cartesian_motion_generator_joint_velocity_limits_violation (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _cartesian_motion_generator_joint_velocity_discontinuity
     (setq _cartesian_motion_generator_joint_velocity_discontinuity (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _cartesian_motion_generator_joint_acceleration_discontinuity
     (setq _cartesian_motion_generator_joint_acceleration_discontinuity (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _cartesian_position_motion_generator_invalid_frame
     (setq _cartesian_position_motion_generator_invalid_frame (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _force_controller_desired_force_tolerance_violation
     (setq _force_controller_desired_force_tolerance_violation (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _controller_torque_discontinuity
     (setq _controller_torque_discontinuity (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _start_elbow_sign_inconsistent
     (setq _start_elbow_sign_inconsistent (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _communication_constraints_violation
     (setq _communication_constraints_violation (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _power_limit_violation
     (setq _power_limit_violation (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _joint_p2p_insufficient_torque_for_planning
     (setq _joint_p2p_insufficient_torque_for_planning (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _tau_j_range_violation
     (setq _tau_j_range_violation (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _instability_detected
     (setq _instability_detected (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _joint_move_in_wrong_direction
     (setq _joint_move_in_wrong_direction (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get franka_interface_msgs::Errors :md5sum-) "b93c14a039f7f69d7321c619961e4708")
(setf (get franka_interface_msgs::Errors :datatype-) "franka_interface_msgs/Errors")
(setf (get franka_interface_msgs::Errors :definition-)
      "bool joint_position_limits_violation
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
bool joint_move_in_wrong_direction
")



(provide :franka_interface_msgs/Errors "b93c14a039f7f69d7321c619961e4708")



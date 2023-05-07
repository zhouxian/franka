;; Auto-generated. Do not edit!


(when (boundp 'franka_interface_msgs::RobotState)
  (if (not (find-package "FRANKA_INTERFACE_MSGS"))
    (make-package "FRANKA_INTERFACE_MSGS"))
  (shadow 'RobotState (find-package "FRANKA_INTERFACE_MSGS")))
(unless (find-package "FRANKA_INTERFACE_MSGS::ROBOTSTATE")
  (make-package "FRANKA_INTERFACE_MSGS::ROBOTSTATE"))

(in-package "ROS")
;;//! \htmlinclude RobotState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*ROBOT_MODE_OTHER*" (find-package "FRANKA_INTERFACE_MSGS::ROBOTSTATE"))
(shadow '*ROBOT_MODE_OTHER* (find-package "FRANKA_INTERFACE_MSGS::ROBOTSTATE"))
(defconstant franka_interface_msgs::RobotState::*ROBOT_MODE_OTHER* 0)
(intern "*ROBOT_MODE_IDLE*" (find-package "FRANKA_INTERFACE_MSGS::ROBOTSTATE"))
(shadow '*ROBOT_MODE_IDLE* (find-package "FRANKA_INTERFACE_MSGS::ROBOTSTATE"))
(defconstant franka_interface_msgs::RobotState::*ROBOT_MODE_IDLE* 1)
(intern "*ROBOT_MODE_MOVE*" (find-package "FRANKA_INTERFACE_MSGS::ROBOTSTATE"))
(shadow '*ROBOT_MODE_MOVE* (find-package "FRANKA_INTERFACE_MSGS::ROBOTSTATE"))
(defconstant franka_interface_msgs::RobotState::*ROBOT_MODE_MOVE* 2)
(intern "*ROBOT_MODE_GUIDING*" (find-package "FRANKA_INTERFACE_MSGS::ROBOTSTATE"))
(shadow '*ROBOT_MODE_GUIDING* (find-package "FRANKA_INTERFACE_MSGS::ROBOTSTATE"))
(defconstant franka_interface_msgs::RobotState::*ROBOT_MODE_GUIDING* 3)
(intern "*ROBOT_MODE_REFLEX*" (find-package "FRANKA_INTERFACE_MSGS::ROBOTSTATE"))
(shadow '*ROBOT_MODE_REFLEX* (find-package "FRANKA_INTERFACE_MSGS::ROBOTSTATE"))
(defconstant franka_interface_msgs::RobotState::*ROBOT_MODE_REFLEX* 4)
(intern "*ROBOT_MODE_USER_STOPPED*" (find-package "FRANKA_INTERFACE_MSGS::ROBOTSTATE"))
(shadow '*ROBOT_MODE_USER_STOPPED* (find-package "FRANKA_INTERFACE_MSGS::ROBOTSTATE"))
(defconstant franka_interface_msgs::RobotState::*ROBOT_MODE_USER_STOPPED* 5)
(intern "*ROBOT_MODE_AUTOMATIC_ERROR_RECOVERY*" (find-package "FRANKA_INTERFACE_MSGS::ROBOTSTATE"))
(shadow '*ROBOT_MODE_AUTOMATIC_ERROR_RECOVERY* (find-package "FRANKA_INTERFACE_MSGS::ROBOTSTATE"))
(defconstant franka_interface_msgs::RobotState::*ROBOT_MODE_AUTOMATIC_ERROR_RECOVERY* 6)

(defun franka_interface_msgs::RobotState-to-symbol (const)
  (cond
        ((= const 0) 'franka_interface_msgs::RobotState::*ROBOT_MODE_OTHER*)
        ((= const 1) 'franka_interface_msgs::RobotState::*ROBOT_MODE_IDLE*)
        ((= const 2) 'franka_interface_msgs::RobotState::*ROBOT_MODE_MOVE*)
        ((= const 3) 'franka_interface_msgs::RobotState::*ROBOT_MODE_GUIDING*)
        ((= const 4) 'franka_interface_msgs::RobotState::*ROBOT_MODE_REFLEX*)
        ((= const 5) 'franka_interface_msgs::RobotState::*ROBOT_MODE_USER_STOPPED*)
        ((= const 6) 'franka_interface_msgs::RobotState::*ROBOT_MODE_AUTOMATIC_ERROR_RECOVERY*)
        (t nil)))

(defclass franka_interface_msgs::RobotState
  :super ros::object
  :slots (_header _pose_desired _O_T_EE _O_T_EE_d _F_T_EE _EE_T_K _m_ee _I_ee _F_x_Cee _m_load _I_load _F_x_Cload _m_total _I_total _F_x_Ctotal _elbow _elbow_d _elbow_c _delbow_c _ddelbow_c _tau_J _tau_J_d _dtau_J _q _q_d _dq _dq_d _ddq_d _joint_contact _cartesian_contact _joint_collision _cartesian_collision _tau_ext_hat_filtered _O_F_ext_hat_K _K_F_ext_hat_K _O_dP_EE_d _O_T_EE_c _O_dP_EE_c _O_ddP_EE_c _theta _dtheta _current_errors _last_motion_errors _control_command_success_rate _robot_mode _robot_time _gripper_width _gripper_max_width _gripper_is_grasped _gripper_temperature _gripper_time _is_fresh ))

(defmethod franka_interface_msgs::RobotState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:pose_desired __pose_desired) (make-array 16 :initial-element 0.0 :element-type :float))
    ((:O_T_EE __O_T_EE) (make-array 16 :initial-element 0.0 :element-type :float))
    ((:O_T_EE_d __O_T_EE_d) (make-array 16 :initial-element 0.0 :element-type :float))
    ((:F_T_EE __F_T_EE) (make-array 16 :initial-element 0.0 :element-type :float))
    ((:EE_T_K __EE_T_K) (make-array 16 :initial-element 0.0 :element-type :float))
    ((:m_ee __m_ee) 0.0)
    ((:I_ee __I_ee) (make-array 9 :initial-element 0.0 :element-type :float))
    ((:F_x_Cee __F_x_Cee) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:m_load __m_load) 0.0)
    ((:I_load __I_load) (make-array 9 :initial-element 0.0 :element-type :float))
    ((:F_x_Cload __F_x_Cload) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:m_total __m_total) 0.0)
    ((:I_total __I_total) (make-array 9 :initial-element 0.0 :element-type :float))
    ((:F_x_Ctotal __F_x_Ctotal) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:elbow __elbow) (make-array 2 :initial-element 0.0 :element-type :float))
    ((:elbow_d __elbow_d) (make-array 2 :initial-element 0.0 :element-type :float))
    ((:elbow_c __elbow_c) (make-array 2 :initial-element 0.0 :element-type :float))
    ((:delbow_c __delbow_c) (make-array 2 :initial-element 0.0 :element-type :float))
    ((:ddelbow_c __ddelbow_c) (make-array 2 :initial-element 0.0 :element-type :float))
    ((:tau_J __tau_J) (make-array 7 :initial-element 0.0 :element-type :float))
    ((:tau_J_d __tau_J_d) (make-array 7 :initial-element 0.0 :element-type :float))
    ((:dtau_J __dtau_J) (make-array 7 :initial-element 0.0 :element-type :float))
    ((:q __q) (make-array 7 :initial-element 0.0 :element-type :float))
    ((:q_d __q_d) (make-array 7 :initial-element 0.0 :element-type :float))
    ((:dq __dq) (make-array 7 :initial-element 0.0 :element-type :float))
    ((:dq_d __dq_d) (make-array 7 :initial-element 0.0 :element-type :float))
    ((:ddq_d __ddq_d) (make-array 7 :initial-element 0.0 :element-type :float))
    ((:joint_contact __joint_contact) (make-array 7 :initial-element 0.0 :element-type :float))
    ((:cartesian_contact __cartesian_contact) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:joint_collision __joint_collision) (make-array 7 :initial-element 0.0 :element-type :float))
    ((:cartesian_collision __cartesian_collision) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:tau_ext_hat_filtered __tau_ext_hat_filtered) (make-array 7 :initial-element 0.0 :element-type :float))
    ((:O_F_ext_hat_K __O_F_ext_hat_K) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:K_F_ext_hat_K __K_F_ext_hat_K) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:O_dP_EE_d __O_dP_EE_d) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:O_T_EE_c __O_T_EE_c) (make-array 16 :initial-element 0.0 :element-type :float))
    ((:O_dP_EE_c __O_dP_EE_c) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:O_ddP_EE_c __O_ddP_EE_c) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:theta __theta) (make-array 7 :initial-element 0.0 :element-type :float))
    ((:dtheta __dtheta) (make-array 7 :initial-element 0.0 :element-type :float))
    ((:current_errors __current_errors) (instance franka_interface_msgs::Errors :init))
    ((:last_motion_errors __last_motion_errors) (instance franka_interface_msgs::Errors :init))
    ((:control_command_success_rate __control_command_success_rate) 0.0)
    ((:robot_mode __robot_mode) 0)
    ((:robot_time __robot_time) 0.0)
    ((:gripper_width __gripper_width) 0.0)
    ((:gripper_max_width __gripper_max_width) 0.0)
    ((:gripper_is_grasped __gripper_is_grasped) nil)
    ((:gripper_temperature __gripper_temperature) 0)
    ((:gripper_time __gripper_time) 0.0)
    ((:is_fresh __is_fresh) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _pose_desired __pose_desired)
   (setq _O_T_EE __O_T_EE)
   (setq _O_T_EE_d __O_T_EE_d)
   (setq _F_T_EE __F_T_EE)
   (setq _EE_T_K __EE_T_K)
   (setq _m_ee (float __m_ee))
   (setq _I_ee __I_ee)
   (setq _F_x_Cee __F_x_Cee)
   (setq _m_load (float __m_load))
   (setq _I_load __I_load)
   (setq _F_x_Cload __F_x_Cload)
   (setq _m_total (float __m_total))
   (setq _I_total __I_total)
   (setq _F_x_Ctotal __F_x_Ctotal)
   (setq _elbow __elbow)
   (setq _elbow_d __elbow_d)
   (setq _elbow_c __elbow_c)
   (setq _delbow_c __delbow_c)
   (setq _ddelbow_c __ddelbow_c)
   (setq _tau_J __tau_J)
   (setq _tau_J_d __tau_J_d)
   (setq _dtau_J __dtau_J)
   (setq _q __q)
   (setq _q_d __q_d)
   (setq _dq __dq)
   (setq _dq_d __dq_d)
   (setq _ddq_d __ddq_d)
   (setq _joint_contact __joint_contact)
   (setq _cartesian_contact __cartesian_contact)
   (setq _joint_collision __joint_collision)
   (setq _cartesian_collision __cartesian_collision)
   (setq _tau_ext_hat_filtered __tau_ext_hat_filtered)
   (setq _O_F_ext_hat_K __O_F_ext_hat_K)
   (setq _K_F_ext_hat_K __K_F_ext_hat_K)
   (setq _O_dP_EE_d __O_dP_EE_d)
   (setq _O_T_EE_c __O_T_EE_c)
   (setq _O_dP_EE_c __O_dP_EE_c)
   (setq _O_ddP_EE_c __O_ddP_EE_c)
   (setq _theta __theta)
   (setq _dtheta __dtheta)
   (setq _current_errors __current_errors)
   (setq _last_motion_errors __last_motion_errors)
   (setq _control_command_success_rate (float __control_command_success_rate))
   (setq _robot_mode (round __robot_mode))
   (setq _robot_time (float __robot_time))
   (setq _gripper_width (float __gripper_width))
   (setq _gripper_max_width (float __gripper_max_width))
   (setq _gripper_is_grasped __gripper_is_grasped)
   (setq _gripper_temperature (round __gripper_temperature))
   (setq _gripper_time (float __gripper_time))
   (setq _is_fresh __is_fresh)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:pose_desired
   (&optional __pose_desired)
   (if __pose_desired (setq _pose_desired __pose_desired)) _pose_desired)
  (:O_T_EE
   (&optional __O_T_EE)
   (if __O_T_EE (setq _O_T_EE __O_T_EE)) _O_T_EE)
  (:O_T_EE_d
   (&optional __O_T_EE_d)
   (if __O_T_EE_d (setq _O_T_EE_d __O_T_EE_d)) _O_T_EE_d)
  (:F_T_EE
   (&optional __F_T_EE)
   (if __F_T_EE (setq _F_T_EE __F_T_EE)) _F_T_EE)
  (:EE_T_K
   (&optional __EE_T_K)
   (if __EE_T_K (setq _EE_T_K __EE_T_K)) _EE_T_K)
  (:m_ee
   (&optional __m_ee)
   (if __m_ee (setq _m_ee __m_ee)) _m_ee)
  (:I_ee
   (&optional __I_ee)
   (if __I_ee (setq _I_ee __I_ee)) _I_ee)
  (:F_x_Cee
   (&optional __F_x_Cee)
   (if __F_x_Cee (setq _F_x_Cee __F_x_Cee)) _F_x_Cee)
  (:m_load
   (&optional __m_load)
   (if __m_load (setq _m_load __m_load)) _m_load)
  (:I_load
   (&optional __I_load)
   (if __I_load (setq _I_load __I_load)) _I_load)
  (:F_x_Cload
   (&optional __F_x_Cload)
   (if __F_x_Cload (setq _F_x_Cload __F_x_Cload)) _F_x_Cload)
  (:m_total
   (&optional __m_total)
   (if __m_total (setq _m_total __m_total)) _m_total)
  (:I_total
   (&optional __I_total)
   (if __I_total (setq _I_total __I_total)) _I_total)
  (:F_x_Ctotal
   (&optional __F_x_Ctotal)
   (if __F_x_Ctotal (setq _F_x_Ctotal __F_x_Ctotal)) _F_x_Ctotal)
  (:elbow
   (&optional __elbow)
   (if __elbow (setq _elbow __elbow)) _elbow)
  (:elbow_d
   (&optional __elbow_d)
   (if __elbow_d (setq _elbow_d __elbow_d)) _elbow_d)
  (:elbow_c
   (&optional __elbow_c)
   (if __elbow_c (setq _elbow_c __elbow_c)) _elbow_c)
  (:delbow_c
   (&optional __delbow_c)
   (if __delbow_c (setq _delbow_c __delbow_c)) _delbow_c)
  (:ddelbow_c
   (&optional __ddelbow_c)
   (if __ddelbow_c (setq _ddelbow_c __ddelbow_c)) _ddelbow_c)
  (:tau_J
   (&optional __tau_J)
   (if __tau_J (setq _tau_J __tau_J)) _tau_J)
  (:tau_J_d
   (&optional __tau_J_d)
   (if __tau_J_d (setq _tau_J_d __tau_J_d)) _tau_J_d)
  (:dtau_J
   (&optional __dtau_J)
   (if __dtau_J (setq _dtau_J __dtau_J)) _dtau_J)
  (:q
   (&optional __q)
   (if __q (setq _q __q)) _q)
  (:q_d
   (&optional __q_d)
   (if __q_d (setq _q_d __q_d)) _q_d)
  (:dq
   (&optional __dq)
   (if __dq (setq _dq __dq)) _dq)
  (:dq_d
   (&optional __dq_d)
   (if __dq_d (setq _dq_d __dq_d)) _dq_d)
  (:ddq_d
   (&optional __ddq_d)
   (if __ddq_d (setq _ddq_d __ddq_d)) _ddq_d)
  (:joint_contact
   (&optional __joint_contact)
   (if __joint_contact (setq _joint_contact __joint_contact)) _joint_contact)
  (:cartesian_contact
   (&optional __cartesian_contact)
   (if __cartesian_contact (setq _cartesian_contact __cartesian_contact)) _cartesian_contact)
  (:joint_collision
   (&optional __joint_collision)
   (if __joint_collision (setq _joint_collision __joint_collision)) _joint_collision)
  (:cartesian_collision
   (&optional __cartesian_collision)
   (if __cartesian_collision (setq _cartesian_collision __cartesian_collision)) _cartesian_collision)
  (:tau_ext_hat_filtered
   (&optional __tau_ext_hat_filtered)
   (if __tau_ext_hat_filtered (setq _tau_ext_hat_filtered __tau_ext_hat_filtered)) _tau_ext_hat_filtered)
  (:O_F_ext_hat_K
   (&optional __O_F_ext_hat_K)
   (if __O_F_ext_hat_K (setq _O_F_ext_hat_K __O_F_ext_hat_K)) _O_F_ext_hat_K)
  (:K_F_ext_hat_K
   (&optional __K_F_ext_hat_K)
   (if __K_F_ext_hat_K (setq _K_F_ext_hat_K __K_F_ext_hat_K)) _K_F_ext_hat_K)
  (:O_dP_EE_d
   (&optional __O_dP_EE_d)
   (if __O_dP_EE_d (setq _O_dP_EE_d __O_dP_EE_d)) _O_dP_EE_d)
  (:O_T_EE_c
   (&optional __O_T_EE_c)
   (if __O_T_EE_c (setq _O_T_EE_c __O_T_EE_c)) _O_T_EE_c)
  (:O_dP_EE_c
   (&optional __O_dP_EE_c)
   (if __O_dP_EE_c (setq _O_dP_EE_c __O_dP_EE_c)) _O_dP_EE_c)
  (:O_ddP_EE_c
   (&optional __O_ddP_EE_c)
   (if __O_ddP_EE_c (setq _O_ddP_EE_c __O_ddP_EE_c)) _O_ddP_EE_c)
  (:theta
   (&optional __theta)
   (if __theta (setq _theta __theta)) _theta)
  (:dtheta
   (&optional __dtheta)
   (if __dtheta (setq _dtheta __dtheta)) _dtheta)
  (:current_errors
   (&rest __current_errors)
   (if (keywordp (car __current_errors))
       (send* _current_errors __current_errors)
     (progn
       (if __current_errors (setq _current_errors (car __current_errors)))
       _current_errors)))
  (:last_motion_errors
   (&rest __last_motion_errors)
   (if (keywordp (car __last_motion_errors))
       (send* _last_motion_errors __last_motion_errors)
     (progn
       (if __last_motion_errors (setq _last_motion_errors (car __last_motion_errors)))
       _last_motion_errors)))
  (:control_command_success_rate
   (&optional __control_command_success_rate)
   (if __control_command_success_rate (setq _control_command_success_rate __control_command_success_rate)) _control_command_success_rate)
  (:robot_mode
   (&optional __robot_mode)
   (if __robot_mode (setq _robot_mode __robot_mode)) _robot_mode)
  (:robot_time
   (&optional __robot_time)
   (if __robot_time (setq _robot_time __robot_time)) _robot_time)
  (:gripper_width
   (&optional __gripper_width)
   (if __gripper_width (setq _gripper_width __gripper_width)) _gripper_width)
  (:gripper_max_width
   (&optional __gripper_max_width)
   (if __gripper_max_width (setq _gripper_max_width __gripper_max_width)) _gripper_max_width)
  (:gripper_is_grasped
   (&optional (__gripper_is_grasped :null))
   (if (not (eq __gripper_is_grasped :null)) (setq _gripper_is_grasped __gripper_is_grasped)) _gripper_is_grasped)
  (:gripper_temperature
   (&optional __gripper_temperature)
   (if __gripper_temperature (setq _gripper_temperature __gripper_temperature)) _gripper_temperature)
  (:gripper_time
   (&optional __gripper_time)
   (if __gripper_time (setq _gripper_time __gripper_time)) _gripper_time)
  (:is_fresh
   (&optional (__is_fresh :null))
   (if (not (eq __is_fresh :null)) (setq _is_fresh __is_fresh)) _is_fresh)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64[16] _pose_desired
    (* 8    16)
    ;; float64[16] _O_T_EE
    (* 8    16)
    ;; float64[16] _O_T_EE_d
    (* 8    16)
    ;; float64[16] _F_T_EE
    (* 8    16)
    ;; float64[16] _EE_T_K
    (* 8    16)
    ;; float64 _m_ee
    8
    ;; float64[9] _I_ee
    (* 8    9)
    ;; float64[3] _F_x_Cee
    (* 8    3)
    ;; float64 _m_load
    8
    ;; float64[9] _I_load
    (* 8    9)
    ;; float64[3] _F_x_Cload
    (* 8    3)
    ;; float64 _m_total
    8
    ;; float64[9] _I_total
    (* 8    9)
    ;; float64[3] _F_x_Ctotal
    (* 8    3)
    ;; float64[2] _elbow
    (* 8    2)
    ;; float64[2] _elbow_d
    (* 8    2)
    ;; float64[2] _elbow_c
    (* 8    2)
    ;; float64[2] _delbow_c
    (* 8    2)
    ;; float64[2] _ddelbow_c
    (* 8    2)
    ;; float64[7] _tau_J
    (* 8    7)
    ;; float64[7] _tau_J_d
    (* 8    7)
    ;; float64[7] _dtau_J
    (* 8    7)
    ;; float64[7] _q
    (* 8    7)
    ;; float64[7] _q_d
    (* 8    7)
    ;; float64[7] _dq
    (* 8    7)
    ;; float64[7] _dq_d
    (* 8    7)
    ;; float64[7] _ddq_d
    (* 8    7)
    ;; float64[7] _joint_contact
    (* 8    7)
    ;; float64[6] _cartesian_contact
    (* 8    6)
    ;; float64[7] _joint_collision
    (* 8    7)
    ;; float64[6] _cartesian_collision
    (* 8    6)
    ;; float64[7] _tau_ext_hat_filtered
    (* 8    7)
    ;; float64[6] _O_F_ext_hat_K
    (* 8    6)
    ;; float64[6] _K_F_ext_hat_K
    (* 8    6)
    ;; float64[6] _O_dP_EE_d
    (* 8    6)
    ;; float64[16] _O_T_EE_c
    (* 8    16)
    ;; float64[6] _O_dP_EE_c
    (* 8    6)
    ;; float64[6] _O_ddP_EE_c
    (* 8    6)
    ;; float64[7] _theta
    (* 8    7)
    ;; float64[7] _dtheta
    (* 8    7)
    ;; franka_interface_msgs/Errors _current_errors
    (send _current_errors :serialization-length)
    ;; franka_interface_msgs/Errors _last_motion_errors
    (send _last_motion_errors :serialization-length)
    ;; float64 _control_command_success_rate
    8
    ;; uint8 _robot_mode
    1
    ;; float64 _robot_time
    8
    ;; float64 _gripper_width
    8
    ;; float64 _gripper_max_width
    8
    ;; bool _gripper_is_grasped
    1
    ;; uint16 _gripper_temperature
    2
    ;; float64 _gripper_time
    8
    ;; bool _is_fresh
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64[16] _pose_desired
     (dotimes (i 16)
       (sys::poke (elt _pose_desired i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[16] _O_T_EE
     (dotimes (i 16)
       (sys::poke (elt _O_T_EE i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[16] _O_T_EE_d
     (dotimes (i 16)
       (sys::poke (elt _O_T_EE_d i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[16] _F_T_EE
     (dotimes (i 16)
       (sys::poke (elt _F_T_EE i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[16] _EE_T_K
     (dotimes (i 16)
       (sys::poke (elt _EE_T_K i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64 _m_ee
       (sys::poke _m_ee (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64[9] _I_ee
     (dotimes (i 9)
       (sys::poke (elt _I_ee i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[3] _F_x_Cee
     (dotimes (i 3)
       (sys::poke (elt _F_x_Cee i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64 _m_load
       (sys::poke _m_load (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64[9] _I_load
     (dotimes (i 9)
       (sys::poke (elt _I_load i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[3] _F_x_Cload
     (dotimes (i 3)
       (sys::poke (elt _F_x_Cload i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64 _m_total
       (sys::poke _m_total (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64[9] _I_total
     (dotimes (i 9)
       (sys::poke (elt _I_total i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[3] _F_x_Ctotal
     (dotimes (i 3)
       (sys::poke (elt _F_x_Ctotal i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[2] _elbow
     (dotimes (i 2)
       (sys::poke (elt _elbow i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[2] _elbow_d
     (dotimes (i 2)
       (sys::poke (elt _elbow_d i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[2] _elbow_c
     (dotimes (i 2)
       (sys::poke (elt _elbow_c i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[2] _delbow_c
     (dotimes (i 2)
       (sys::poke (elt _delbow_c i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[2] _ddelbow_c
     (dotimes (i 2)
       (sys::poke (elt _ddelbow_c i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[7] _tau_J
     (dotimes (i 7)
       (sys::poke (elt _tau_J i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[7] _tau_J_d
     (dotimes (i 7)
       (sys::poke (elt _tau_J_d i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[7] _dtau_J
     (dotimes (i 7)
       (sys::poke (elt _dtau_J i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[7] _q
     (dotimes (i 7)
       (sys::poke (elt _q i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[7] _q_d
     (dotimes (i 7)
       (sys::poke (elt _q_d i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[7] _dq
     (dotimes (i 7)
       (sys::poke (elt _dq i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[7] _dq_d
     (dotimes (i 7)
       (sys::poke (elt _dq_d i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[7] _ddq_d
     (dotimes (i 7)
       (sys::poke (elt _ddq_d i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[7] _joint_contact
     (dotimes (i 7)
       (sys::poke (elt _joint_contact i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _cartesian_contact
     (dotimes (i 6)
       (sys::poke (elt _cartesian_contact i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[7] _joint_collision
     (dotimes (i 7)
       (sys::poke (elt _joint_collision i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _cartesian_collision
     (dotimes (i 6)
       (sys::poke (elt _cartesian_collision i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[7] _tau_ext_hat_filtered
     (dotimes (i 7)
       (sys::poke (elt _tau_ext_hat_filtered i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _O_F_ext_hat_K
     (dotimes (i 6)
       (sys::poke (elt _O_F_ext_hat_K i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _K_F_ext_hat_K
     (dotimes (i 6)
       (sys::poke (elt _K_F_ext_hat_K i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _O_dP_EE_d
     (dotimes (i 6)
       (sys::poke (elt _O_dP_EE_d i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[16] _O_T_EE_c
     (dotimes (i 16)
       (sys::poke (elt _O_T_EE_c i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _O_dP_EE_c
     (dotimes (i 6)
       (sys::poke (elt _O_dP_EE_c i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[6] _O_ddP_EE_c
     (dotimes (i 6)
       (sys::poke (elt _O_ddP_EE_c i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[7] _theta
     (dotimes (i 7)
       (sys::poke (elt _theta i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[7] _dtheta
     (dotimes (i 7)
       (sys::poke (elt _dtheta i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; franka_interface_msgs/Errors _current_errors
       (send _current_errors :serialize s)
     ;; franka_interface_msgs/Errors _last_motion_errors
       (send _last_motion_errors :serialize s)
     ;; float64 _control_command_success_rate
       (sys::poke _control_command_success_rate (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; uint8 _robot_mode
       (write-byte _robot_mode s)
     ;; float64 _robot_time
       (sys::poke _robot_time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _gripper_width
       (sys::poke _gripper_width (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _gripper_max_width
       (sys::poke _gripper_max_width (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _gripper_is_grasped
       (if _gripper_is_grasped (write-byte -1 s) (write-byte 0 s))
     ;; uint16 _gripper_temperature
       (write-word _gripper_temperature s)
     ;; float64 _gripper_time
       (sys::poke _gripper_time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _is_fresh
       (if _is_fresh (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64[16] _pose_desired
   (dotimes (i (length _pose_desired))
     (setf (elt _pose_desired i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[16] _O_T_EE
   (dotimes (i (length _O_T_EE))
     (setf (elt _O_T_EE i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[16] _O_T_EE_d
   (dotimes (i (length _O_T_EE_d))
     (setf (elt _O_T_EE_d i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[16] _F_T_EE
   (dotimes (i (length _F_T_EE))
     (setf (elt _F_T_EE i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[16] _EE_T_K
   (dotimes (i (length _EE_T_K))
     (setf (elt _EE_T_K i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64 _m_ee
     (setq _m_ee (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64[9] _I_ee
   (dotimes (i (length _I_ee))
     (setf (elt _I_ee i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[3] _F_x_Cee
   (dotimes (i (length _F_x_Cee))
     (setf (elt _F_x_Cee i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64 _m_load
     (setq _m_load (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64[9] _I_load
   (dotimes (i (length _I_load))
     (setf (elt _I_load i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[3] _F_x_Cload
   (dotimes (i (length _F_x_Cload))
     (setf (elt _F_x_Cload i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64 _m_total
     (setq _m_total (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64[9] _I_total
   (dotimes (i (length _I_total))
     (setf (elt _I_total i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[3] _F_x_Ctotal
   (dotimes (i (length _F_x_Ctotal))
     (setf (elt _F_x_Ctotal i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[2] _elbow
   (dotimes (i (length _elbow))
     (setf (elt _elbow i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[2] _elbow_d
   (dotimes (i (length _elbow_d))
     (setf (elt _elbow_d i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[2] _elbow_c
   (dotimes (i (length _elbow_c))
     (setf (elt _elbow_c i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[2] _delbow_c
   (dotimes (i (length _delbow_c))
     (setf (elt _delbow_c i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[2] _ddelbow_c
   (dotimes (i (length _ddelbow_c))
     (setf (elt _ddelbow_c i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[7] _tau_J
   (dotimes (i (length _tau_J))
     (setf (elt _tau_J i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[7] _tau_J_d
   (dotimes (i (length _tau_J_d))
     (setf (elt _tau_J_d i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[7] _dtau_J
   (dotimes (i (length _dtau_J))
     (setf (elt _dtau_J i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[7] _q
   (dotimes (i (length _q))
     (setf (elt _q i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[7] _q_d
   (dotimes (i (length _q_d))
     (setf (elt _q_d i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[7] _dq
   (dotimes (i (length _dq))
     (setf (elt _dq i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[7] _dq_d
   (dotimes (i (length _dq_d))
     (setf (elt _dq_d i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[7] _ddq_d
   (dotimes (i (length _ddq_d))
     (setf (elt _ddq_d i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[7] _joint_contact
   (dotimes (i (length _joint_contact))
     (setf (elt _joint_contact i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _cartesian_contact
   (dotimes (i (length _cartesian_contact))
     (setf (elt _cartesian_contact i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[7] _joint_collision
   (dotimes (i (length _joint_collision))
     (setf (elt _joint_collision i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _cartesian_collision
   (dotimes (i (length _cartesian_collision))
     (setf (elt _cartesian_collision i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[7] _tau_ext_hat_filtered
   (dotimes (i (length _tau_ext_hat_filtered))
     (setf (elt _tau_ext_hat_filtered i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _O_F_ext_hat_K
   (dotimes (i (length _O_F_ext_hat_K))
     (setf (elt _O_F_ext_hat_K i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _K_F_ext_hat_K
   (dotimes (i (length _K_F_ext_hat_K))
     (setf (elt _K_F_ext_hat_K i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _O_dP_EE_d
   (dotimes (i (length _O_dP_EE_d))
     (setf (elt _O_dP_EE_d i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[16] _O_T_EE_c
   (dotimes (i (length _O_T_EE_c))
     (setf (elt _O_T_EE_c i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _O_dP_EE_c
   (dotimes (i (length _O_dP_EE_c))
     (setf (elt _O_dP_EE_c i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[6] _O_ddP_EE_c
   (dotimes (i (length _O_ddP_EE_c))
     (setf (elt _O_ddP_EE_c i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[7] _theta
   (dotimes (i (length _theta))
     (setf (elt _theta i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[7] _dtheta
   (dotimes (i (length _dtheta))
     (setf (elt _dtheta i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; franka_interface_msgs/Errors _current_errors
     (send _current_errors :deserialize buf ptr-) (incf ptr- (send _current_errors :serialization-length))
   ;; franka_interface_msgs/Errors _last_motion_errors
     (send _last_motion_errors :deserialize buf ptr-) (incf ptr- (send _last_motion_errors :serialization-length))
   ;; float64 _control_command_success_rate
     (setq _control_command_success_rate (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; uint8 _robot_mode
     (setq _robot_mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float64 _robot_time
     (setq _robot_time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _gripper_width
     (setq _gripper_width (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _gripper_max_width
     (setq _gripper_max_width (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _gripper_is_grasped
     (setq _gripper_is_grasped (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint16 _gripper_temperature
     (setq _gripper_temperature (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float64 _gripper_time
     (setq _gripper_time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _is_fresh
     (setq _is_fresh (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get franka_interface_msgs::RobotState :md5sum-) "1bd13b23161f53b273eafd78a98128f7")
(setf (get franka_interface_msgs::RobotState :datatype-) "franka_interface_msgs/RobotState")
(setf (get franka_interface_msgs::RobotState :definition-)
      "# Franka robot state
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
bool joint_move_in_wrong_direction
")



(provide :franka_interface_msgs/RobotState "1bd13b23161f53b273eafd78a98128f7")



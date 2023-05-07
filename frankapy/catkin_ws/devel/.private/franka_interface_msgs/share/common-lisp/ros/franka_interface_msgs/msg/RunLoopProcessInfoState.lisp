; Auto-generated. Do not edit!


(cl:in-package franka_interface_msgs-msg)


;//! \htmlinclude RunLoopProcessInfoState.msg.html

(cl:defclass <RunLoopProcessInfoState> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (current_memory_region
    :reader current_memory_region
    :initarg :current_memory_region
    :type cl:integer
    :initform 0)
   (current_sensor_region
    :reader current_sensor_region
    :initarg :current_sensor_region
    :type cl:integer
    :initform 0)
   (current_feedback_region
    :reader current_feedback_region
    :initarg :current_feedback_region
    :type cl:integer
    :initform 0)
   (current_skill_id
    :reader current_skill_id
    :initarg :current_skill_id
    :type cl:integer
    :initform 0)
   (current_skill_type
    :reader current_skill_type
    :initarg :current_skill_type
    :type cl:integer
    :initform 0)
   (current_meta_skill_id
    :reader current_meta_skill_id
    :initarg :current_meta_skill_id
    :type cl:integer
    :initform 0)
   (current_meta_skill_type
    :reader current_meta_skill_type
    :initarg :current_meta_skill_type
    :type cl:integer
    :initform 0)
   (current_skill_description
    :reader current_skill_description
    :initarg :current_skill_description
    :type cl:string
    :initform "")
   (new_skill_available
    :reader new_skill_available
    :initarg :new_skill_available
    :type cl:boolean
    :initform cl:nil)
   (new_skill_id
    :reader new_skill_id
    :initarg :new_skill_id
    :type cl:integer
    :initform 0)
   (new_skill_type
    :reader new_skill_type
    :initarg :new_skill_type
    :type cl:integer
    :initform 0)
   (new_meta_skill_id
    :reader new_meta_skill_id
    :initarg :new_meta_skill_id
    :type cl:integer
    :initform 0)
   (new_meta_skill_type
    :reader new_meta_skill_type
    :initarg :new_meta_skill_type
    :type cl:integer
    :initform 0)
   (new_skill_description
    :reader new_skill_description
    :initarg :new_skill_description
    :type cl:string
    :initform "")
   (is_running_skill
    :reader is_running_skill
    :initarg :is_running_skill
    :type cl:boolean
    :initform cl:nil)
   (skill_preempted
    :reader skill_preempted
    :initarg :skill_preempted
    :type cl:boolean
    :initform cl:nil)
   (done_skill_id
    :reader done_skill_id
    :initarg :done_skill_id
    :type cl:integer
    :initform 0)
   (result_skill_id
    :reader result_skill_id
    :initarg :result_skill_id
    :type cl:integer
    :initform 0)
   (time_skill_started_in_robot_time
    :reader time_skill_started_in_robot_time
    :initarg :time_skill_started_in_robot_time
    :type cl:float
    :initform 0.0)
   (time_skill_finished_in_robot_time
    :reader time_skill_finished_in_robot_time
    :initarg :time_skill_finished_in_robot_time
    :type cl:float
    :initform 0.0)
   (is_fresh
    :reader is_fresh
    :initarg :is_fresh
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass RunLoopProcessInfoState (<RunLoopProcessInfoState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RunLoopProcessInfoState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RunLoopProcessInfoState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name franka_interface_msgs-msg:<RunLoopProcessInfoState> is deprecated: use franka_interface_msgs-msg:RunLoopProcessInfoState instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <RunLoopProcessInfoState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:header-val is deprecated.  Use franka_interface_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'current_memory_region-val :lambda-list '(m))
(cl:defmethod current_memory_region-val ((m <RunLoopProcessInfoState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:current_memory_region-val is deprecated.  Use franka_interface_msgs-msg:current_memory_region instead.")
  (current_memory_region m))

(cl:ensure-generic-function 'current_sensor_region-val :lambda-list '(m))
(cl:defmethod current_sensor_region-val ((m <RunLoopProcessInfoState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:current_sensor_region-val is deprecated.  Use franka_interface_msgs-msg:current_sensor_region instead.")
  (current_sensor_region m))

(cl:ensure-generic-function 'current_feedback_region-val :lambda-list '(m))
(cl:defmethod current_feedback_region-val ((m <RunLoopProcessInfoState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:current_feedback_region-val is deprecated.  Use franka_interface_msgs-msg:current_feedback_region instead.")
  (current_feedback_region m))

(cl:ensure-generic-function 'current_skill_id-val :lambda-list '(m))
(cl:defmethod current_skill_id-val ((m <RunLoopProcessInfoState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:current_skill_id-val is deprecated.  Use franka_interface_msgs-msg:current_skill_id instead.")
  (current_skill_id m))

(cl:ensure-generic-function 'current_skill_type-val :lambda-list '(m))
(cl:defmethod current_skill_type-val ((m <RunLoopProcessInfoState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:current_skill_type-val is deprecated.  Use franka_interface_msgs-msg:current_skill_type instead.")
  (current_skill_type m))

(cl:ensure-generic-function 'current_meta_skill_id-val :lambda-list '(m))
(cl:defmethod current_meta_skill_id-val ((m <RunLoopProcessInfoState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:current_meta_skill_id-val is deprecated.  Use franka_interface_msgs-msg:current_meta_skill_id instead.")
  (current_meta_skill_id m))

(cl:ensure-generic-function 'current_meta_skill_type-val :lambda-list '(m))
(cl:defmethod current_meta_skill_type-val ((m <RunLoopProcessInfoState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:current_meta_skill_type-val is deprecated.  Use franka_interface_msgs-msg:current_meta_skill_type instead.")
  (current_meta_skill_type m))

(cl:ensure-generic-function 'current_skill_description-val :lambda-list '(m))
(cl:defmethod current_skill_description-val ((m <RunLoopProcessInfoState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:current_skill_description-val is deprecated.  Use franka_interface_msgs-msg:current_skill_description instead.")
  (current_skill_description m))

(cl:ensure-generic-function 'new_skill_available-val :lambda-list '(m))
(cl:defmethod new_skill_available-val ((m <RunLoopProcessInfoState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:new_skill_available-val is deprecated.  Use franka_interface_msgs-msg:new_skill_available instead.")
  (new_skill_available m))

(cl:ensure-generic-function 'new_skill_id-val :lambda-list '(m))
(cl:defmethod new_skill_id-val ((m <RunLoopProcessInfoState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:new_skill_id-val is deprecated.  Use franka_interface_msgs-msg:new_skill_id instead.")
  (new_skill_id m))

(cl:ensure-generic-function 'new_skill_type-val :lambda-list '(m))
(cl:defmethod new_skill_type-val ((m <RunLoopProcessInfoState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:new_skill_type-val is deprecated.  Use franka_interface_msgs-msg:new_skill_type instead.")
  (new_skill_type m))

(cl:ensure-generic-function 'new_meta_skill_id-val :lambda-list '(m))
(cl:defmethod new_meta_skill_id-val ((m <RunLoopProcessInfoState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:new_meta_skill_id-val is deprecated.  Use franka_interface_msgs-msg:new_meta_skill_id instead.")
  (new_meta_skill_id m))

(cl:ensure-generic-function 'new_meta_skill_type-val :lambda-list '(m))
(cl:defmethod new_meta_skill_type-val ((m <RunLoopProcessInfoState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:new_meta_skill_type-val is deprecated.  Use franka_interface_msgs-msg:new_meta_skill_type instead.")
  (new_meta_skill_type m))

(cl:ensure-generic-function 'new_skill_description-val :lambda-list '(m))
(cl:defmethod new_skill_description-val ((m <RunLoopProcessInfoState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:new_skill_description-val is deprecated.  Use franka_interface_msgs-msg:new_skill_description instead.")
  (new_skill_description m))

(cl:ensure-generic-function 'is_running_skill-val :lambda-list '(m))
(cl:defmethod is_running_skill-val ((m <RunLoopProcessInfoState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:is_running_skill-val is deprecated.  Use franka_interface_msgs-msg:is_running_skill instead.")
  (is_running_skill m))

(cl:ensure-generic-function 'skill_preempted-val :lambda-list '(m))
(cl:defmethod skill_preempted-val ((m <RunLoopProcessInfoState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:skill_preempted-val is deprecated.  Use franka_interface_msgs-msg:skill_preempted instead.")
  (skill_preempted m))

(cl:ensure-generic-function 'done_skill_id-val :lambda-list '(m))
(cl:defmethod done_skill_id-val ((m <RunLoopProcessInfoState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:done_skill_id-val is deprecated.  Use franka_interface_msgs-msg:done_skill_id instead.")
  (done_skill_id m))

(cl:ensure-generic-function 'result_skill_id-val :lambda-list '(m))
(cl:defmethod result_skill_id-val ((m <RunLoopProcessInfoState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:result_skill_id-val is deprecated.  Use franka_interface_msgs-msg:result_skill_id instead.")
  (result_skill_id m))

(cl:ensure-generic-function 'time_skill_started_in_robot_time-val :lambda-list '(m))
(cl:defmethod time_skill_started_in_robot_time-val ((m <RunLoopProcessInfoState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:time_skill_started_in_robot_time-val is deprecated.  Use franka_interface_msgs-msg:time_skill_started_in_robot_time instead.")
  (time_skill_started_in_robot_time m))

(cl:ensure-generic-function 'time_skill_finished_in_robot_time-val :lambda-list '(m))
(cl:defmethod time_skill_finished_in_robot_time-val ((m <RunLoopProcessInfoState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:time_skill_finished_in_robot_time-val is deprecated.  Use franka_interface_msgs-msg:time_skill_finished_in_robot_time instead.")
  (time_skill_finished_in_robot_time m))

(cl:ensure-generic-function 'is_fresh-val :lambda-list '(m))
(cl:defmethod is_fresh-val ((m <RunLoopProcessInfoState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:is_fresh-val is deprecated.  Use franka_interface_msgs-msg:is_fresh instead.")
  (is_fresh m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RunLoopProcessInfoState>) ostream)
  "Serializes a message object of type '<RunLoopProcessInfoState>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'current_memory_region)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'current_sensor_region)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'current_feedback_region)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'current_skill_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'current_skill_type)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'current_meta_skill_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'current_meta_skill_type)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'current_skill_description))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'current_skill_description))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'new_skill_available) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'new_skill_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'new_skill_type)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'new_meta_skill_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'new_meta_skill_type)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'new_skill_description))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'new_skill_description))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_running_skill) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'skill_preempted) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'done_skill_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'result_skill_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'time_skill_started_in_robot_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'time_skill_finished_in_robot_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_fresh) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RunLoopProcessInfoState>) istream)
  "Deserializes a message object of type '<RunLoopProcessInfoState>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'current_memory_region) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'current_sensor_region) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'current_feedback_region) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'current_skill_id) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'current_skill_type) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'current_meta_skill_id) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'current_meta_skill_type) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'current_skill_description) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'current_skill_description) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'new_skill_available) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'new_skill_id) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'new_skill_type) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'new_meta_skill_id) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'new_meta_skill_type) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'new_skill_description) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'new_skill_description) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'is_running_skill) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'skill_preempted) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'done_skill_id) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'result_skill_id) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'time_skill_started_in_robot_time) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'time_skill_finished_in_robot_time) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'is_fresh) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RunLoopProcessInfoState>)))
  "Returns string type for a message object of type '<RunLoopProcessInfoState>"
  "franka_interface_msgs/RunLoopProcessInfoState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RunLoopProcessInfoState)))
  "Returns string type for a message object of type 'RunLoopProcessInfoState"
  "franka_interface_msgs/RunLoopProcessInfoState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RunLoopProcessInfoState>)))
  "Returns md5sum for a message object of type '<RunLoopProcessInfoState>"
  "86cd879bb4a3cbdb82beb45c42836130")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RunLoopProcessInfoState)))
  "Returns md5sum for a message object of type 'RunLoopProcessInfoState"
  "86cd879bb4a3cbdb82beb45c42836130")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RunLoopProcessInfoState>)))
  "Returns full string definition for message of type '<RunLoopProcessInfoState>"
  (cl:format cl:nil "# Skill state~%std_msgs/Header header~%int64 current_memory_region~%int64 current_sensor_region~%int64 current_feedback_region~%int64 current_skill_id ~%int64 current_skill_type~%int64 current_meta_skill_id~%int64 current_meta_skill_type~%string current_skill_description~%bool new_skill_available~%int64 new_skill_id~%int64 new_skill_type~%int64 new_meta_skill_id~%int64 new_meta_skill_type~%string new_skill_description~%bool is_running_skill~%bool skill_preempted~%int64 done_skill_id~%int64 result_skill_id~%float64 time_skill_started_in_robot_time~%float64 time_skill_finished_in_robot_time~%bool is_fresh~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RunLoopProcessInfoState)))
  "Returns full string definition for message of type 'RunLoopProcessInfoState"
  (cl:format cl:nil "# Skill state~%std_msgs/Header header~%int64 current_memory_region~%int64 current_sensor_region~%int64 current_feedback_region~%int64 current_skill_id ~%int64 current_skill_type~%int64 current_meta_skill_id~%int64 current_meta_skill_type~%string current_skill_description~%bool new_skill_available~%int64 new_skill_id~%int64 new_skill_type~%int64 new_meta_skill_id~%int64 new_meta_skill_type~%string new_skill_description~%bool is_running_skill~%bool skill_preempted~%int64 done_skill_id~%int64 result_skill_id~%float64 time_skill_started_in_robot_time~%float64 time_skill_finished_in_robot_time~%bool is_fresh~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RunLoopProcessInfoState>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     8
     8
     8
     8
     8
     8
     4 (cl:length (cl:slot-value msg 'current_skill_description))
     1
     8
     8
     8
     8
     4 (cl:length (cl:slot-value msg 'new_skill_description))
     1
     1
     8
     8
     8
     8
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RunLoopProcessInfoState>))
  "Converts a ROS message object to a list"
  (cl:list 'RunLoopProcessInfoState
    (cl:cons ':header (header msg))
    (cl:cons ':current_memory_region (current_memory_region msg))
    (cl:cons ':current_sensor_region (current_sensor_region msg))
    (cl:cons ':current_feedback_region (current_feedback_region msg))
    (cl:cons ':current_skill_id (current_skill_id msg))
    (cl:cons ':current_skill_type (current_skill_type msg))
    (cl:cons ':current_meta_skill_id (current_meta_skill_id msg))
    (cl:cons ':current_meta_skill_type (current_meta_skill_type msg))
    (cl:cons ':current_skill_description (current_skill_description msg))
    (cl:cons ':new_skill_available (new_skill_available msg))
    (cl:cons ':new_skill_id (new_skill_id msg))
    (cl:cons ':new_skill_type (new_skill_type msg))
    (cl:cons ':new_meta_skill_id (new_meta_skill_id msg))
    (cl:cons ':new_meta_skill_type (new_meta_skill_type msg))
    (cl:cons ':new_skill_description (new_skill_description msg))
    (cl:cons ':is_running_skill (is_running_skill msg))
    (cl:cons ':skill_preempted (skill_preempted msg))
    (cl:cons ':done_skill_id (done_skill_id msg))
    (cl:cons ':result_skill_id (result_skill_id msg))
    (cl:cons ':time_skill_started_in_robot_time (time_skill_started_in_robot_time msg))
    (cl:cons ':time_skill_finished_in_robot_time (time_skill_finished_in_robot_time msg))
    (cl:cons ':is_fresh (is_fresh msg))
))

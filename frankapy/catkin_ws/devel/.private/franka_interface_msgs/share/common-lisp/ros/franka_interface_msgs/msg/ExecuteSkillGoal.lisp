; Auto-generated. Do not edit!


(cl:in-package franka_interface_msgs-msg)


;//! \htmlinclude ExecuteSkillGoal.msg.html

(cl:defclass <ExecuteSkillGoal> (roslisp-msg-protocol:ros-message)
  ((skill_type
    :reader skill_type
    :initarg :skill_type
    :type cl:fixnum
    :initform 0)
   (skill_description
    :reader skill_description
    :initarg :skill_description
    :type cl:string
    :initform "")
   (meta_skill_type
    :reader meta_skill_type
    :initarg :meta_skill_type
    :type cl:fixnum
    :initform 0)
   (meta_skill_id
    :reader meta_skill_id
    :initarg :meta_skill_id
    :type cl:integer
    :initform 0)
   (sensor_topics
    :reader sensor_topics
    :initarg :sensor_topics
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (sensor_value_sizes
    :reader sensor_value_sizes
    :initarg :sensor_value_sizes
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (initial_sensor_values
    :reader initial_sensor_values
    :initarg :initial_sensor_values
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (trajectory_generator_type
    :reader trajectory_generator_type
    :initarg :trajectory_generator_type
    :type cl:fixnum
    :initform 0)
   (trajectory_generator_param_data_size
    :reader trajectory_generator_param_data_size
    :initarg :trajectory_generator_param_data_size
    :type cl:integer
    :initform 0)
   (trajectory_generator_param_data
    :reader trajectory_generator_param_data
    :initarg :trajectory_generator_param_data
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (feedback_controller_type
    :reader feedback_controller_type
    :initarg :feedback_controller_type
    :type cl:fixnum
    :initform 0)
   (feedback_controller_param_data_size
    :reader feedback_controller_param_data_size
    :initarg :feedback_controller_param_data_size
    :type cl:integer
    :initform 0)
   (feedback_controller_param_data
    :reader feedback_controller_param_data
    :initarg :feedback_controller_param_data
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (termination_handler_type
    :reader termination_handler_type
    :initarg :termination_handler_type
    :type cl:fixnum
    :initform 0)
   (termination_handler_param_data_size
    :reader termination_handler_param_data_size
    :initarg :termination_handler_param_data_size
    :type cl:integer
    :initform 0)
   (termination_handler_param_data
    :reader termination_handler_param_data
    :initarg :termination_handler_param_data
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (timer_type
    :reader timer_type
    :initarg :timer_type
    :type cl:fixnum
    :initform 0)
   (num_timer_params
    :reader num_timer_params
    :initarg :num_timer_params
    :type cl:integer
    :initform 0)
   (timer_params
    :reader timer_params
    :initarg :timer_params
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass ExecuteSkillGoal (<ExecuteSkillGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ExecuteSkillGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ExecuteSkillGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name franka_interface_msgs-msg:<ExecuteSkillGoal> is deprecated: use franka_interface_msgs-msg:ExecuteSkillGoal instead.")))

(cl:ensure-generic-function 'skill_type-val :lambda-list '(m))
(cl:defmethod skill_type-val ((m <ExecuteSkillGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:skill_type-val is deprecated.  Use franka_interface_msgs-msg:skill_type instead.")
  (skill_type m))

(cl:ensure-generic-function 'skill_description-val :lambda-list '(m))
(cl:defmethod skill_description-val ((m <ExecuteSkillGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:skill_description-val is deprecated.  Use franka_interface_msgs-msg:skill_description instead.")
  (skill_description m))

(cl:ensure-generic-function 'meta_skill_type-val :lambda-list '(m))
(cl:defmethod meta_skill_type-val ((m <ExecuteSkillGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:meta_skill_type-val is deprecated.  Use franka_interface_msgs-msg:meta_skill_type instead.")
  (meta_skill_type m))

(cl:ensure-generic-function 'meta_skill_id-val :lambda-list '(m))
(cl:defmethod meta_skill_id-val ((m <ExecuteSkillGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:meta_skill_id-val is deprecated.  Use franka_interface_msgs-msg:meta_skill_id instead.")
  (meta_skill_id m))

(cl:ensure-generic-function 'sensor_topics-val :lambda-list '(m))
(cl:defmethod sensor_topics-val ((m <ExecuteSkillGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:sensor_topics-val is deprecated.  Use franka_interface_msgs-msg:sensor_topics instead.")
  (sensor_topics m))

(cl:ensure-generic-function 'sensor_value_sizes-val :lambda-list '(m))
(cl:defmethod sensor_value_sizes-val ((m <ExecuteSkillGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:sensor_value_sizes-val is deprecated.  Use franka_interface_msgs-msg:sensor_value_sizes instead.")
  (sensor_value_sizes m))

(cl:ensure-generic-function 'initial_sensor_values-val :lambda-list '(m))
(cl:defmethod initial_sensor_values-val ((m <ExecuteSkillGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:initial_sensor_values-val is deprecated.  Use franka_interface_msgs-msg:initial_sensor_values instead.")
  (initial_sensor_values m))

(cl:ensure-generic-function 'trajectory_generator_type-val :lambda-list '(m))
(cl:defmethod trajectory_generator_type-val ((m <ExecuteSkillGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:trajectory_generator_type-val is deprecated.  Use franka_interface_msgs-msg:trajectory_generator_type instead.")
  (trajectory_generator_type m))

(cl:ensure-generic-function 'trajectory_generator_param_data_size-val :lambda-list '(m))
(cl:defmethod trajectory_generator_param_data_size-val ((m <ExecuteSkillGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:trajectory_generator_param_data_size-val is deprecated.  Use franka_interface_msgs-msg:trajectory_generator_param_data_size instead.")
  (trajectory_generator_param_data_size m))

(cl:ensure-generic-function 'trajectory_generator_param_data-val :lambda-list '(m))
(cl:defmethod trajectory_generator_param_data-val ((m <ExecuteSkillGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:trajectory_generator_param_data-val is deprecated.  Use franka_interface_msgs-msg:trajectory_generator_param_data instead.")
  (trajectory_generator_param_data m))

(cl:ensure-generic-function 'feedback_controller_type-val :lambda-list '(m))
(cl:defmethod feedback_controller_type-val ((m <ExecuteSkillGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:feedback_controller_type-val is deprecated.  Use franka_interface_msgs-msg:feedback_controller_type instead.")
  (feedback_controller_type m))

(cl:ensure-generic-function 'feedback_controller_param_data_size-val :lambda-list '(m))
(cl:defmethod feedback_controller_param_data_size-val ((m <ExecuteSkillGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:feedback_controller_param_data_size-val is deprecated.  Use franka_interface_msgs-msg:feedback_controller_param_data_size instead.")
  (feedback_controller_param_data_size m))

(cl:ensure-generic-function 'feedback_controller_param_data-val :lambda-list '(m))
(cl:defmethod feedback_controller_param_data-val ((m <ExecuteSkillGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:feedback_controller_param_data-val is deprecated.  Use franka_interface_msgs-msg:feedback_controller_param_data instead.")
  (feedback_controller_param_data m))

(cl:ensure-generic-function 'termination_handler_type-val :lambda-list '(m))
(cl:defmethod termination_handler_type-val ((m <ExecuteSkillGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:termination_handler_type-val is deprecated.  Use franka_interface_msgs-msg:termination_handler_type instead.")
  (termination_handler_type m))

(cl:ensure-generic-function 'termination_handler_param_data_size-val :lambda-list '(m))
(cl:defmethod termination_handler_param_data_size-val ((m <ExecuteSkillGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:termination_handler_param_data_size-val is deprecated.  Use franka_interface_msgs-msg:termination_handler_param_data_size instead.")
  (termination_handler_param_data_size m))

(cl:ensure-generic-function 'termination_handler_param_data-val :lambda-list '(m))
(cl:defmethod termination_handler_param_data-val ((m <ExecuteSkillGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:termination_handler_param_data-val is deprecated.  Use franka_interface_msgs-msg:termination_handler_param_data instead.")
  (termination_handler_param_data m))

(cl:ensure-generic-function 'timer_type-val :lambda-list '(m))
(cl:defmethod timer_type-val ((m <ExecuteSkillGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:timer_type-val is deprecated.  Use franka_interface_msgs-msg:timer_type instead.")
  (timer_type m))

(cl:ensure-generic-function 'num_timer_params-val :lambda-list '(m))
(cl:defmethod num_timer_params-val ((m <ExecuteSkillGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:num_timer_params-val is deprecated.  Use franka_interface_msgs-msg:num_timer_params instead.")
  (num_timer_params m))

(cl:ensure-generic-function 'timer_params-val :lambda-list '(m))
(cl:defmethod timer_params-val ((m <ExecuteSkillGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:timer_params-val is deprecated.  Use franka_interface_msgs-msg:timer_params instead.")
  (timer_params m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ExecuteSkillGoal>) ostream)
  "Serializes a message object of type '<ExecuteSkillGoal>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'skill_type)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'skill_description))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'skill_description))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'meta_skill_type)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'meta_skill_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'sensor_topics))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'sensor_topics))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'sensor_value_sizes))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) ele) ostream))
   (cl:slot-value msg 'sensor_value_sizes))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'initial_sensor_values))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'initial_sensor_values))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'trajectory_generator_type)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'trajectory_generator_param_data_size)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'trajectory_generator_param_data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'trajectory_generator_param_data))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'feedback_controller_type)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'feedback_controller_param_data_size)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'feedback_controller_param_data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'feedback_controller_param_data))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'termination_handler_type)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'termination_handler_param_data_size)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'termination_handler_param_data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'termination_handler_param_data))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'timer_type)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'num_timer_params)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'timer_params))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'timer_params))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ExecuteSkillGoal>) istream)
  "Deserializes a message object of type '<ExecuteSkillGoal>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'skill_type)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'skill_description) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'skill_description) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'meta_skill_type)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'meta_skill_id) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'sensor_topics) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'sensor_topics)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'sensor_value_sizes) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'sensor_value_sizes)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:aref vals i)) (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'initial_sensor_values) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'initial_sensor_values)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'trajectory_generator_type)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'trajectory_generator_param_data_size) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'trajectory_generator_param_data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'trajectory_generator_param_data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'feedback_controller_type)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'feedback_controller_param_data_size) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'feedback_controller_param_data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'feedback_controller_param_data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'termination_handler_type)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'termination_handler_param_data_size) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'termination_handler_param_data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'termination_handler_param_data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'timer_type)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num_timer_params) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'timer_params) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'timer_params)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ExecuteSkillGoal>)))
  "Returns string type for a message object of type '<ExecuteSkillGoal>"
  "franka_interface_msgs/ExecuteSkillGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExecuteSkillGoal)))
  "Returns string type for a message object of type 'ExecuteSkillGoal"
  "franka_interface_msgs/ExecuteSkillGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ExecuteSkillGoal>)))
  "Returns md5sum for a message object of type '<ExecuteSkillGoal>"
  "55e633f630d923733f24dd64b606c67a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ExecuteSkillGoal)))
  "Returns md5sum for a message object of type 'ExecuteSkillGoal"
  "55e633f630d923733f24dd64b606c67a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ExecuteSkillGoal>)))
  "Returns full string definition for message of type '<ExecuteSkillGoal>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%uint8 skill_type~%string skill_description~%uint8 meta_skill_type~%int64 meta_skill_id~%~%# Sensor topic to subscribe to~%string[] sensor_topics~%uint64[] sensor_value_sizes~%float64[] initial_sensor_values~%~%# traj gen~%uint8 trajectory_generator_type~%int32 trajectory_generator_param_data_size~%uint8[] trajectory_generator_param_data~%~%# fbc~%uint8 feedback_controller_type~%int32 feedback_controller_param_data_size~%uint8[] feedback_controller_param_data~%~%# termination~%uint8 termination_handler_type~%int32 termination_handler_param_data_size~%uint8[] termination_handler_param_data ~%~%# timer~%uint8 timer_type~%int32 num_timer_params~%uint8[] timer_params~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ExecuteSkillGoal)))
  "Returns full string definition for message of type 'ExecuteSkillGoal"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%uint8 skill_type~%string skill_description~%uint8 meta_skill_type~%int64 meta_skill_id~%~%# Sensor topic to subscribe to~%string[] sensor_topics~%uint64[] sensor_value_sizes~%float64[] initial_sensor_values~%~%# traj gen~%uint8 trajectory_generator_type~%int32 trajectory_generator_param_data_size~%uint8[] trajectory_generator_param_data~%~%# fbc~%uint8 feedback_controller_type~%int32 feedback_controller_param_data_size~%uint8[] feedback_controller_param_data~%~%# termination~%uint8 termination_handler_type~%int32 termination_handler_param_data_size~%uint8[] termination_handler_param_data ~%~%# timer~%uint8 timer_type~%int32 num_timer_params~%uint8[] timer_params~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ExecuteSkillGoal>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'skill_description))
     1
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'sensor_topics) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'sensor_value_sizes) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'initial_sensor_values) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'trajectory_generator_param_data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     1
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'feedback_controller_param_data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     1
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'termination_handler_param_data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     1
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'timer_params) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ExecuteSkillGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'ExecuteSkillGoal
    (cl:cons ':skill_type (skill_type msg))
    (cl:cons ':skill_description (skill_description msg))
    (cl:cons ':meta_skill_type (meta_skill_type msg))
    (cl:cons ':meta_skill_id (meta_skill_id msg))
    (cl:cons ':sensor_topics (sensor_topics msg))
    (cl:cons ':sensor_value_sizes (sensor_value_sizes msg))
    (cl:cons ':initial_sensor_values (initial_sensor_values msg))
    (cl:cons ':trajectory_generator_type (trajectory_generator_type msg))
    (cl:cons ':trajectory_generator_param_data_size (trajectory_generator_param_data_size msg))
    (cl:cons ':trajectory_generator_param_data (trajectory_generator_param_data msg))
    (cl:cons ':feedback_controller_type (feedback_controller_type msg))
    (cl:cons ':feedback_controller_param_data_size (feedback_controller_param_data_size msg))
    (cl:cons ':feedback_controller_param_data (feedback_controller_param_data msg))
    (cl:cons ':termination_handler_type (termination_handler_type msg))
    (cl:cons ':termination_handler_param_data_size (termination_handler_param_data_size msg))
    (cl:cons ':termination_handler_param_data (termination_handler_param_data msg))
    (cl:cons ':timer_type (timer_type msg))
    (cl:cons ':num_timer_params (num_timer_params msg))
    (cl:cons ':timer_params (timer_params msg))
))

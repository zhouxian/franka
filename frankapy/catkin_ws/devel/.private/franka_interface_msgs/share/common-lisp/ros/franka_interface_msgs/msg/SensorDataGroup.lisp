; Auto-generated. Do not edit!


(cl:in-package franka_interface_msgs-msg)


;//! \htmlinclude SensorDataGroup.msg.html

(cl:defclass <SensorDataGroup> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (has_trajectory_generator_sensor_data
    :reader has_trajectory_generator_sensor_data
    :initarg :has_trajectory_generator_sensor_data
    :type cl:boolean
    :initform cl:nil)
   (has_feedback_controller_sensor_data
    :reader has_feedback_controller_sensor_data
    :initarg :has_feedback_controller_sensor_data
    :type cl:boolean
    :initform cl:nil)
   (has_termination_handler_sensor_data
    :reader has_termination_handler_sensor_data
    :initarg :has_termination_handler_sensor_data
    :type cl:boolean
    :initform cl:nil)
   (trajectoryGeneratorSensorData
    :reader trajectoryGeneratorSensorData
    :initarg :trajectoryGeneratorSensorData
    :type franka_interface_msgs-msg:SensorData
    :initform (cl:make-instance 'franka_interface_msgs-msg:SensorData))
   (feedbackControllerSensorData
    :reader feedbackControllerSensorData
    :initarg :feedbackControllerSensorData
    :type franka_interface_msgs-msg:SensorData
    :initform (cl:make-instance 'franka_interface_msgs-msg:SensorData))
   (terminationHandlerSensorData
    :reader terminationHandlerSensorData
    :initarg :terminationHandlerSensorData
    :type franka_interface_msgs-msg:SensorData
    :initform (cl:make-instance 'franka_interface_msgs-msg:SensorData)))
)

(cl:defclass SensorDataGroup (<SensorDataGroup>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SensorDataGroup>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SensorDataGroup)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name franka_interface_msgs-msg:<SensorDataGroup> is deprecated: use franka_interface_msgs-msg:SensorDataGroup instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SensorDataGroup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:header-val is deprecated.  Use franka_interface_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'has_trajectory_generator_sensor_data-val :lambda-list '(m))
(cl:defmethod has_trajectory_generator_sensor_data-val ((m <SensorDataGroup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:has_trajectory_generator_sensor_data-val is deprecated.  Use franka_interface_msgs-msg:has_trajectory_generator_sensor_data instead.")
  (has_trajectory_generator_sensor_data m))

(cl:ensure-generic-function 'has_feedback_controller_sensor_data-val :lambda-list '(m))
(cl:defmethod has_feedback_controller_sensor_data-val ((m <SensorDataGroup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:has_feedback_controller_sensor_data-val is deprecated.  Use franka_interface_msgs-msg:has_feedback_controller_sensor_data instead.")
  (has_feedback_controller_sensor_data m))

(cl:ensure-generic-function 'has_termination_handler_sensor_data-val :lambda-list '(m))
(cl:defmethod has_termination_handler_sensor_data-val ((m <SensorDataGroup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:has_termination_handler_sensor_data-val is deprecated.  Use franka_interface_msgs-msg:has_termination_handler_sensor_data instead.")
  (has_termination_handler_sensor_data m))

(cl:ensure-generic-function 'trajectoryGeneratorSensorData-val :lambda-list '(m))
(cl:defmethod trajectoryGeneratorSensorData-val ((m <SensorDataGroup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:trajectoryGeneratorSensorData-val is deprecated.  Use franka_interface_msgs-msg:trajectoryGeneratorSensorData instead.")
  (trajectoryGeneratorSensorData m))

(cl:ensure-generic-function 'feedbackControllerSensorData-val :lambda-list '(m))
(cl:defmethod feedbackControllerSensorData-val ((m <SensorDataGroup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:feedbackControllerSensorData-val is deprecated.  Use franka_interface_msgs-msg:feedbackControllerSensorData instead.")
  (feedbackControllerSensorData m))

(cl:ensure-generic-function 'terminationHandlerSensorData-val :lambda-list '(m))
(cl:defmethod terminationHandlerSensorData-val ((m <SensorDataGroup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:terminationHandlerSensorData-val is deprecated.  Use franka_interface_msgs-msg:terminationHandlerSensorData instead.")
  (terminationHandlerSensorData m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SensorDataGroup>) ostream)
  "Serializes a message object of type '<SensorDataGroup>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'has_trajectory_generator_sensor_data) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'has_feedback_controller_sensor_data) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'has_termination_handler_sensor_data) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'trajectoryGeneratorSensorData) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'feedbackControllerSensorData) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'terminationHandlerSensorData) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SensorDataGroup>) istream)
  "Deserializes a message object of type '<SensorDataGroup>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'has_trajectory_generator_sensor_data) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'has_feedback_controller_sensor_data) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'has_termination_handler_sensor_data) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'trajectoryGeneratorSensorData) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'feedbackControllerSensorData) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'terminationHandlerSensorData) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SensorDataGroup>)))
  "Returns string type for a message object of type '<SensorDataGroup>"
  "franka_interface_msgs/SensorDataGroup")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SensorDataGroup)))
  "Returns string type for a message object of type 'SensorDataGroup"
  "franka_interface_msgs/SensorDataGroup")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SensorDataGroup>)))
  "Returns md5sum for a message object of type '<SensorDataGroup>"
  "859c186ac0046509397a685ba0f2e8d2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SensorDataGroup)))
  "Returns md5sum for a message object of type 'SensorDataGroup"
  "859c186ac0046509397a685ba0f2e8d2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SensorDataGroup>)))
  "Returns full string definition for message of type '<SensorDataGroup>"
  (cl:format cl:nil "# SensorDataGroup~%std_msgs/Header header~%bool has_trajectory_generator_sensor_data~%bool has_feedback_controller_sensor_data~%bool has_termination_handler_sensor_data~%SensorData trajectoryGeneratorSensorData~%SensorData feedbackControllerSensorData~%SensorData terminationHandlerSensorData~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: franka_interface_msgs/SensorData~%# Sensor data~%std_msgs/Header header~%string info~%uint8 type~%int32 size~%uint8[] sensorData~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SensorDataGroup)))
  "Returns full string definition for message of type 'SensorDataGroup"
  (cl:format cl:nil "# SensorDataGroup~%std_msgs/Header header~%bool has_trajectory_generator_sensor_data~%bool has_feedback_controller_sensor_data~%bool has_termination_handler_sensor_data~%SensorData trajectoryGeneratorSensorData~%SensorData feedbackControllerSensorData~%SensorData terminationHandlerSensorData~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: franka_interface_msgs/SensorData~%# Sensor data~%std_msgs/Header header~%string info~%uint8 type~%int32 size~%uint8[] sensorData~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SensorDataGroup>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'trajectoryGeneratorSensorData))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'feedbackControllerSensorData))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'terminationHandlerSensorData))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SensorDataGroup>))
  "Converts a ROS message object to a list"
  (cl:list 'SensorDataGroup
    (cl:cons ':header (header msg))
    (cl:cons ':has_trajectory_generator_sensor_data (has_trajectory_generator_sensor_data msg))
    (cl:cons ':has_feedback_controller_sensor_data (has_feedback_controller_sensor_data msg))
    (cl:cons ':has_termination_handler_sensor_data (has_termination_handler_sensor_data msg))
    (cl:cons ':trajectoryGeneratorSensorData (trajectoryGeneratorSensorData msg))
    (cl:cons ':feedbackControllerSensorData (feedbackControllerSensorData msg))
    (cl:cons ':terminationHandlerSensorData (terminationHandlerSensorData msg))
))

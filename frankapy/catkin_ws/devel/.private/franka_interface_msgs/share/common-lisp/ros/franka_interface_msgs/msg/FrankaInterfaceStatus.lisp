; Auto-generated. Do not edit!


(cl:in-package franka_interface_msgs-msg)


;//! \htmlinclude FrankaInterfaceStatus.msg.html

(cl:defclass <FrankaInterfaceStatus> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (is_ready
    :reader is_ready
    :initarg :is_ready
    :type cl:boolean
    :initform cl:nil)
   (error_description
    :reader error_description
    :initarg :error_description
    :type cl:string
    :initform "")
   (is_fresh
    :reader is_fresh
    :initarg :is_fresh
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass FrankaInterfaceStatus (<FrankaInterfaceStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FrankaInterfaceStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FrankaInterfaceStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name franka_interface_msgs-msg:<FrankaInterfaceStatus> is deprecated: use franka_interface_msgs-msg:FrankaInterfaceStatus instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <FrankaInterfaceStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:header-val is deprecated.  Use franka_interface_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'is_ready-val :lambda-list '(m))
(cl:defmethod is_ready-val ((m <FrankaInterfaceStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:is_ready-val is deprecated.  Use franka_interface_msgs-msg:is_ready instead.")
  (is_ready m))

(cl:ensure-generic-function 'error_description-val :lambda-list '(m))
(cl:defmethod error_description-val ((m <FrankaInterfaceStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:error_description-val is deprecated.  Use franka_interface_msgs-msg:error_description instead.")
  (error_description m))

(cl:ensure-generic-function 'is_fresh-val :lambda-list '(m))
(cl:defmethod is_fresh-val ((m <FrankaInterfaceStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:is_fresh-val is deprecated.  Use franka_interface_msgs-msg:is_fresh instead.")
  (is_fresh m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FrankaInterfaceStatus>) ostream)
  "Serializes a message object of type '<FrankaInterfaceStatus>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_ready) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'error_description))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'error_description))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_fresh) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FrankaInterfaceStatus>) istream)
  "Deserializes a message object of type '<FrankaInterfaceStatus>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'is_ready) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'error_description) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'error_description) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'is_fresh) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FrankaInterfaceStatus>)))
  "Returns string type for a message object of type '<FrankaInterfaceStatus>"
  "franka_interface_msgs/FrankaInterfaceStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FrankaInterfaceStatus)))
  "Returns string type for a message object of type 'FrankaInterfaceStatus"
  "franka_interface_msgs/FrankaInterfaceStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FrankaInterfaceStatus>)))
  "Returns md5sum for a message object of type '<FrankaInterfaceStatus>"
  "b3b55f2e82de58761423a010b7b229a0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FrankaInterfaceStatus)))
  "Returns md5sum for a message object of type 'FrankaInterfaceStatus"
  "b3b55f2e82de58761423a010b7b229a0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FrankaInterfaceStatus>)))
  "Returns full string definition for message of type '<FrankaInterfaceStatus>"
  (cl:format cl:nil "# Franka robot state~%std_msgs/Header header~%bool is_ready~%string error_description~%bool is_fresh~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FrankaInterfaceStatus)))
  "Returns full string definition for message of type 'FrankaInterfaceStatus"
  (cl:format cl:nil "# Franka robot state~%std_msgs/Header header~%bool is_ready~%string error_description~%bool is_fresh~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FrankaInterfaceStatus>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     4 (cl:length (cl:slot-value msg 'error_description))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FrankaInterfaceStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'FrankaInterfaceStatus
    (cl:cons ':header (header msg))
    (cl:cons ':is_ready (is_ready msg))
    (cl:cons ':error_description (error_description msg))
    (cl:cons ':is_fresh (is_fresh msg))
))

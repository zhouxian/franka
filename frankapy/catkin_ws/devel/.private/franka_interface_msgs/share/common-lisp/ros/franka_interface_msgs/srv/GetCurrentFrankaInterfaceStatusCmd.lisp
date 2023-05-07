; Auto-generated. Do not edit!


(cl:in-package franka_interface_msgs-srv)


;//! \htmlinclude GetCurrentFrankaInterfaceStatusCmd-request.msg.html

(cl:defclass <GetCurrentFrankaInterfaceStatusCmd-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetCurrentFrankaInterfaceStatusCmd-request (<GetCurrentFrankaInterfaceStatusCmd-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetCurrentFrankaInterfaceStatusCmd-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetCurrentFrankaInterfaceStatusCmd-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name franka_interface_msgs-srv:<GetCurrentFrankaInterfaceStatusCmd-request> is deprecated: use franka_interface_msgs-srv:GetCurrentFrankaInterfaceStatusCmd-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetCurrentFrankaInterfaceStatusCmd-request>) ostream)
  "Serializes a message object of type '<GetCurrentFrankaInterfaceStatusCmd-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetCurrentFrankaInterfaceStatusCmd-request>) istream)
  "Deserializes a message object of type '<GetCurrentFrankaInterfaceStatusCmd-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetCurrentFrankaInterfaceStatusCmd-request>)))
  "Returns string type for a service object of type '<GetCurrentFrankaInterfaceStatusCmd-request>"
  "franka_interface_msgs/GetCurrentFrankaInterfaceStatusCmdRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCurrentFrankaInterfaceStatusCmd-request)))
  "Returns string type for a service object of type 'GetCurrentFrankaInterfaceStatusCmd-request"
  "franka_interface_msgs/GetCurrentFrankaInterfaceStatusCmdRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetCurrentFrankaInterfaceStatusCmd-request>)))
  "Returns md5sum for a message object of type '<GetCurrentFrankaInterfaceStatusCmd-request>"
  "1d3fe55393993751ebdd77c7f87c1d1e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetCurrentFrankaInterfaceStatusCmd-request)))
  "Returns md5sum for a message object of type 'GetCurrentFrankaInterfaceStatusCmd-request"
  "1d3fe55393993751ebdd77c7f87c1d1e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetCurrentFrankaInterfaceStatusCmd-request>)))
  "Returns full string definition for message of type '<GetCurrentFrankaInterfaceStatusCmd-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetCurrentFrankaInterfaceStatusCmd-request)))
  "Returns full string definition for message of type 'GetCurrentFrankaInterfaceStatusCmd-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetCurrentFrankaInterfaceStatusCmd-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetCurrentFrankaInterfaceStatusCmd-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetCurrentFrankaInterfaceStatusCmd-request
))
;//! \htmlinclude GetCurrentFrankaInterfaceStatusCmd-response.msg.html

(cl:defclass <GetCurrentFrankaInterfaceStatusCmd-response> (roslisp-msg-protocol:ros-message)
  ((franka_interface_status
    :reader franka_interface_status
    :initarg :franka_interface_status
    :type franka_interface_msgs-msg:FrankaInterfaceStatus
    :initform (cl:make-instance 'franka_interface_msgs-msg:FrankaInterfaceStatus)))
)

(cl:defclass GetCurrentFrankaInterfaceStatusCmd-response (<GetCurrentFrankaInterfaceStatusCmd-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetCurrentFrankaInterfaceStatusCmd-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetCurrentFrankaInterfaceStatusCmd-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name franka_interface_msgs-srv:<GetCurrentFrankaInterfaceStatusCmd-response> is deprecated: use franka_interface_msgs-srv:GetCurrentFrankaInterfaceStatusCmd-response instead.")))

(cl:ensure-generic-function 'franka_interface_status-val :lambda-list '(m))
(cl:defmethod franka_interface_status-val ((m <GetCurrentFrankaInterfaceStatusCmd-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-srv:franka_interface_status-val is deprecated.  Use franka_interface_msgs-srv:franka_interface_status instead.")
  (franka_interface_status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetCurrentFrankaInterfaceStatusCmd-response>) ostream)
  "Serializes a message object of type '<GetCurrentFrankaInterfaceStatusCmd-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'franka_interface_status) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetCurrentFrankaInterfaceStatusCmd-response>) istream)
  "Deserializes a message object of type '<GetCurrentFrankaInterfaceStatusCmd-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'franka_interface_status) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetCurrentFrankaInterfaceStatusCmd-response>)))
  "Returns string type for a service object of type '<GetCurrentFrankaInterfaceStatusCmd-response>"
  "franka_interface_msgs/GetCurrentFrankaInterfaceStatusCmdResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCurrentFrankaInterfaceStatusCmd-response)))
  "Returns string type for a service object of type 'GetCurrentFrankaInterfaceStatusCmd-response"
  "franka_interface_msgs/GetCurrentFrankaInterfaceStatusCmdResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetCurrentFrankaInterfaceStatusCmd-response>)))
  "Returns md5sum for a message object of type '<GetCurrentFrankaInterfaceStatusCmd-response>"
  "1d3fe55393993751ebdd77c7f87c1d1e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetCurrentFrankaInterfaceStatusCmd-response)))
  "Returns md5sum for a message object of type 'GetCurrentFrankaInterfaceStatusCmd-response"
  "1d3fe55393993751ebdd77c7f87c1d1e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetCurrentFrankaInterfaceStatusCmd-response>)))
  "Returns full string definition for message of type '<GetCurrentFrankaInterfaceStatusCmd-response>"
  (cl:format cl:nil "FrankaInterfaceStatus franka_interface_status~%~%================================================================================~%MSG: franka_interface_msgs/FrankaInterfaceStatus~%# Franka robot state~%std_msgs/Header header~%bool is_ready~%string error_description~%bool is_fresh~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetCurrentFrankaInterfaceStatusCmd-response)))
  "Returns full string definition for message of type 'GetCurrentFrankaInterfaceStatusCmd-response"
  (cl:format cl:nil "FrankaInterfaceStatus franka_interface_status~%~%================================================================================~%MSG: franka_interface_msgs/FrankaInterfaceStatus~%# Franka robot state~%std_msgs/Header header~%bool is_ready~%string error_description~%bool is_fresh~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetCurrentFrankaInterfaceStatusCmd-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'franka_interface_status))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetCurrentFrankaInterfaceStatusCmd-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetCurrentFrankaInterfaceStatusCmd-response
    (cl:cons ':franka_interface_status (franka_interface_status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetCurrentFrankaInterfaceStatusCmd)))
  'GetCurrentFrankaInterfaceStatusCmd-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetCurrentFrankaInterfaceStatusCmd)))
  'GetCurrentFrankaInterfaceStatusCmd-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCurrentFrankaInterfaceStatusCmd)))
  "Returns string type for a service object of type '<GetCurrentFrankaInterfaceStatusCmd>"
  "franka_interface_msgs/GetCurrentFrankaInterfaceStatusCmd")
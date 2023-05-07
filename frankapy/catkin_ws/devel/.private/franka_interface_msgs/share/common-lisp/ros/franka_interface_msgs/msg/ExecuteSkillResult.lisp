; Auto-generated. Do not edit!


(cl:in-package franka_interface_msgs-msg)


;//! \htmlinclude ExecuteSkillResult.msg.html

(cl:defclass <ExecuteSkillResult> (roslisp-msg-protocol:ros-message)
  ((num_execution_result
    :reader num_execution_result
    :initarg :num_execution_result
    :type cl:integer
    :initform 0)
   (execution_result
    :reader execution_result
    :initarg :execution_result
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass ExecuteSkillResult (<ExecuteSkillResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ExecuteSkillResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ExecuteSkillResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name franka_interface_msgs-msg:<ExecuteSkillResult> is deprecated: use franka_interface_msgs-msg:ExecuteSkillResult instead.")))

(cl:ensure-generic-function 'num_execution_result-val :lambda-list '(m))
(cl:defmethod num_execution_result-val ((m <ExecuteSkillResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:num_execution_result-val is deprecated.  Use franka_interface_msgs-msg:num_execution_result instead.")
  (num_execution_result m))

(cl:ensure-generic-function 'execution_result-val :lambda-list '(m))
(cl:defmethod execution_result-val ((m <ExecuteSkillResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:execution_result-val is deprecated.  Use franka_interface_msgs-msg:execution_result instead.")
  (execution_result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ExecuteSkillResult>) ostream)
  "Serializes a message object of type '<ExecuteSkillResult>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'num_execution_result)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'num_execution_result)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'num_execution_result)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'num_execution_result)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'num_execution_result)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'num_execution_result)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'num_execution_result)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'num_execution_result)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'execution_result))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'execution_result))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ExecuteSkillResult>) istream)
  "Deserializes a message object of type '<ExecuteSkillResult>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'num_execution_result)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'num_execution_result)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'num_execution_result)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'num_execution_result)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'num_execution_result)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'num_execution_result)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'num_execution_result)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'num_execution_result)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'execution_result) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'execution_result)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ExecuteSkillResult>)))
  "Returns string type for a message object of type '<ExecuteSkillResult>"
  "franka_interface_msgs/ExecuteSkillResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExecuteSkillResult)))
  "Returns string type for a message object of type 'ExecuteSkillResult"
  "franka_interface_msgs/ExecuteSkillResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ExecuteSkillResult>)))
  "Returns md5sum for a message object of type '<ExecuteSkillResult>"
  "0e5d113ae1195e7cbc619ec49d1ebaab")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ExecuteSkillResult)))
  "Returns md5sum for a message object of type 'ExecuteSkillResult"
  "0e5d113ae1195e7cbc619ec49d1ebaab")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ExecuteSkillResult>)))
  "Returns full string definition for message of type '<ExecuteSkillResult>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Result - happens when the skill terminates~%uint64 num_execution_result~%uint8[] execution_result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ExecuteSkillResult)))
  "Returns full string definition for message of type 'ExecuteSkillResult"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Result - happens when the skill terminates~%uint64 num_execution_result~%uint8[] execution_result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ExecuteSkillResult>))
  (cl:+ 0
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'execution_result) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ExecuteSkillResult>))
  "Converts a ROS message object to a list"
  (cl:list 'ExecuteSkillResult
    (cl:cons ':num_execution_result (num_execution_result msg))
    (cl:cons ':execution_result (execution_result msg))
))

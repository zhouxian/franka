; Auto-generated. Do not edit!


(cl:in-package franka_interface_msgs-msg)


;//! \htmlinclude ExecuteSkillFeedback.msg.html

(cl:defclass <ExecuteSkillFeedback> (roslisp-msg-protocol:ros-message)
  ((num_execution_feedback
    :reader num_execution_feedback
    :initarg :num_execution_feedback
    :type cl:integer
    :initform 0)
   (execution_feedback
    :reader execution_feedback
    :initarg :execution_feedback
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass ExecuteSkillFeedback (<ExecuteSkillFeedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ExecuteSkillFeedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ExecuteSkillFeedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name franka_interface_msgs-msg:<ExecuteSkillFeedback> is deprecated: use franka_interface_msgs-msg:ExecuteSkillFeedback instead.")))

(cl:ensure-generic-function 'num_execution_feedback-val :lambda-list '(m))
(cl:defmethod num_execution_feedback-val ((m <ExecuteSkillFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:num_execution_feedback-val is deprecated.  Use franka_interface_msgs-msg:num_execution_feedback instead.")
  (num_execution_feedback m))

(cl:ensure-generic-function 'execution_feedback-val :lambda-list '(m))
(cl:defmethod execution_feedback-val ((m <ExecuteSkillFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader franka_interface_msgs-msg:execution_feedback-val is deprecated.  Use franka_interface_msgs-msg:execution_feedback instead.")
  (execution_feedback m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ExecuteSkillFeedback>) ostream)
  "Serializes a message object of type '<ExecuteSkillFeedback>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'num_execution_feedback)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'num_execution_feedback)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'num_execution_feedback)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'num_execution_feedback)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'num_execution_feedback)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'num_execution_feedback)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'num_execution_feedback)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'num_execution_feedback)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'execution_feedback))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'execution_feedback))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ExecuteSkillFeedback>) istream)
  "Deserializes a message object of type '<ExecuteSkillFeedback>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'num_execution_feedback)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'num_execution_feedback)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'num_execution_feedback)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'num_execution_feedback)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 32) (cl:slot-value msg 'num_execution_feedback)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 40) (cl:slot-value msg 'num_execution_feedback)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 48) (cl:slot-value msg 'num_execution_feedback)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 56) (cl:slot-value msg 'num_execution_feedback)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'execution_feedback) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'execution_feedback)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ExecuteSkillFeedback>)))
  "Returns string type for a message object of type '<ExecuteSkillFeedback>"
  "franka_interface_msgs/ExecuteSkillFeedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExecuteSkillFeedback)))
  "Returns string type for a message object of type 'ExecuteSkillFeedback"
  "franka_interface_msgs/ExecuteSkillFeedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ExecuteSkillFeedback>)))
  "Returns md5sum for a message object of type '<ExecuteSkillFeedback>"
  "ff908ea97d128e4dfc430213d4a485e0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ExecuteSkillFeedback)))
  "Returns md5sum for a message object of type 'ExecuteSkillFeedback"
  "ff908ea97d128e4dfc430213d4a485e0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ExecuteSkillFeedback>)))
  "Returns full string definition for message of type '<ExecuteSkillFeedback>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Feedback message - happens during skill execution ~%uint64 num_execution_feedback~%uint8[] execution_feedback~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ExecuteSkillFeedback)))
  "Returns full string definition for message of type 'ExecuteSkillFeedback"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Feedback message - happens during skill execution ~%uint64 num_execution_feedback~%uint8[] execution_feedback~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ExecuteSkillFeedback>))
  (cl:+ 0
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'execution_feedback) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ExecuteSkillFeedback>))
  "Converts a ROS message object to a list"
  (cl:list 'ExecuteSkillFeedback
    (cl:cons ':num_execution_feedback (num_execution_feedback msg))
    (cl:cons ':execution_feedback (execution_feedback msg))
))

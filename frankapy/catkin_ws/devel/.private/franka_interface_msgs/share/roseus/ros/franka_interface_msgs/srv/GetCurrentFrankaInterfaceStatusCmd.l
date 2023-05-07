;; Auto-generated. Do not edit!


(when (boundp 'franka_interface_msgs::GetCurrentFrankaInterfaceStatusCmd)
  (if (not (find-package "FRANKA_INTERFACE_MSGS"))
    (make-package "FRANKA_INTERFACE_MSGS"))
  (shadow 'GetCurrentFrankaInterfaceStatusCmd (find-package "FRANKA_INTERFACE_MSGS")))
(unless (find-package "FRANKA_INTERFACE_MSGS::GETCURRENTFRANKAINTERFACESTATUSCMD")
  (make-package "FRANKA_INTERFACE_MSGS::GETCURRENTFRANKAINTERFACESTATUSCMD"))
(unless (find-package "FRANKA_INTERFACE_MSGS::GETCURRENTFRANKAINTERFACESTATUSCMDREQUEST")
  (make-package "FRANKA_INTERFACE_MSGS::GETCURRENTFRANKAINTERFACESTATUSCMDREQUEST"))
(unless (find-package "FRANKA_INTERFACE_MSGS::GETCURRENTFRANKAINTERFACESTATUSCMDRESPONSE")
  (make-package "FRANKA_INTERFACE_MSGS::GETCURRENTFRANKAINTERFACESTATUSCMDRESPONSE"))

(in-package "ROS")





(defclass franka_interface_msgs::GetCurrentFrankaInterfaceStatusCmdRequest
  :super ros::object
  :slots ())

(defmethod franka_interface_msgs::GetCurrentFrankaInterfaceStatusCmdRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass franka_interface_msgs::GetCurrentFrankaInterfaceStatusCmdResponse
  :super ros::object
  :slots (_franka_interface_status ))

(defmethod franka_interface_msgs::GetCurrentFrankaInterfaceStatusCmdResponse
  (:init
   (&key
    ((:franka_interface_status __franka_interface_status) (instance franka_interface_msgs::FrankaInterfaceStatus :init))
    )
   (send-super :init)
   (setq _franka_interface_status __franka_interface_status)
   self)
  (:franka_interface_status
   (&rest __franka_interface_status)
   (if (keywordp (car __franka_interface_status))
       (send* _franka_interface_status __franka_interface_status)
     (progn
       (if __franka_interface_status (setq _franka_interface_status (car __franka_interface_status)))
       _franka_interface_status)))
  (:serialization-length
   ()
   (+
    ;; franka_interface_msgs/FrankaInterfaceStatus _franka_interface_status
    (send _franka_interface_status :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; franka_interface_msgs/FrankaInterfaceStatus _franka_interface_status
       (send _franka_interface_status :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; franka_interface_msgs/FrankaInterfaceStatus _franka_interface_status
     (send _franka_interface_status :deserialize buf ptr-) (incf ptr- (send _franka_interface_status :serialization-length))
   ;;
   self)
  )

(defclass franka_interface_msgs::GetCurrentFrankaInterfaceStatusCmd
  :super ros::object
  :slots ())

(setf (get franka_interface_msgs::GetCurrentFrankaInterfaceStatusCmd :md5sum-) "1d3fe55393993751ebdd77c7f87c1d1e")
(setf (get franka_interface_msgs::GetCurrentFrankaInterfaceStatusCmd :datatype-) "franka_interface_msgs/GetCurrentFrankaInterfaceStatusCmd")
(setf (get franka_interface_msgs::GetCurrentFrankaInterfaceStatusCmd :request) franka_interface_msgs::GetCurrentFrankaInterfaceStatusCmdRequest)
(setf (get franka_interface_msgs::GetCurrentFrankaInterfaceStatusCmd :response) franka_interface_msgs::GetCurrentFrankaInterfaceStatusCmdResponse)

(defmethod franka_interface_msgs::GetCurrentFrankaInterfaceStatusCmdRequest
  (:response () (instance franka_interface_msgs::GetCurrentFrankaInterfaceStatusCmdResponse :init)))

(setf (get franka_interface_msgs::GetCurrentFrankaInterfaceStatusCmdRequest :md5sum-) "1d3fe55393993751ebdd77c7f87c1d1e")
(setf (get franka_interface_msgs::GetCurrentFrankaInterfaceStatusCmdRequest :datatype-) "franka_interface_msgs/GetCurrentFrankaInterfaceStatusCmdRequest")
(setf (get franka_interface_msgs::GetCurrentFrankaInterfaceStatusCmdRequest :definition-)
      "---
FrankaInterfaceStatus franka_interface_status

================================================================================
MSG: franka_interface_msgs/FrankaInterfaceStatus
# Franka robot state
std_msgs/Header header
bool is_ready
string error_description
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
")

(setf (get franka_interface_msgs::GetCurrentFrankaInterfaceStatusCmdResponse :md5sum-) "1d3fe55393993751ebdd77c7f87c1d1e")
(setf (get franka_interface_msgs::GetCurrentFrankaInterfaceStatusCmdResponse :datatype-) "franka_interface_msgs/GetCurrentFrankaInterfaceStatusCmdResponse")
(setf (get franka_interface_msgs::GetCurrentFrankaInterfaceStatusCmdResponse :definition-)
      "---
FrankaInterfaceStatus franka_interface_status

================================================================================
MSG: franka_interface_msgs/FrankaInterfaceStatus
# Franka robot state
std_msgs/Header header
bool is_ready
string error_description
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
")



(provide :franka_interface_msgs/GetCurrentFrankaInterfaceStatusCmd "1d3fe55393993751ebdd77c7f87c1d1e")



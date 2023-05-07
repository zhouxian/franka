;; Auto-generated. Do not edit!


(when (boundp 'franka_interface_msgs::FrankaInterfaceStatus)
  (if (not (find-package "FRANKA_INTERFACE_MSGS"))
    (make-package "FRANKA_INTERFACE_MSGS"))
  (shadow 'FrankaInterfaceStatus (find-package "FRANKA_INTERFACE_MSGS")))
(unless (find-package "FRANKA_INTERFACE_MSGS::FRANKAINTERFACESTATUS")
  (make-package "FRANKA_INTERFACE_MSGS::FRANKAINTERFACESTATUS"))

(in-package "ROS")
;;//! \htmlinclude FrankaInterfaceStatus.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass franka_interface_msgs::FrankaInterfaceStatus
  :super ros::object
  :slots (_header _is_ready _error_description _is_fresh ))

(defmethod franka_interface_msgs::FrankaInterfaceStatus
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:is_ready __is_ready) nil)
    ((:error_description __error_description) "")
    ((:is_fresh __is_fresh) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _is_ready __is_ready)
   (setq _error_description (string __error_description))
   (setq _is_fresh __is_fresh)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:is_ready
   (&optional (__is_ready :null))
   (if (not (eq __is_ready :null)) (setq _is_ready __is_ready)) _is_ready)
  (:error_description
   (&optional __error_description)
   (if __error_description (setq _error_description __error_description)) _error_description)
  (:is_fresh
   (&optional (__is_fresh :null))
   (if (not (eq __is_fresh :null)) (setq _is_fresh __is_fresh)) _is_fresh)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _is_ready
    1
    ;; string _error_description
    4 (length _error_description)
    ;; bool _is_fresh
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _is_ready
       (if _is_ready (write-byte -1 s) (write-byte 0 s))
     ;; string _error_description
       (write-long (length _error_description) s) (princ _error_description s)
     ;; bool _is_fresh
       (if _is_fresh (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _is_ready
     (setq _is_ready (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _error_description
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _error_description (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _is_fresh
     (setq _is_fresh (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get franka_interface_msgs::FrankaInterfaceStatus :md5sum-) "b3b55f2e82de58761423a010b7b229a0")
(setf (get franka_interface_msgs::FrankaInterfaceStatus :datatype-) "franka_interface_msgs/FrankaInterfaceStatus")
(setf (get franka_interface_msgs::FrankaInterfaceStatus :definition-)
      "# Franka robot state
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



(provide :franka_interface_msgs/FrankaInterfaceStatus "b3b55f2e82de58761423a010b7b229a0")



;; Auto-generated. Do not edit!


(when (boundp 'franka_interface_msgs::SensorData)
  (if (not (find-package "FRANKA_INTERFACE_MSGS"))
    (make-package "FRANKA_INTERFACE_MSGS"))
  (shadow 'SensorData (find-package "FRANKA_INTERFACE_MSGS")))
(unless (find-package "FRANKA_INTERFACE_MSGS::SENSORDATA")
  (make-package "FRANKA_INTERFACE_MSGS::SENSORDATA"))

(in-package "ROS")
;;//! \htmlinclude SensorData.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass franka_interface_msgs::SensorData
  :super ros::object
  :slots (_header _info _type _size _sensorData ))

(defmethod franka_interface_msgs::SensorData
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:info __info) "")
    ((:type __type) 0)
    ((:size __size) 0)
    ((:sensorData __sensorData) (make-array 0 :initial-element 0 :element-type :char))
    )
   (send-super :init)
   (setq _header __header)
   (setq _info (string __info))
   (setq _type (round __type))
   (setq _size (round __size))
   (setq _sensorData __sensorData)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:info
   (&optional __info)
   (if __info (setq _info __info)) _info)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:size
   (&optional __size)
   (if __size (setq _size __size)) _size)
  (:sensorData
   (&optional __sensorData)
   (if __sensorData (setq _sensorData __sensorData)) _sensorData)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _info
    4 (length _info)
    ;; uint8 _type
    1
    ;; int32 _size
    4
    ;; uint8[] _sensorData
    (* 1    (length _sensorData)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _info
       (write-long (length _info) s) (princ _info s)
     ;; uint8 _type
       (write-byte _type s)
     ;; int32 _size
       (write-long _size s)
     ;; uint8[] _sensorData
     (write-long (length _sensorData) s)
     (princ _sensorData s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _info
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _info (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint8 _type
     (setq _type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; int32 _size
     (setq _size (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8[] _sensorData
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _sensorData (make-array n :element-type :char))
     (replace _sensorData buf :start2 ptr-) (incf ptr- n))
   ;;
   self)
  )

(setf (get franka_interface_msgs::SensorData :md5sum-) "aaefdf50e303c513cc9b7bfed30bdd94")
(setf (get franka_interface_msgs::SensorData :datatype-) "franka_interface_msgs/SensorData")
(setf (get franka_interface_msgs::SensorData :definition-)
      "# Sensor data
std_msgs/Header header
string info
uint8 type
int32 size
uint8[] sensorData
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



(provide :franka_interface_msgs/SensorData "aaefdf50e303c513cc9b7bfed30bdd94")



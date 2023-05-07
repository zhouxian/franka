;; Auto-generated. Do not edit!


(when (boundp 'franka_interface_msgs::SensorDataGroup)
  (if (not (find-package "FRANKA_INTERFACE_MSGS"))
    (make-package "FRANKA_INTERFACE_MSGS"))
  (shadow 'SensorDataGroup (find-package "FRANKA_INTERFACE_MSGS")))
(unless (find-package "FRANKA_INTERFACE_MSGS::SENSORDATAGROUP")
  (make-package "FRANKA_INTERFACE_MSGS::SENSORDATAGROUP"))

(in-package "ROS")
;;//! \htmlinclude SensorDataGroup.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass franka_interface_msgs::SensorDataGroup
  :super ros::object
  :slots (_header _has_trajectory_generator_sensor_data _has_feedback_controller_sensor_data _has_termination_handler_sensor_data _trajectoryGeneratorSensorData _feedbackControllerSensorData _terminationHandlerSensorData ))

(defmethod franka_interface_msgs::SensorDataGroup
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:has_trajectory_generator_sensor_data __has_trajectory_generator_sensor_data) nil)
    ((:has_feedback_controller_sensor_data __has_feedback_controller_sensor_data) nil)
    ((:has_termination_handler_sensor_data __has_termination_handler_sensor_data) nil)
    ((:trajectoryGeneratorSensorData __trajectoryGeneratorSensorData) (instance franka_interface_msgs::SensorData :init))
    ((:feedbackControllerSensorData __feedbackControllerSensorData) (instance franka_interface_msgs::SensorData :init))
    ((:terminationHandlerSensorData __terminationHandlerSensorData) (instance franka_interface_msgs::SensorData :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _has_trajectory_generator_sensor_data __has_trajectory_generator_sensor_data)
   (setq _has_feedback_controller_sensor_data __has_feedback_controller_sensor_data)
   (setq _has_termination_handler_sensor_data __has_termination_handler_sensor_data)
   (setq _trajectoryGeneratorSensorData __trajectoryGeneratorSensorData)
   (setq _feedbackControllerSensorData __feedbackControllerSensorData)
   (setq _terminationHandlerSensorData __terminationHandlerSensorData)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:has_trajectory_generator_sensor_data
   (&optional (__has_trajectory_generator_sensor_data :null))
   (if (not (eq __has_trajectory_generator_sensor_data :null)) (setq _has_trajectory_generator_sensor_data __has_trajectory_generator_sensor_data)) _has_trajectory_generator_sensor_data)
  (:has_feedback_controller_sensor_data
   (&optional (__has_feedback_controller_sensor_data :null))
   (if (not (eq __has_feedback_controller_sensor_data :null)) (setq _has_feedback_controller_sensor_data __has_feedback_controller_sensor_data)) _has_feedback_controller_sensor_data)
  (:has_termination_handler_sensor_data
   (&optional (__has_termination_handler_sensor_data :null))
   (if (not (eq __has_termination_handler_sensor_data :null)) (setq _has_termination_handler_sensor_data __has_termination_handler_sensor_data)) _has_termination_handler_sensor_data)
  (:trajectoryGeneratorSensorData
   (&rest __trajectoryGeneratorSensorData)
   (if (keywordp (car __trajectoryGeneratorSensorData))
       (send* _trajectoryGeneratorSensorData __trajectoryGeneratorSensorData)
     (progn
       (if __trajectoryGeneratorSensorData (setq _trajectoryGeneratorSensorData (car __trajectoryGeneratorSensorData)))
       _trajectoryGeneratorSensorData)))
  (:feedbackControllerSensorData
   (&rest __feedbackControllerSensorData)
   (if (keywordp (car __feedbackControllerSensorData))
       (send* _feedbackControllerSensorData __feedbackControllerSensorData)
     (progn
       (if __feedbackControllerSensorData (setq _feedbackControllerSensorData (car __feedbackControllerSensorData)))
       _feedbackControllerSensorData)))
  (:terminationHandlerSensorData
   (&rest __terminationHandlerSensorData)
   (if (keywordp (car __terminationHandlerSensorData))
       (send* _terminationHandlerSensorData __terminationHandlerSensorData)
     (progn
       (if __terminationHandlerSensorData (setq _terminationHandlerSensorData (car __terminationHandlerSensorData)))
       _terminationHandlerSensorData)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _has_trajectory_generator_sensor_data
    1
    ;; bool _has_feedback_controller_sensor_data
    1
    ;; bool _has_termination_handler_sensor_data
    1
    ;; franka_interface_msgs/SensorData _trajectoryGeneratorSensorData
    (send _trajectoryGeneratorSensorData :serialization-length)
    ;; franka_interface_msgs/SensorData _feedbackControllerSensorData
    (send _feedbackControllerSensorData :serialization-length)
    ;; franka_interface_msgs/SensorData _terminationHandlerSensorData
    (send _terminationHandlerSensorData :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _has_trajectory_generator_sensor_data
       (if _has_trajectory_generator_sensor_data (write-byte -1 s) (write-byte 0 s))
     ;; bool _has_feedback_controller_sensor_data
       (if _has_feedback_controller_sensor_data (write-byte -1 s) (write-byte 0 s))
     ;; bool _has_termination_handler_sensor_data
       (if _has_termination_handler_sensor_data (write-byte -1 s) (write-byte 0 s))
     ;; franka_interface_msgs/SensorData _trajectoryGeneratorSensorData
       (send _trajectoryGeneratorSensorData :serialize s)
     ;; franka_interface_msgs/SensorData _feedbackControllerSensorData
       (send _feedbackControllerSensorData :serialize s)
     ;; franka_interface_msgs/SensorData _terminationHandlerSensorData
       (send _terminationHandlerSensorData :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _has_trajectory_generator_sensor_data
     (setq _has_trajectory_generator_sensor_data (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _has_feedback_controller_sensor_data
     (setq _has_feedback_controller_sensor_data (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _has_termination_handler_sensor_data
     (setq _has_termination_handler_sensor_data (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; franka_interface_msgs/SensorData _trajectoryGeneratorSensorData
     (send _trajectoryGeneratorSensorData :deserialize buf ptr-) (incf ptr- (send _trajectoryGeneratorSensorData :serialization-length))
   ;; franka_interface_msgs/SensorData _feedbackControllerSensorData
     (send _feedbackControllerSensorData :deserialize buf ptr-) (incf ptr- (send _feedbackControllerSensorData :serialization-length))
   ;; franka_interface_msgs/SensorData _terminationHandlerSensorData
     (send _terminationHandlerSensorData :deserialize buf ptr-) (incf ptr- (send _terminationHandlerSensorData :serialization-length))
   ;;
   self)
  )

(setf (get franka_interface_msgs::SensorDataGroup :md5sum-) "859c186ac0046509397a685ba0f2e8d2")
(setf (get franka_interface_msgs::SensorDataGroup :datatype-) "franka_interface_msgs/SensorDataGroup")
(setf (get franka_interface_msgs::SensorDataGroup :definition-)
      "# SensorDataGroup
std_msgs/Header header
bool has_trajectory_generator_sensor_data
bool has_feedback_controller_sensor_data
bool has_termination_handler_sensor_data
SensorData trajectoryGeneratorSensorData
SensorData feedbackControllerSensorData
SensorData terminationHandlerSensorData
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

================================================================================
MSG: franka_interface_msgs/SensorData
# Sensor data
std_msgs/Header header
string info
uint8 type
int32 size
uint8[] sensorData
")



(provide :franka_interface_msgs/SensorDataGroup "859c186ac0046509397a685ba0f2e8d2")



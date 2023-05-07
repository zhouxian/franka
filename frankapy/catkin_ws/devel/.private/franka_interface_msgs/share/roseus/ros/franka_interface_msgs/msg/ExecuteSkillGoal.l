;; Auto-generated. Do not edit!


(when (boundp 'franka_interface_msgs::ExecuteSkillGoal)
  (if (not (find-package "FRANKA_INTERFACE_MSGS"))
    (make-package "FRANKA_INTERFACE_MSGS"))
  (shadow 'ExecuteSkillGoal (find-package "FRANKA_INTERFACE_MSGS")))
(unless (find-package "FRANKA_INTERFACE_MSGS::EXECUTESKILLGOAL")
  (make-package "FRANKA_INTERFACE_MSGS::EXECUTESKILLGOAL"))

(in-package "ROS")
;;//! \htmlinclude ExecuteSkillGoal.msg.html


(defclass franka_interface_msgs::ExecuteSkillGoal
  :super ros::object
  :slots (_skill_type _skill_description _meta_skill_type _meta_skill_id _sensor_topics _sensor_value_sizes _initial_sensor_values _trajectory_generator_type _trajectory_generator_param_data_size _trajectory_generator_param_data _feedback_controller_type _feedback_controller_param_data_size _feedback_controller_param_data _termination_handler_type _termination_handler_param_data_size _termination_handler_param_data _timer_type _num_timer_params _timer_params ))

(defmethod franka_interface_msgs::ExecuteSkillGoal
  (:init
   (&key
    ((:skill_type __skill_type) 0)
    ((:skill_description __skill_description) "")
    ((:meta_skill_type __meta_skill_type) 0)
    ((:meta_skill_id __meta_skill_id) 0)
    ((:sensor_topics __sensor_topics) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:sensor_value_sizes __sensor_value_sizes) (make-array 0 :initial-element 0 :element-type :integer))
    ((:initial_sensor_values __initial_sensor_values) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:trajectory_generator_type __trajectory_generator_type) 0)
    ((:trajectory_generator_param_data_size __trajectory_generator_param_data_size) 0)
    ((:trajectory_generator_param_data __trajectory_generator_param_data) (make-array 0 :initial-element 0 :element-type :char))
    ((:feedback_controller_type __feedback_controller_type) 0)
    ((:feedback_controller_param_data_size __feedback_controller_param_data_size) 0)
    ((:feedback_controller_param_data __feedback_controller_param_data) (make-array 0 :initial-element 0 :element-type :char))
    ((:termination_handler_type __termination_handler_type) 0)
    ((:termination_handler_param_data_size __termination_handler_param_data_size) 0)
    ((:termination_handler_param_data __termination_handler_param_data) (make-array 0 :initial-element 0 :element-type :char))
    ((:timer_type __timer_type) 0)
    ((:num_timer_params __num_timer_params) 0)
    ((:timer_params __timer_params) (make-array 0 :initial-element 0 :element-type :char))
    )
   (send-super :init)
   (setq _skill_type (round __skill_type))
   (setq _skill_description (string __skill_description))
   (setq _meta_skill_type (round __meta_skill_type))
   (setq _meta_skill_id (round __meta_skill_id))
   (setq _sensor_topics __sensor_topics)
   (setq _sensor_value_sizes __sensor_value_sizes)
   (setq _initial_sensor_values __initial_sensor_values)
   (setq _trajectory_generator_type (round __trajectory_generator_type))
   (setq _trajectory_generator_param_data_size (round __trajectory_generator_param_data_size))
   (setq _trajectory_generator_param_data __trajectory_generator_param_data)
   (setq _feedback_controller_type (round __feedback_controller_type))
   (setq _feedback_controller_param_data_size (round __feedback_controller_param_data_size))
   (setq _feedback_controller_param_data __feedback_controller_param_data)
   (setq _termination_handler_type (round __termination_handler_type))
   (setq _termination_handler_param_data_size (round __termination_handler_param_data_size))
   (setq _termination_handler_param_data __termination_handler_param_data)
   (setq _timer_type (round __timer_type))
   (setq _num_timer_params (round __num_timer_params))
   (setq _timer_params __timer_params)
   self)
  (:skill_type
   (&optional __skill_type)
   (if __skill_type (setq _skill_type __skill_type)) _skill_type)
  (:skill_description
   (&optional __skill_description)
   (if __skill_description (setq _skill_description __skill_description)) _skill_description)
  (:meta_skill_type
   (&optional __meta_skill_type)
   (if __meta_skill_type (setq _meta_skill_type __meta_skill_type)) _meta_skill_type)
  (:meta_skill_id
   (&optional __meta_skill_id)
   (if __meta_skill_id (setq _meta_skill_id __meta_skill_id)) _meta_skill_id)
  (:sensor_topics
   (&optional __sensor_topics)
   (if __sensor_topics (setq _sensor_topics __sensor_topics)) _sensor_topics)
  (:sensor_value_sizes
   (&optional __sensor_value_sizes)
   (if __sensor_value_sizes (setq _sensor_value_sizes __sensor_value_sizes)) _sensor_value_sizes)
  (:initial_sensor_values
   (&optional __initial_sensor_values)
   (if __initial_sensor_values (setq _initial_sensor_values __initial_sensor_values)) _initial_sensor_values)
  (:trajectory_generator_type
   (&optional __trajectory_generator_type)
   (if __trajectory_generator_type (setq _trajectory_generator_type __trajectory_generator_type)) _trajectory_generator_type)
  (:trajectory_generator_param_data_size
   (&optional __trajectory_generator_param_data_size)
   (if __trajectory_generator_param_data_size (setq _trajectory_generator_param_data_size __trajectory_generator_param_data_size)) _trajectory_generator_param_data_size)
  (:trajectory_generator_param_data
   (&optional __trajectory_generator_param_data)
   (if __trajectory_generator_param_data (setq _trajectory_generator_param_data __trajectory_generator_param_data)) _trajectory_generator_param_data)
  (:feedback_controller_type
   (&optional __feedback_controller_type)
   (if __feedback_controller_type (setq _feedback_controller_type __feedback_controller_type)) _feedback_controller_type)
  (:feedback_controller_param_data_size
   (&optional __feedback_controller_param_data_size)
   (if __feedback_controller_param_data_size (setq _feedback_controller_param_data_size __feedback_controller_param_data_size)) _feedback_controller_param_data_size)
  (:feedback_controller_param_data
   (&optional __feedback_controller_param_data)
   (if __feedback_controller_param_data (setq _feedback_controller_param_data __feedback_controller_param_data)) _feedback_controller_param_data)
  (:termination_handler_type
   (&optional __termination_handler_type)
   (if __termination_handler_type (setq _termination_handler_type __termination_handler_type)) _termination_handler_type)
  (:termination_handler_param_data_size
   (&optional __termination_handler_param_data_size)
   (if __termination_handler_param_data_size (setq _termination_handler_param_data_size __termination_handler_param_data_size)) _termination_handler_param_data_size)
  (:termination_handler_param_data
   (&optional __termination_handler_param_data)
   (if __termination_handler_param_data (setq _termination_handler_param_data __termination_handler_param_data)) _termination_handler_param_data)
  (:timer_type
   (&optional __timer_type)
   (if __timer_type (setq _timer_type __timer_type)) _timer_type)
  (:num_timer_params
   (&optional __num_timer_params)
   (if __num_timer_params (setq _num_timer_params __num_timer_params)) _num_timer_params)
  (:timer_params
   (&optional __timer_params)
   (if __timer_params (setq _timer_params __timer_params)) _timer_params)
  (:serialization-length
   ()
   (+
    ;; uint8 _skill_type
    1
    ;; string _skill_description
    4 (length _skill_description)
    ;; uint8 _meta_skill_type
    1
    ;; int64 _meta_skill_id
    8
    ;; string[] _sensor_topics
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _sensor_topics)) 4
    ;; uint64[] _sensor_value_sizes
    (* 8    (length _sensor_value_sizes)) 4
    ;; float64[] _initial_sensor_values
    (* 8    (length _initial_sensor_values)) 4
    ;; uint8 _trajectory_generator_type
    1
    ;; int32 _trajectory_generator_param_data_size
    4
    ;; uint8[] _trajectory_generator_param_data
    (* 1    (length _trajectory_generator_param_data)) 4
    ;; uint8 _feedback_controller_type
    1
    ;; int32 _feedback_controller_param_data_size
    4
    ;; uint8[] _feedback_controller_param_data
    (* 1    (length _feedback_controller_param_data)) 4
    ;; uint8 _termination_handler_type
    1
    ;; int32 _termination_handler_param_data_size
    4
    ;; uint8[] _termination_handler_param_data
    (* 1    (length _termination_handler_param_data)) 4
    ;; uint8 _timer_type
    1
    ;; int32 _num_timer_params
    4
    ;; uint8[] _timer_params
    (* 1    (length _timer_params)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _skill_type
       (write-byte _skill_type s)
     ;; string _skill_description
       (write-long (length _skill_description) s) (princ _skill_description s)
     ;; uint8 _meta_skill_type
       (write-byte _meta_skill_type s)
     ;; int64 _meta_skill_id
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _meta_skill_id (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _meta_skill_id) (= (length (_meta_skill_id . bv)) 2)) ;; bignum
              (write-long (ash (elt (_meta_skill_id . bv) 0) 0) s)
              (write-long (ash (elt (_meta_skill_id . bv) 1) -1) s))
             ((and (class _meta_skill_id) (= (length (_meta_skill_id . bv)) 1)) ;; big1
              (write-long (elt (_meta_skill_id . bv) 0) s)
              (write-long (if (>= _meta_skill_id 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _meta_skill_id s)(write-long (if (>= _meta_skill_id 0) 0 #xffffffff) s)))
     ;; string[] _sensor_topics
     (write-long (length _sensor_topics) s)
     (dolist (elem _sensor_topics)
       (write-long (length elem) s) (princ elem s)
       )
     ;; uint64[] _sensor_value_sizes
     (write-long (length _sensor_value_sizes) s)
     (dotimes (i (length _sensor_value_sizes))
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke (elt _sensor_value_sizes i) (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class (elt _sensor_value_sizes i)) (= (length ((elt _sensor_value_sizes i) . bv)) 2)) ;; bignum
              (write-long (ash (elt ((elt _sensor_value_sizes i) . bv) 0) 0) s)
              (write-long (ash (elt ((elt _sensor_value_sizes i) . bv) 1) -1) s))
             ((and (class (elt _sensor_value_sizes i)) (= (length ((elt _sensor_value_sizes i) . bv)) 1)) ;; big1
              (write-long (elt ((elt _sensor_value_sizes i) . bv) 0) s)
              (write-long (if (>= (elt _sensor_value_sizes i) 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long (elt _sensor_value_sizes i) s)(write-long (if (>= (elt _sensor_value_sizes i) 0) 0 #xffffffff) s)))
       )
     ;; float64[] _initial_sensor_values
     (write-long (length _initial_sensor_values) s)
     (dotimes (i (length _initial_sensor_values))
       (sys::poke (elt _initial_sensor_values i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; uint8 _trajectory_generator_type
       (write-byte _trajectory_generator_type s)
     ;; int32 _trajectory_generator_param_data_size
       (write-long _trajectory_generator_param_data_size s)
     ;; uint8[] _trajectory_generator_param_data
     (write-long (length _trajectory_generator_param_data) s)
     (princ _trajectory_generator_param_data s)
     ;; uint8 _feedback_controller_type
       (write-byte _feedback_controller_type s)
     ;; int32 _feedback_controller_param_data_size
       (write-long _feedback_controller_param_data_size s)
     ;; uint8[] _feedback_controller_param_data
     (write-long (length _feedback_controller_param_data) s)
     (princ _feedback_controller_param_data s)
     ;; uint8 _termination_handler_type
       (write-byte _termination_handler_type s)
     ;; int32 _termination_handler_param_data_size
       (write-long _termination_handler_param_data_size s)
     ;; uint8[] _termination_handler_param_data
     (write-long (length _termination_handler_param_data) s)
     (princ _termination_handler_param_data s)
     ;; uint8 _timer_type
       (write-byte _timer_type s)
     ;; int32 _num_timer_params
       (write-long _num_timer_params s)
     ;; uint8[] _timer_params
     (write-long (length _timer_params) s)
     (princ _timer_params s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _skill_type
     (setq _skill_type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; string _skill_description
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _skill_description (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint8 _meta_skill_type
     (setq _meta_skill_type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; int64 _meta_skill_id
#+(or :alpha :irix6 :x86_64)
      (setf _meta_skill_id (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _meta_skill_id (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; string[] _sensor_topics
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _sensor_topics (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _sensor_topics i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; uint64[] _sensor_value_sizes
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _sensor_value_sizes (instantiate integer-vector n))
     (dotimes (i n)
     
#+(or :alpha :irix6 :x86_64)
      (setf (elt _sensor_value_sizes i) (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf (elt _sensor_value_sizes i) (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
     ))
   ;; float64[] _initial_sensor_values
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _initial_sensor_values (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _initial_sensor_values i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; uint8 _trajectory_generator_type
     (setq _trajectory_generator_type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; int32 _trajectory_generator_param_data_size
     (setq _trajectory_generator_param_data_size (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8[] _trajectory_generator_param_data
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _trajectory_generator_param_data (make-array n :element-type :char))
     (replace _trajectory_generator_param_data buf :start2 ptr-) (incf ptr- n))
   ;; uint8 _feedback_controller_type
     (setq _feedback_controller_type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; int32 _feedback_controller_param_data_size
     (setq _feedback_controller_param_data_size (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8[] _feedback_controller_param_data
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _feedback_controller_param_data (make-array n :element-type :char))
     (replace _feedback_controller_param_data buf :start2 ptr-) (incf ptr- n))
   ;; uint8 _termination_handler_type
     (setq _termination_handler_type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; int32 _termination_handler_param_data_size
     (setq _termination_handler_param_data_size (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8[] _termination_handler_param_data
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _termination_handler_param_data (make-array n :element-type :char))
     (replace _termination_handler_param_data buf :start2 ptr-) (incf ptr- n))
   ;; uint8 _timer_type
     (setq _timer_type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; int32 _num_timer_params
     (setq _num_timer_params (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8[] _timer_params
   (let ((n (sys::peek buf ptr- :integer))) (incf ptr- 4)
     (setq _timer_params (make-array n :element-type :char))
     (replace _timer_params buf :start2 ptr-) (incf ptr- n))
   ;;
   self)
  )

(setf (get franka_interface_msgs::ExecuteSkillGoal :md5sum-) "55e633f630d923733f24dd64b606c67a")
(setf (get franka_interface_msgs::ExecuteSkillGoal :datatype-) "franka_interface_msgs/ExecuteSkillGoal")
(setf (get franka_interface_msgs::ExecuteSkillGoal :definition-)
      "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
uint8 skill_type
string skill_description
uint8 meta_skill_type
int64 meta_skill_id

# Sensor topic to subscribe to
string[] sensor_topics
uint64[] sensor_value_sizes
float64[] initial_sensor_values

# traj gen
uint8 trajectory_generator_type
int32 trajectory_generator_param_data_size
uint8[] trajectory_generator_param_data

# fbc
uint8 feedback_controller_type
int32 feedback_controller_param_data_size
uint8[] feedback_controller_param_data

# termination
uint8 termination_handler_type
int32 termination_handler_param_data_size
uint8[] termination_handler_param_data 

# timer
uint8 timer_type
int32 num_timer_params
uint8[] timer_params

")



(provide :franka_interface_msgs/ExecuteSkillGoal "55e633f630d923733f24dd64b606c67a")



;; Auto-generated. Do not edit!


(when (boundp 'franka_interface_msgs::RunLoopProcessInfoState)
  (if (not (find-package "FRANKA_INTERFACE_MSGS"))
    (make-package "FRANKA_INTERFACE_MSGS"))
  (shadow 'RunLoopProcessInfoState (find-package "FRANKA_INTERFACE_MSGS")))
(unless (find-package "FRANKA_INTERFACE_MSGS::RUNLOOPPROCESSINFOSTATE")
  (make-package "FRANKA_INTERFACE_MSGS::RUNLOOPPROCESSINFOSTATE"))

(in-package "ROS")
;;//! \htmlinclude RunLoopProcessInfoState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass franka_interface_msgs::RunLoopProcessInfoState
  :super ros::object
  :slots (_header _current_memory_region _current_sensor_region _current_feedback_region _current_skill_id _current_skill_type _current_meta_skill_id _current_meta_skill_type _current_skill_description _new_skill_available _new_skill_id _new_skill_type _new_meta_skill_id _new_meta_skill_type _new_skill_description _is_running_skill _skill_preempted _done_skill_id _result_skill_id _time_skill_started_in_robot_time _time_skill_finished_in_robot_time _is_fresh ))

(defmethod franka_interface_msgs::RunLoopProcessInfoState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:current_memory_region __current_memory_region) 0)
    ((:current_sensor_region __current_sensor_region) 0)
    ((:current_feedback_region __current_feedback_region) 0)
    ((:current_skill_id __current_skill_id) 0)
    ((:current_skill_type __current_skill_type) 0)
    ((:current_meta_skill_id __current_meta_skill_id) 0)
    ((:current_meta_skill_type __current_meta_skill_type) 0)
    ((:current_skill_description __current_skill_description) "")
    ((:new_skill_available __new_skill_available) nil)
    ((:new_skill_id __new_skill_id) 0)
    ((:new_skill_type __new_skill_type) 0)
    ((:new_meta_skill_id __new_meta_skill_id) 0)
    ((:new_meta_skill_type __new_meta_skill_type) 0)
    ((:new_skill_description __new_skill_description) "")
    ((:is_running_skill __is_running_skill) nil)
    ((:skill_preempted __skill_preempted) nil)
    ((:done_skill_id __done_skill_id) 0)
    ((:result_skill_id __result_skill_id) 0)
    ((:time_skill_started_in_robot_time __time_skill_started_in_robot_time) 0.0)
    ((:time_skill_finished_in_robot_time __time_skill_finished_in_robot_time) 0.0)
    ((:is_fresh __is_fresh) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _current_memory_region (round __current_memory_region))
   (setq _current_sensor_region (round __current_sensor_region))
   (setq _current_feedback_region (round __current_feedback_region))
   (setq _current_skill_id (round __current_skill_id))
   (setq _current_skill_type (round __current_skill_type))
   (setq _current_meta_skill_id (round __current_meta_skill_id))
   (setq _current_meta_skill_type (round __current_meta_skill_type))
   (setq _current_skill_description (string __current_skill_description))
   (setq _new_skill_available __new_skill_available)
   (setq _new_skill_id (round __new_skill_id))
   (setq _new_skill_type (round __new_skill_type))
   (setq _new_meta_skill_id (round __new_meta_skill_id))
   (setq _new_meta_skill_type (round __new_meta_skill_type))
   (setq _new_skill_description (string __new_skill_description))
   (setq _is_running_skill __is_running_skill)
   (setq _skill_preempted __skill_preempted)
   (setq _done_skill_id (round __done_skill_id))
   (setq _result_skill_id (round __result_skill_id))
   (setq _time_skill_started_in_robot_time (float __time_skill_started_in_robot_time))
   (setq _time_skill_finished_in_robot_time (float __time_skill_finished_in_robot_time))
   (setq _is_fresh __is_fresh)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:current_memory_region
   (&optional __current_memory_region)
   (if __current_memory_region (setq _current_memory_region __current_memory_region)) _current_memory_region)
  (:current_sensor_region
   (&optional __current_sensor_region)
   (if __current_sensor_region (setq _current_sensor_region __current_sensor_region)) _current_sensor_region)
  (:current_feedback_region
   (&optional __current_feedback_region)
   (if __current_feedback_region (setq _current_feedback_region __current_feedback_region)) _current_feedback_region)
  (:current_skill_id
   (&optional __current_skill_id)
   (if __current_skill_id (setq _current_skill_id __current_skill_id)) _current_skill_id)
  (:current_skill_type
   (&optional __current_skill_type)
   (if __current_skill_type (setq _current_skill_type __current_skill_type)) _current_skill_type)
  (:current_meta_skill_id
   (&optional __current_meta_skill_id)
   (if __current_meta_skill_id (setq _current_meta_skill_id __current_meta_skill_id)) _current_meta_skill_id)
  (:current_meta_skill_type
   (&optional __current_meta_skill_type)
   (if __current_meta_skill_type (setq _current_meta_skill_type __current_meta_skill_type)) _current_meta_skill_type)
  (:current_skill_description
   (&optional __current_skill_description)
   (if __current_skill_description (setq _current_skill_description __current_skill_description)) _current_skill_description)
  (:new_skill_available
   (&optional (__new_skill_available :null))
   (if (not (eq __new_skill_available :null)) (setq _new_skill_available __new_skill_available)) _new_skill_available)
  (:new_skill_id
   (&optional __new_skill_id)
   (if __new_skill_id (setq _new_skill_id __new_skill_id)) _new_skill_id)
  (:new_skill_type
   (&optional __new_skill_type)
   (if __new_skill_type (setq _new_skill_type __new_skill_type)) _new_skill_type)
  (:new_meta_skill_id
   (&optional __new_meta_skill_id)
   (if __new_meta_skill_id (setq _new_meta_skill_id __new_meta_skill_id)) _new_meta_skill_id)
  (:new_meta_skill_type
   (&optional __new_meta_skill_type)
   (if __new_meta_skill_type (setq _new_meta_skill_type __new_meta_skill_type)) _new_meta_skill_type)
  (:new_skill_description
   (&optional __new_skill_description)
   (if __new_skill_description (setq _new_skill_description __new_skill_description)) _new_skill_description)
  (:is_running_skill
   (&optional (__is_running_skill :null))
   (if (not (eq __is_running_skill :null)) (setq _is_running_skill __is_running_skill)) _is_running_skill)
  (:skill_preempted
   (&optional (__skill_preempted :null))
   (if (not (eq __skill_preempted :null)) (setq _skill_preempted __skill_preempted)) _skill_preempted)
  (:done_skill_id
   (&optional __done_skill_id)
   (if __done_skill_id (setq _done_skill_id __done_skill_id)) _done_skill_id)
  (:result_skill_id
   (&optional __result_skill_id)
   (if __result_skill_id (setq _result_skill_id __result_skill_id)) _result_skill_id)
  (:time_skill_started_in_robot_time
   (&optional __time_skill_started_in_robot_time)
   (if __time_skill_started_in_robot_time (setq _time_skill_started_in_robot_time __time_skill_started_in_robot_time)) _time_skill_started_in_robot_time)
  (:time_skill_finished_in_robot_time
   (&optional __time_skill_finished_in_robot_time)
   (if __time_skill_finished_in_robot_time (setq _time_skill_finished_in_robot_time __time_skill_finished_in_robot_time)) _time_skill_finished_in_robot_time)
  (:is_fresh
   (&optional (__is_fresh :null))
   (if (not (eq __is_fresh :null)) (setq _is_fresh __is_fresh)) _is_fresh)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int64 _current_memory_region
    8
    ;; int64 _current_sensor_region
    8
    ;; int64 _current_feedback_region
    8
    ;; int64 _current_skill_id
    8
    ;; int64 _current_skill_type
    8
    ;; int64 _current_meta_skill_id
    8
    ;; int64 _current_meta_skill_type
    8
    ;; string _current_skill_description
    4 (length _current_skill_description)
    ;; bool _new_skill_available
    1
    ;; int64 _new_skill_id
    8
    ;; int64 _new_skill_type
    8
    ;; int64 _new_meta_skill_id
    8
    ;; int64 _new_meta_skill_type
    8
    ;; string _new_skill_description
    4 (length _new_skill_description)
    ;; bool _is_running_skill
    1
    ;; bool _skill_preempted
    1
    ;; int64 _done_skill_id
    8
    ;; int64 _result_skill_id
    8
    ;; float64 _time_skill_started_in_robot_time
    8
    ;; float64 _time_skill_finished_in_robot_time
    8
    ;; bool _is_fresh
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int64 _current_memory_region
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _current_memory_region (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _current_memory_region) (= (length (_current_memory_region . bv)) 2)) ;; bignum
              (write-long (ash (elt (_current_memory_region . bv) 0) 0) s)
              (write-long (ash (elt (_current_memory_region . bv) 1) -1) s))
             ((and (class _current_memory_region) (= (length (_current_memory_region . bv)) 1)) ;; big1
              (write-long (elt (_current_memory_region . bv) 0) s)
              (write-long (if (>= _current_memory_region 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _current_memory_region s)(write-long (if (>= _current_memory_region 0) 0 #xffffffff) s)))
     ;; int64 _current_sensor_region
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _current_sensor_region (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _current_sensor_region) (= (length (_current_sensor_region . bv)) 2)) ;; bignum
              (write-long (ash (elt (_current_sensor_region . bv) 0) 0) s)
              (write-long (ash (elt (_current_sensor_region . bv) 1) -1) s))
             ((and (class _current_sensor_region) (= (length (_current_sensor_region . bv)) 1)) ;; big1
              (write-long (elt (_current_sensor_region . bv) 0) s)
              (write-long (if (>= _current_sensor_region 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _current_sensor_region s)(write-long (if (>= _current_sensor_region 0) 0 #xffffffff) s)))
     ;; int64 _current_feedback_region
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _current_feedback_region (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _current_feedback_region) (= (length (_current_feedback_region . bv)) 2)) ;; bignum
              (write-long (ash (elt (_current_feedback_region . bv) 0) 0) s)
              (write-long (ash (elt (_current_feedback_region . bv) 1) -1) s))
             ((and (class _current_feedback_region) (= (length (_current_feedback_region . bv)) 1)) ;; big1
              (write-long (elt (_current_feedback_region . bv) 0) s)
              (write-long (if (>= _current_feedback_region 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _current_feedback_region s)(write-long (if (>= _current_feedback_region 0) 0 #xffffffff) s)))
     ;; int64 _current_skill_id
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _current_skill_id (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _current_skill_id) (= (length (_current_skill_id . bv)) 2)) ;; bignum
              (write-long (ash (elt (_current_skill_id . bv) 0) 0) s)
              (write-long (ash (elt (_current_skill_id . bv) 1) -1) s))
             ((and (class _current_skill_id) (= (length (_current_skill_id . bv)) 1)) ;; big1
              (write-long (elt (_current_skill_id . bv) 0) s)
              (write-long (if (>= _current_skill_id 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _current_skill_id s)(write-long (if (>= _current_skill_id 0) 0 #xffffffff) s)))
     ;; int64 _current_skill_type
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _current_skill_type (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _current_skill_type) (= (length (_current_skill_type . bv)) 2)) ;; bignum
              (write-long (ash (elt (_current_skill_type . bv) 0) 0) s)
              (write-long (ash (elt (_current_skill_type . bv) 1) -1) s))
             ((and (class _current_skill_type) (= (length (_current_skill_type . bv)) 1)) ;; big1
              (write-long (elt (_current_skill_type . bv) 0) s)
              (write-long (if (>= _current_skill_type 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _current_skill_type s)(write-long (if (>= _current_skill_type 0) 0 #xffffffff) s)))
     ;; int64 _current_meta_skill_id
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _current_meta_skill_id (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _current_meta_skill_id) (= (length (_current_meta_skill_id . bv)) 2)) ;; bignum
              (write-long (ash (elt (_current_meta_skill_id . bv) 0) 0) s)
              (write-long (ash (elt (_current_meta_skill_id . bv) 1) -1) s))
             ((and (class _current_meta_skill_id) (= (length (_current_meta_skill_id . bv)) 1)) ;; big1
              (write-long (elt (_current_meta_skill_id . bv) 0) s)
              (write-long (if (>= _current_meta_skill_id 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _current_meta_skill_id s)(write-long (if (>= _current_meta_skill_id 0) 0 #xffffffff) s)))
     ;; int64 _current_meta_skill_type
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _current_meta_skill_type (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _current_meta_skill_type) (= (length (_current_meta_skill_type . bv)) 2)) ;; bignum
              (write-long (ash (elt (_current_meta_skill_type . bv) 0) 0) s)
              (write-long (ash (elt (_current_meta_skill_type . bv) 1) -1) s))
             ((and (class _current_meta_skill_type) (= (length (_current_meta_skill_type . bv)) 1)) ;; big1
              (write-long (elt (_current_meta_skill_type . bv) 0) s)
              (write-long (if (>= _current_meta_skill_type 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _current_meta_skill_type s)(write-long (if (>= _current_meta_skill_type 0) 0 #xffffffff) s)))
     ;; string _current_skill_description
       (write-long (length _current_skill_description) s) (princ _current_skill_description s)
     ;; bool _new_skill_available
       (if _new_skill_available (write-byte -1 s) (write-byte 0 s))
     ;; int64 _new_skill_id
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _new_skill_id (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _new_skill_id) (= (length (_new_skill_id . bv)) 2)) ;; bignum
              (write-long (ash (elt (_new_skill_id . bv) 0) 0) s)
              (write-long (ash (elt (_new_skill_id . bv) 1) -1) s))
             ((and (class _new_skill_id) (= (length (_new_skill_id . bv)) 1)) ;; big1
              (write-long (elt (_new_skill_id . bv) 0) s)
              (write-long (if (>= _new_skill_id 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _new_skill_id s)(write-long (if (>= _new_skill_id 0) 0 #xffffffff) s)))
     ;; int64 _new_skill_type
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _new_skill_type (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _new_skill_type) (= (length (_new_skill_type . bv)) 2)) ;; bignum
              (write-long (ash (elt (_new_skill_type . bv) 0) 0) s)
              (write-long (ash (elt (_new_skill_type . bv) 1) -1) s))
             ((and (class _new_skill_type) (= (length (_new_skill_type . bv)) 1)) ;; big1
              (write-long (elt (_new_skill_type . bv) 0) s)
              (write-long (if (>= _new_skill_type 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _new_skill_type s)(write-long (if (>= _new_skill_type 0) 0 #xffffffff) s)))
     ;; int64 _new_meta_skill_id
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _new_meta_skill_id (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _new_meta_skill_id) (= (length (_new_meta_skill_id . bv)) 2)) ;; bignum
              (write-long (ash (elt (_new_meta_skill_id . bv) 0) 0) s)
              (write-long (ash (elt (_new_meta_skill_id . bv) 1) -1) s))
             ((and (class _new_meta_skill_id) (= (length (_new_meta_skill_id . bv)) 1)) ;; big1
              (write-long (elt (_new_meta_skill_id . bv) 0) s)
              (write-long (if (>= _new_meta_skill_id 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _new_meta_skill_id s)(write-long (if (>= _new_meta_skill_id 0) 0 #xffffffff) s)))
     ;; int64 _new_meta_skill_type
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _new_meta_skill_type (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _new_meta_skill_type) (= (length (_new_meta_skill_type . bv)) 2)) ;; bignum
              (write-long (ash (elt (_new_meta_skill_type . bv) 0) 0) s)
              (write-long (ash (elt (_new_meta_skill_type . bv) 1) -1) s))
             ((and (class _new_meta_skill_type) (= (length (_new_meta_skill_type . bv)) 1)) ;; big1
              (write-long (elt (_new_meta_skill_type . bv) 0) s)
              (write-long (if (>= _new_meta_skill_type 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _new_meta_skill_type s)(write-long (if (>= _new_meta_skill_type 0) 0 #xffffffff) s)))
     ;; string _new_skill_description
       (write-long (length _new_skill_description) s) (princ _new_skill_description s)
     ;; bool _is_running_skill
       (if _is_running_skill (write-byte -1 s) (write-byte 0 s))
     ;; bool _skill_preempted
       (if _skill_preempted (write-byte -1 s) (write-byte 0 s))
     ;; int64 _done_skill_id
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _done_skill_id (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _done_skill_id) (= (length (_done_skill_id . bv)) 2)) ;; bignum
              (write-long (ash (elt (_done_skill_id . bv) 0) 0) s)
              (write-long (ash (elt (_done_skill_id . bv) 1) -1) s))
             ((and (class _done_skill_id) (= (length (_done_skill_id . bv)) 1)) ;; big1
              (write-long (elt (_done_skill_id . bv) 0) s)
              (write-long (if (>= _done_skill_id 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _done_skill_id s)(write-long (if (>= _done_skill_id 0) 0 #xffffffff) s)))
     ;; int64 _result_skill_id
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _result_skill_id (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _result_skill_id) (= (length (_result_skill_id . bv)) 2)) ;; bignum
              (write-long (ash (elt (_result_skill_id . bv) 0) 0) s)
              (write-long (ash (elt (_result_skill_id . bv) 1) -1) s))
             ((and (class _result_skill_id) (= (length (_result_skill_id . bv)) 1)) ;; big1
              (write-long (elt (_result_skill_id . bv) 0) s)
              (write-long (if (>= _result_skill_id 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _result_skill_id s)(write-long (if (>= _result_skill_id 0) 0 #xffffffff) s)))
     ;; float64 _time_skill_started_in_robot_time
       (sys::poke _time_skill_started_in_robot_time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _time_skill_finished_in_robot_time
       (sys::poke _time_skill_finished_in_robot_time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _is_fresh
       (if _is_fresh (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int64 _current_memory_region
#+(or :alpha :irix6 :x86_64)
      (setf _current_memory_region (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _current_memory_region (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _current_sensor_region
#+(or :alpha :irix6 :x86_64)
      (setf _current_sensor_region (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _current_sensor_region (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _current_feedback_region
#+(or :alpha :irix6 :x86_64)
      (setf _current_feedback_region (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _current_feedback_region (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _current_skill_id
#+(or :alpha :irix6 :x86_64)
      (setf _current_skill_id (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _current_skill_id (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _current_skill_type
#+(or :alpha :irix6 :x86_64)
      (setf _current_skill_type (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _current_skill_type (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _current_meta_skill_id
#+(or :alpha :irix6 :x86_64)
      (setf _current_meta_skill_id (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _current_meta_skill_id (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _current_meta_skill_type
#+(or :alpha :irix6 :x86_64)
      (setf _current_meta_skill_type (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _current_meta_skill_type (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; string _current_skill_description
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _current_skill_description (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _new_skill_available
     (setq _new_skill_available (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int64 _new_skill_id
#+(or :alpha :irix6 :x86_64)
      (setf _new_skill_id (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _new_skill_id (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _new_skill_type
#+(or :alpha :irix6 :x86_64)
      (setf _new_skill_type (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _new_skill_type (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _new_meta_skill_id
#+(or :alpha :irix6 :x86_64)
      (setf _new_meta_skill_id (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _new_meta_skill_id (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _new_meta_skill_type
#+(or :alpha :irix6 :x86_64)
      (setf _new_meta_skill_type (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _new_meta_skill_type (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; string _new_skill_description
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _new_skill_description (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _is_running_skill
     (setq _is_running_skill (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _skill_preempted
     (setq _skill_preempted (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int64 _done_skill_id
#+(or :alpha :irix6 :x86_64)
      (setf _done_skill_id (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _done_skill_id (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _result_skill_id
#+(or :alpha :irix6 :x86_64)
      (setf _result_skill_id (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _result_skill_id (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; float64 _time_skill_started_in_robot_time
     (setq _time_skill_started_in_robot_time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _time_skill_finished_in_robot_time
     (setq _time_skill_finished_in_robot_time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _is_fresh
     (setq _is_fresh (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get franka_interface_msgs::RunLoopProcessInfoState :md5sum-) "86cd879bb4a3cbdb82beb45c42836130")
(setf (get franka_interface_msgs::RunLoopProcessInfoState :datatype-) "franka_interface_msgs/RunLoopProcessInfoState")
(setf (get franka_interface_msgs::RunLoopProcessInfoState :definition-)
      "# Skill state
std_msgs/Header header
int64 current_memory_region
int64 current_sensor_region
int64 current_feedback_region
int64 current_skill_id 
int64 current_skill_type
int64 current_meta_skill_id
int64 current_meta_skill_type
string current_skill_description
bool new_skill_available
int64 new_skill_id
int64 new_skill_type
int64 new_meta_skill_id
int64 new_meta_skill_type
string new_skill_description
bool is_running_skill
bool skill_preempted
int64 done_skill_id
int64 result_skill_id
float64 time_skill_started_in_robot_time
float64 time_skill_finished_in_robot_time
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



(provide :franka_interface_msgs/RunLoopProcessInfoState "86cd879bb4a3cbdb82beb45c42836130")



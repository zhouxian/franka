// Auto-generated. Do not edit!

// (in-package franka_interface_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class RunLoopProcessInfoState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.current_memory_region = null;
      this.current_sensor_region = null;
      this.current_feedback_region = null;
      this.current_skill_id = null;
      this.current_skill_type = null;
      this.current_meta_skill_id = null;
      this.current_meta_skill_type = null;
      this.current_skill_description = null;
      this.new_skill_available = null;
      this.new_skill_id = null;
      this.new_skill_type = null;
      this.new_meta_skill_id = null;
      this.new_meta_skill_type = null;
      this.new_skill_description = null;
      this.is_running_skill = null;
      this.skill_preempted = null;
      this.done_skill_id = null;
      this.result_skill_id = null;
      this.time_skill_started_in_robot_time = null;
      this.time_skill_finished_in_robot_time = null;
      this.is_fresh = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('current_memory_region')) {
        this.current_memory_region = initObj.current_memory_region
      }
      else {
        this.current_memory_region = 0;
      }
      if (initObj.hasOwnProperty('current_sensor_region')) {
        this.current_sensor_region = initObj.current_sensor_region
      }
      else {
        this.current_sensor_region = 0;
      }
      if (initObj.hasOwnProperty('current_feedback_region')) {
        this.current_feedback_region = initObj.current_feedback_region
      }
      else {
        this.current_feedback_region = 0;
      }
      if (initObj.hasOwnProperty('current_skill_id')) {
        this.current_skill_id = initObj.current_skill_id
      }
      else {
        this.current_skill_id = 0;
      }
      if (initObj.hasOwnProperty('current_skill_type')) {
        this.current_skill_type = initObj.current_skill_type
      }
      else {
        this.current_skill_type = 0;
      }
      if (initObj.hasOwnProperty('current_meta_skill_id')) {
        this.current_meta_skill_id = initObj.current_meta_skill_id
      }
      else {
        this.current_meta_skill_id = 0;
      }
      if (initObj.hasOwnProperty('current_meta_skill_type')) {
        this.current_meta_skill_type = initObj.current_meta_skill_type
      }
      else {
        this.current_meta_skill_type = 0;
      }
      if (initObj.hasOwnProperty('current_skill_description')) {
        this.current_skill_description = initObj.current_skill_description
      }
      else {
        this.current_skill_description = '';
      }
      if (initObj.hasOwnProperty('new_skill_available')) {
        this.new_skill_available = initObj.new_skill_available
      }
      else {
        this.new_skill_available = false;
      }
      if (initObj.hasOwnProperty('new_skill_id')) {
        this.new_skill_id = initObj.new_skill_id
      }
      else {
        this.new_skill_id = 0;
      }
      if (initObj.hasOwnProperty('new_skill_type')) {
        this.new_skill_type = initObj.new_skill_type
      }
      else {
        this.new_skill_type = 0;
      }
      if (initObj.hasOwnProperty('new_meta_skill_id')) {
        this.new_meta_skill_id = initObj.new_meta_skill_id
      }
      else {
        this.new_meta_skill_id = 0;
      }
      if (initObj.hasOwnProperty('new_meta_skill_type')) {
        this.new_meta_skill_type = initObj.new_meta_skill_type
      }
      else {
        this.new_meta_skill_type = 0;
      }
      if (initObj.hasOwnProperty('new_skill_description')) {
        this.new_skill_description = initObj.new_skill_description
      }
      else {
        this.new_skill_description = '';
      }
      if (initObj.hasOwnProperty('is_running_skill')) {
        this.is_running_skill = initObj.is_running_skill
      }
      else {
        this.is_running_skill = false;
      }
      if (initObj.hasOwnProperty('skill_preempted')) {
        this.skill_preempted = initObj.skill_preempted
      }
      else {
        this.skill_preempted = false;
      }
      if (initObj.hasOwnProperty('done_skill_id')) {
        this.done_skill_id = initObj.done_skill_id
      }
      else {
        this.done_skill_id = 0;
      }
      if (initObj.hasOwnProperty('result_skill_id')) {
        this.result_skill_id = initObj.result_skill_id
      }
      else {
        this.result_skill_id = 0;
      }
      if (initObj.hasOwnProperty('time_skill_started_in_robot_time')) {
        this.time_skill_started_in_robot_time = initObj.time_skill_started_in_robot_time
      }
      else {
        this.time_skill_started_in_robot_time = 0.0;
      }
      if (initObj.hasOwnProperty('time_skill_finished_in_robot_time')) {
        this.time_skill_finished_in_robot_time = initObj.time_skill_finished_in_robot_time
      }
      else {
        this.time_skill_finished_in_robot_time = 0.0;
      }
      if (initObj.hasOwnProperty('is_fresh')) {
        this.is_fresh = initObj.is_fresh
      }
      else {
        this.is_fresh = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RunLoopProcessInfoState
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [current_memory_region]
    bufferOffset = _serializer.int64(obj.current_memory_region, buffer, bufferOffset);
    // Serialize message field [current_sensor_region]
    bufferOffset = _serializer.int64(obj.current_sensor_region, buffer, bufferOffset);
    // Serialize message field [current_feedback_region]
    bufferOffset = _serializer.int64(obj.current_feedback_region, buffer, bufferOffset);
    // Serialize message field [current_skill_id]
    bufferOffset = _serializer.int64(obj.current_skill_id, buffer, bufferOffset);
    // Serialize message field [current_skill_type]
    bufferOffset = _serializer.int64(obj.current_skill_type, buffer, bufferOffset);
    // Serialize message field [current_meta_skill_id]
    bufferOffset = _serializer.int64(obj.current_meta_skill_id, buffer, bufferOffset);
    // Serialize message field [current_meta_skill_type]
    bufferOffset = _serializer.int64(obj.current_meta_skill_type, buffer, bufferOffset);
    // Serialize message field [current_skill_description]
    bufferOffset = _serializer.string(obj.current_skill_description, buffer, bufferOffset);
    // Serialize message field [new_skill_available]
    bufferOffset = _serializer.bool(obj.new_skill_available, buffer, bufferOffset);
    // Serialize message field [new_skill_id]
    bufferOffset = _serializer.int64(obj.new_skill_id, buffer, bufferOffset);
    // Serialize message field [new_skill_type]
    bufferOffset = _serializer.int64(obj.new_skill_type, buffer, bufferOffset);
    // Serialize message field [new_meta_skill_id]
    bufferOffset = _serializer.int64(obj.new_meta_skill_id, buffer, bufferOffset);
    // Serialize message field [new_meta_skill_type]
    bufferOffset = _serializer.int64(obj.new_meta_skill_type, buffer, bufferOffset);
    // Serialize message field [new_skill_description]
    bufferOffset = _serializer.string(obj.new_skill_description, buffer, bufferOffset);
    // Serialize message field [is_running_skill]
    bufferOffset = _serializer.bool(obj.is_running_skill, buffer, bufferOffset);
    // Serialize message field [skill_preempted]
    bufferOffset = _serializer.bool(obj.skill_preempted, buffer, bufferOffset);
    // Serialize message field [done_skill_id]
    bufferOffset = _serializer.int64(obj.done_skill_id, buffer, bufferOffset);
    // Serialize message field [result_skill_id]
    bufferOffset = _serializer.int64(obj.result_skill_id, buffer, bufferOffset);
    // Serialize message field [time_skill_started_in_robot_time]
    bufferOffset = _serializer.float64(obj.time_skill_started_in_robot_time, buffer, bufferOffset);
    // Serialize message field [time_skill_finished_in_robot_time]
    bufferOffset = _serializer.float64(obj.time_skill_finished_in_robot_time, buffer, bufferOffset);
    // Serialize message field [is_fresh]
    bufferOffset = _serializer.bool(obj.is_fresh, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RunLoopProcessInfoState
    let len;
    let data = new RunLoopProcessInfoState(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [current_memory_region]
    data.current_memory_region = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [current_sensor_region]
    data.current_sensor_region = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [current_feedback_region]
    data.current_feedback_region = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [current_skill_id]
    data.current_skill_id = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [current_skill_type]
    data.current_skill_type = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [current_meta_skill_id]
    data.current_meta_skill_id = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [current_meta_skill_type]
    data.current_meta_skill_type = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [current_skill_description]
    data.current_skill_description = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [new_skill_available]
    data.new_skill_available = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [new_skill_id]
    data.new_skill_id = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [new_skill_type]
    data.new_skill_type = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [new_meta_skill_id]
    data.new_meta_skill_id = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [new_meta_skill_type]
    data.new_meta_skill_type = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [new_skill_description]
    data.new_skill_description = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [is_running_skill]
    data.is_running_skill = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [skill_preempted]
    data.skill_preempted = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [done_skill_id]
    data.done_skill_id = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [result_skill_id]
    data.result_skill_id = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [time_skill_started_in_robot_time]
    data.time_skill_started_in_robot_time = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [time_skill_finished_in_robot_time]
    data.time_skill_finished_in_robot_time = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [is_fresh]
    data.is_fresh = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.current_skill_description);
    length += _getByteLength(object.new_skill_description);
    return length + 132;
  }

  static datatype() {
    // Returns string type for a message object
    return 'franka_interface_msgs/RunLoopProcessInfoState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '86cd879bb4a3cbdb82beb45c42836130';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Skill state
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RunLoopProcessInfoState(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.current_memory_region !== undefined) {
      resolved.current_memory_region = msg.current_memory_region;
    }
    else {
      resolved.current_memory_region = 0
    }

    if (msg.current_sensor_region !== undefined) {
      resolved.current_sensor_region = msg.current_sensor_region;
    }
    else {
      resolved.current_sensor_region = 0
    }

    if (msg.current_feedback_region !== undefined) {
      resolved.current_feedback_region = msg.current_feedback_region;
    }
    else {
      resolved.current_feedback_region = 0
    }

    if (msg.current_skill_id !== undefined) {
      resolved.current_skill_id = msg.current_skill_id;
    }
    else {
      resolved.current_skill_id = 0
    }

    if (msg.current_skill_type !== undefined) {
      resolved.current_skill_type = msg.current_skill_type;
    }
    else {
      resolved.current_skill_type = 0
    }

    if (msg.current_meta_skill_id !== undefined) {
      resolved.current_meta_skill_id = msg.current_meta_skill_id;
    }
    else {
      resolved.current_meta_skill_id = 0
    }

    if (msg.current_meta_skill_type !== undefined) {
      resolved.current_meta_skill_type = msg.current_meta_skill_type;
    }
    else {
      resolved.current_meta_skill_type = 0
    }

    if (msg.current_skill_description !== undefined) {
      resolved.current_skill_description = msg.current_skill_description;
    }
    else {
      resolved.current_skill_description = ''
    }

    if (msg.new_skill_available !== undefined) {
      resolved.new_skill_available = msg.new_skill_available;
    }
    else {
      resolved.new_skill_available = false
    }

    if (msg.new_skill_id !== undefined) {
      resolved.new_skill_id = msg.new_skill_id;
    }
    else {
      resolved.new_skill_id = 0
    }

    if (msg.new_skill_type !== undefined) {
      resolved.new_skill_type = msg.new_skill_type;
    }
    else {
      resolved.new_skill_type = 0
    }

    if (msg.new_meta_skill_id !== undefined) {
      resolved.new_meta_skill_id = msg.new_meta_skill_id;
    }
    else {
      resolved.new_meta_skill_id = 0
    }

    if (msg.new_meta_skill_type !== undefined) {
      resolved.new_meta_skill_type = msg.new_meta_skill_type;
    }
    else {
      resolved.new_meta_skill_type = 0
    }

    if (msg.new_skill_description !== undefined) {
      resolved.new_skill_description = msg.new_skill_description;
    }
    else {
      resolved.new_skill_description = ''
    }

    if (msg.is_running_skill !== undefined) {
      resolved.is_running_skill = msg.is_running_skill;
    }
    else {
      resolved.is_running_skill = false
    }

    if (msg.skill_preempted !== undefined) {
      resolved.skill_preempted = msg.skill_preempted;
    }
    else {
      resolved.skill_preempted = false
    }

    if (msg.done_skill_id !== undefined) {
      resolved.done_skill_id = msg.done_skill_id;
    }
    else {
      resolved.done_skill_id = 0
    }

    if (msg.result_skill_id !== undefined) {
      resolved.result_skill_id = msg.result_skill_id;
    }
    else {
      resolved.result_skill_id = 0
    }

    if (msg.time_skill_started_in_robot_time !== undefined) {
      resolved.time_skill_started_in_robot_time = msg.time_skill_started_in_robot_time;
    }
    else {
      resolved.time_skill_started_in_robot_time = 0.0
    }

    if (msg.time_skill_finished_in_robot_time !== undefined) {
      resolved.time_skill_finished_in_robot_time = msg.time_skill_finished_in_robot_time;
    }
    else {
      resolved.time_skill_finished_in_robot_time = 0.0
    }

    if (msg.is_fresh !== undefined) {
      resolved.is_fresh = msg.is_fresh;
    }
    else {
      resolved.is_fresh = false
    }

    return resolved;
    }
};

module.exports = RunLoopProcessInfoState;

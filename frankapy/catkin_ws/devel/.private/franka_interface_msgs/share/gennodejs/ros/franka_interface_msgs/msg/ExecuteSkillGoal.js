// Auto-generated. Do not edit!

// (in-package franka_interface_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class ExecuteSkillGoal {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.skill_type = null;
      this.skill_description = null;
      this.meta_skill_type = null;
      this.meta_skill_id = null;
      this.sensor_topics = null;
      this.sensor_value_sizes = null;
      this.initial_sensor_values = null;
      this.trajectory_generator_type = null;
      this.trajectory_generator_param_data_size = null;
      this.trajectory_generator_param_data = null;
      this.feedback_controller_type = null;
      this.feedback_controller_param_data_size = null;
      this.feedback_controller_param_data = null;
      this.termination_handler_type = null;
      this.termination_handler_param_data_size = null;
      this.termination_handler_param_data = null;
      this.timer_type = null;
      this.num_timer_params = null;
      this.timer_params = null;
    }
    else {
      if (initObj.hasOwnProperty('skill_type')) {
        this.skill_type = initObj.skill_type
      }
      else {
        this.skill_type = 0;
      }
      if (initObj.hasOwnProperty('skill_description')) {
        this.skill_description = initObj.skill_description
      }
      else {
        this.skill_description = '';
      }
      if (initObj.hasOwnProperty('meta_skill_type')) {
        this.meta_skill_type = initObj.meta_skill_type
      }
      else {
        this.meta_skill_type = 0;
      }
      if (initObj.hasOwnProperty('meta_skill_id')) {
        this.meta_skill_id = initObj.meta_skill_id
      }
      else {
        this.meta_skill_id = 0;
      }
      if (initObj.hasOwnProperty('sensor_topics')) {
        this.sensor_topics = initObj.sensor_topics
      }
      else {
        this.sensor_topics = [];
      }
      if (initObj.hasOwnProperty('sensor_value_sizes')) {
        this.sensor_value_sizes = initObj.sensor_value_sizes
      }
      else {
        this.sensor_value_sizes = [];
      }
      if (initObj.hasOwnProperty('initial_sensor_values')) {
        this.initial_sensor_values = initObj.initial_sensor_values
      }
      else {
        this.initial_sensor_values = [];
      }
      if (initObj.hasOwnProperty('trajectory_generator_type')) {
        this.trajectory_generator_type = initObj.trajectory_generator_type
      }
      else {
        this.trajectory_generator_type = 0;
      }
      if (initObj.hasOwnProperty('trajectory_generator_param_data_size')) {
        this.trajectory_generator_param_data_size = initObj.trajectory_generator_param_data_size
      }
      else {
        this.trajectory_generator_param_data_size = 0;
      }
      if (initObj.hasOwnProperty('trajectory_generator_param_data')) {
        this.trajectory_generator_param_data = initObj.trajectory_generator_param_data
      }
      else {
        this.trajectory_generator_param_data = [];
      }
      if (initObj.hasOwnProperty('feedback_controller_type')) {
        this.feedback_controller_type = initObj.feedback_controller_type
      }
      else {
        this.feedback_controller_type = 0;
      }
      if (initObj.hasOwnProperty('feedback_controller_param_data_size')) {
        this.feedback_controller_param_data_size = initObj.feedback_controller_param_data_size
      }
      else {
        this.feedback_controller_param_data_size = 0;
      }
      if (initObj.hasOwnProperty('feedback_controller_param_data')) {
        this.feedback_controller_param_data = initObj.feedback_controller_param_data
      }
      else {
        this.feedback_controller_param_data = [];
      }
      if (initObj.hasOwnProperty('termination_handler_type')) {
        this.termination_handler_type = initObj.termination_handler_type
      }
      else {
        this.termination_handler_type = 0;
      }
      if (initObj.hasOwnProperty('termination_handler_param_data_size')) {
        this.termination_handler_param_data_size = initObj.termination_handler_param_data_size
      }
      else {
        this.termination_handler_param_data_size = 0;
      }
      if (initObj.hasOwnProperty('termination_handler_param_data')) {
        this.termination_handler_param_data = initObj.termination_handler_param_data
      }
      else {
        this.termination_handler_param_data = [];
      }
      if (initObj.hasOwnProperty('timer_type')) {
        this.timer_type = initObj.timer_type
      }
      else {
        this.timer_type = 0;
      }
      if (initObj.hasOwnProperty('num_timer_params')) {
        this.num_timer_params = initObj.num_timer_params
      }
      else {
        this.num_timer_params = 0;
      }
      if (initObj.hasOwnProperty('timer_params')) {
        this.timer_params = initObj.timer_params
      }
      else {
        this.timer_params = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ExecuteSkillGoal
    // Serialize message field [skill_type]
    bufferOffset = _serializer.uint8(obj.skill_type, buffer, bufferOffset);
    // Serialize message field [skill_description]
    bufferOffset = _serializer.string(obj.skill_description, buffer, bufferOffset);
    // Serialize message field [meta_skill_type]
    bufferOffset = _serializer.uint8(obj.meta_skill_type, buffer, bufferOffset);
    // Serialize message field [meta_skill_id]
    bufferOffset = _serializer.int64(obj.meta_skill_id, buffer, bufferOffset);
    // Serialize message field [sensor_topics]
    bufferOffset = _arraySerializer.string(obj.sensor_topics, buffer, bufferOffset, null);
    // Serialize message field [sensor_value_sizes]
    bufferOffset = _arraySerializer.uint64(obj.sensor_value_sizes, buffer, bufferOffset, null);
    // Serialize message field [initial_sensor_values]
    bufferOffset = _arraySerializer.float64(obj.initial_sensor_values, buffer, bufferOffset, null);
    // Serialize message field [trajectory_generator_type]
    bufferOffset = _serializer.uint8(obj.trajectory_generator_type, buffer, bufferOffset);
    // Serialize message field [trajectory_generator_param_data_size]
    bufferOffset = _serializer.int32(obj.trajectory_generator_param_data_size, buffer, bufferOffset);
    // Serialize message field [trajectory_generator_param_data]
    bufferOffset = _arraySerializer.uint8(obj.trajectory_generator_param_data, buffer, bufferOffset, null);
    // Serialize message field [feedback_controller_type]
    bufferOffset = _serializer.uint8(obj.feedback_controller_type, buffer, bufferOffset);
    // Serialize message field [feedback_controller_param_data_size]
    bufferOffset = _serializer.int32(obj.feedback_controller_param_data_size, buffer, bufferOffset);
    // Serialize message field [feedback_controller_param_data]
    bufferOffset = _arraySerializer.uint8(obj.feedback_controller_param_data, buffer, bufferOffset, null);
    // Serialize message field [termination_handler_type]
    bufferOffset = _serializer.uint8(obj.termination_handler_type, buffer, bufferOffset);
    // Serialize message field [termination_handler_param_data_size]
    bufferOffset = _serializer.int32(obj.termination_handler_param_data_size, buffer, bufferOffset);
    // Serialize message field [termination_handler_param_data]
    bufferOffset = _arraySerializer.uint8(obj.termination_handler_param_data, buffer, bufferOffset, null);
    // Serialize message field [timer_type]
    bufferOffset = _serializer.uint8(obj.timer_type, buffer, bufferOffset);
    // Serialize message field [num_timer_params]
    bufferOffset = _serializer.int32(obj.num_timer_params, buffer, bufferOffset);
    // Serialize message field [timer_params]
    bufferOffset = _arraySerializer.uint8(obj.timer_params, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ExecuteSkillGoal
    let len;
    let data = new ExecuteSkillGoal(null);
    // Deserialize message field [skill_type]
    data.skill_type = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [skill_description]
    data.skill_description = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [meta_skill_type]
    data.meta_skill_type = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [meta_skill_id]
    data.meta_skill_id = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [sensor_topics]
    data.sensor_topics = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [sensor_value_sizes]
    data.sensor_value_sizes = _arrayDeserializer.uint64(buffer, bufferOffset, null)
    // Deserialize message field [initial_sensor_values]
    data.initial_sensor_values = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [trajectory_generator_type]
    data.trajectory_generator_type = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [trajectory_generator_param_data_size]
    data.trajectory_generator_param_data_size = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [trajectory_generator_param_data]
    data.trajectory_generator_param_data = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    // Deserialize message field [feedback_controller_type]
    data.feedback_controller_type = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [feedback_controller_param_data_size]
    data.feedback_controller_param_data_size = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [feedback_controller_param_data]
    data.feedback_controller_param_data = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    // Deserialize message field [termination_handler_type]
    data.termination_handler_type = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [termination_handler_param_data_size]
    data.termination_handler_param_data_size = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [termination_handler_param_data]
    data.termination_handler_param_data = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    // Deserialize message field [timer_type]
    data.timer_type = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [num_timer_params]
    data.num_timer_params = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [timer_params]
    data.timer_params = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.skill_description);
    object.sensor_topics.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    length += 8 * object.sensor_value_sizes.length;
    length += 8 * object.initial_sensor_values.length;
    length += object.trajectory_generator_param_data.length;
    length += object.feedback_controller_param_data.length;
    length += object.termination_handler_param_data.length;
    length += object.timer_params.length;
    return length + 62;
  }

  static datatype() {
    // Returns string type for a message object
    return 'franka_interface_msgs/ExecuteSkillGoal';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '55e633f630d923733f24dd64b606c67a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ExecuteSkillGoal(null);
    if (msg.skill_type !== undefined) {
      resolved.skill_type = msg.skill_type;
    }
    else {
      resolved.skill_type = 0
    }

    if (msg.skill_description !== undefined) {
      resolved.skill_description = msg.skill_description;
    }
    else {
      resolved.skill_description = ''
    }

    if (msg.meta_skill_type !== undefined) {
      resolved.meta_skill_type = msg.meta_skill_type;
    }
    else {
      resolved.meta_skill_type = 0
    }

    if (msg.meta_skill_id !== undefined) {
      resolved.meta_skill_id = msg.meta_skill_id;
    }
    else {
      resolved.meta_skill_id = 0
    }

    if (msg.sensor_topics !== undefined) {
      resolved.sensor_topics = msg.sensor_topics;
    }
    else {
      resolved.sensor_topics = []
    }

    if (msg.sensor_value_sizes !== undefined) {
      resolved.sensor_value_sizes = msg.sensor_value_sizes;
    }
    else {
      resolved.sensor_value_sizes = []
    }

    if (msg.initial_sensor_values !== undefined) {
      resolved.initial_sensor_values = msg.initial_sensor_values;
    }
    else {
      resolved.initial_sensor_values = []
    }

    if (msg.trajectory_generator_type !== undefined) {
      resolved.trajectory_generator_type = msg.trajectory_generator_type;
    }
    else {
      resolved.trajectory_generator_type = 0
    }

    if (msg.trajectory_generator_param_data_size !== undefined) {
      resolved.trajectory_generator_param_data_size = msg.trajectory_generator_param_data_size;
    }
    else {
      resolved.trajectory_generator_param_data_size = 0
    }

    if (msg.trajectory_generator_param_data !== undefined) {
      resolved.trajectory_generator_param_data = msg.trajectory_generator_param_data;
    }
    else {
      resolved.trajectory_generator_param_data = []
    }

    if (msg.feedback_controller_type !== undefined) {
      resolved.feedback_controller_type = msg.feedback_controller_type;
    }
    else {
      resolved.feedback_controller_type = 0
    }

    if (msg.feedback_controller_param_data_size !== undefined) {
      resolved.feedback_controller_param_data_size = msg.feedback_controller_param_data_size;
    }
    else {
      resolved.feedback_controller_param_data_size = 0
    }

    if (msg.feedback_controller_param_data !== undefined) {
      resolved.feedback_controller_param_data = msg.feedback_controller_param_data;
    }
    else {
      resolved.feedback_controller_param_data = []
    }

    if (msg.termination_handler_type !== undefined) {
      resolved.termination_handler_type = msg.termination_handler_type;
    }
    else {
      resolved.termination_handler_type = 0
    }

    if (msg.termination_handler_param_data_size !== undefined) {
      resolved.termination_handler_param_data_size = msg.termination_handler_param_data_size;
    }
    else {
      resolved.termination_handler_param_data_size = 0
    }

    if (msg.termination_handler_param_data !== undefined) {
      resolved.termination_handler_param_data = msg.termination_handler_param_data;
    }
    else {
      resolved.termination_handler_param_data = []
    }

    if (msg.timer_type !== undefined) {
      resolved.timer_type = msg.timer_type;
    }
    else {
      resolved.timer_type = 0
    }

    if (msg.num_timer_params !== undefined) {
      resolved.num_timer_params = msg.num_timer_params;
    }
    else {
      resolved.num_timer_params = 0
    }

    if (msg.timer_params !== undefined) {
      resolved.timer_params = msg.timer_params;
    }
    else {
      resolved.timer_params = []
    }

    return resolved;
    }
};

module.exports = ExecuteSkillGoal;

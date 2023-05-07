// Auto-generated. Do not edit!

// (in-package franka_interface_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let SensorData = require('./SensorData.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class SensorDataGroup {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.has_trajectory_generator_sensor_data = null;
      this.has_feedback_controller_sensor_data = null;
      this.has_termination_handler_sensor_data = null;
      this.trajectoryGeneratorSensorData = null;
      this.feedbackControllerSensorData = null;
      this.terminationHandlerSensorData = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('has_trajectory_generator_sensor_data')) {
        this.has_trajectory_generator_sensor_data = initObj.has_trajectory_generator_sensor_data
      }
      else {
        this.has_trajectory_generator_sensor_data = false;
      }
      if (initObj.hasOwnProperty('has_feedback_controller_sensor_data')) {
        this.has_feedback_controller_sensor_data = initObj.has_feedback_controller_sensor_data
      }
      else {
        this.has_feedback_controller_sensor_data = false;
      }
      if (initObj.hasOwnProperty('has_termination_handler_sensor_data')) {
        this.has_termination_handler_sensor_data = initObj.has_termination_handler_sensor_data
      }
      else {
        this.has_termination_handler_sensor_data = false;
      }
      if (initObj.hasOwnProperty('trajectoryGeneratorSensorData')) {
        this.trajectoryGeneratorSensorData = initObj.trajectoryGeneratorSensorData
      }
      else {
        this.trajectoryGeneratorSensorData = new SensorData();
      }
      if (initObj.hasOwnProperty('feedbackControllerSensorData')) {
        this.feedbackControllerSensorData = initObj.feedbackControllerSensorData
      }
      else {
        this.feedbackControllerSensorData = new SensorData();
      }
      if (initObj.hasOwnProperty('terminationHandlerSensorData')) {
        this.terminationHandlerSensorData = initObj.terminationHandlerSensorData
      }
      else {
        this.terminationHandlerSensorData = new SensorData();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SensorDataGroup
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [has_trajectory_generator_sensor_data]
    bufferOffset = _serializer.bool(obj.has_trajectory_generator_sensor_data, buffer, bufferOffset);
    // Serialize message field [has_feedback_controller_sensor_data]
    bufferOffset = _serializer.bool(obj.has_feedback_controller_sensor_data, buffer, bufferOffset);
    // Serialize message field [has_termination_handler_sensor_data]
    bufferOffset = _serializer.bool(obj.has_termination_handler_sensor_data, buffer, bufferOffset);
    // Serialize message field [trajectoryGeneratorSensorData]
    bufferOffset = SensorData.serialize(obj.trajectoryGeneratorSensorData, buffer, bufferOffset);
    // Serialize message field [feedbackControllerSensorData]
    bufferOffset = SensorData.serialize(obj.feedbackControllerSensorData, buffer, bufferOffset);
    // Serialize message field [terminationHandlerSensorData]
    bufferOffset = SensorData.serialize(obj.terminationHandlerSensorData, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SensorDataGroup
    let len;
    let data = new SensorDataGroup(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [has_trajectory_generator_sensor_data]
    data.has_trajectory_generator_sensor_data = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [has_feedback_controller_sensor_data]
    data.has_feedback_controller_sensor_data = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [has_termination_handler_sensor_data]
    data.has_termination_handler_sensor_data = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [trajectoryGeneratorSensorData]
    data.trajectoryGeneratorSensorData = SensorData.deserialize(buffer, bufferOffset);
    // Deserialize message field [feedbackControllerSensorData]
    data.feedbackControllerSensorData = SensorData.deserialize(buffer, bufferOffset);
    // Deserialize message field [terminationHandlerSensorData]
    data.terminationHandlerSensorData = SensorData.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += SensorData.getMessageSize(object.trajectoryGeneratorSensorData);
    length += SensorData.getMessageSize(object.feedbackControllerSensorData);
    length += SensorData.getMessageSize(object.terminationHandlerSensorData);
    return length + 3;
  }

  static datatype() {
    // Returns string type for a message object
    return 'franka_interface_msgs/SensorDataGroup';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '859c186ac0046509397a685ba0f2e8d2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # SensorDataGroup
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SensorDataGroup(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.has_trajectory_generator_sensor_data !== undefined) {
      resolved.has_trajectory_generator_sensor_data = msg.has_trajectory_generator_sensor_data;
    }
    else {
      resolved.has_trajectory_generator_sensor_data = false
    }

    if (msg.has_feedback_controller_sensor_data !== undefined) {
      resolved.has_feedback_controller_sensor_data = msg.has_feedback_controller_sensor_data;
    }
    else {
      resolved.has_feedback_controller_sensor_data = false
    }

    if (msg.has_termination_handler_sensor_data !== undefined) {
      resolved.has_termination_handler_sensor_data = msg.has_termination_handler_sensor_data;
    }
    else {
      resolved.has_termination_handler_sensor_data = false
    }

    if (msg.trajectoryGeneratorSensorData !== undefined) {
      resolved.trajectoryGeneratorSensorData = SensorData.Resolve(msg.trajectoryGeneratorSensorData)
    }
    else {
      resolved.trajectoryGeneratorSensorData = new SensorData()
    }

    if (msg.feedbackControllerSensorData !== undefined) {
      resolved.feedbackControllerSensorData = SensorData.Resolve(msg.feedbackControllerSensorData)
    }
    else {
      resolved.feedbackControllerSensorData = new SensorData()
    }

    if (msg.terminationHandlerSensorData !== undefined) {
      resolved.terminationHandlerSensorData = SensorData.Resolve(msg.terminationHandlerSensorData)
    }
    else {
      resolved.terminationHandlerSensorData = new SensorData()
    }

    return resolved;
    }
};

module.exports = SensorDataGroup;

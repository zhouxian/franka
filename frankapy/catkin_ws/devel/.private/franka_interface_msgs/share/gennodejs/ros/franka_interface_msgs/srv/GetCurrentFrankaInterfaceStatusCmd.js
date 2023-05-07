// Auto-generated. Do not edit!

// (in-package franka_interface_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let FrankaInterfaceStatus = require('../msg/FrankaInterfaceStatus.js');

//-----------------------------------------------------------

class GetCurrentFrankaInterfaceStatusCmdRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetCurrentFrankaInterfaceStatusCmdRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetCurrentFrankaInterfaceStatusCmdRequest
    let len;
    let data = new GetCurrentFrankaInterfaceStatusCmdRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'franka_interface_msgs/GetCurrentFrankaInterfaceStatusCmdRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetCurrentFrankaInterfaceStatusCmdRequest(null);
    return resolved;
    }
};

class GetCurrentFrankaInterfaceStatusCmdResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.franka_interface_status = null;
    }
    else {
      if (initObj.hasOwnProperty('franka_interface_status')) {
        this.franka_interface_status = initObj.franka_interface_status
      }
      else {
        this.franka_interface_status = new FrankaInterfaceStatus();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetCurrentFrankaInterfaceStatusCmdResponse
    // Serialize message field [franka_interface_status]
    bufferOffset = FrankaInterfaceStatus.serialize(obj.franka_interface_status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetCurrentFrankaInterfaceStatusCmdResponse
    let len;
    let data = new GetCurrentFrankaInterfaceStatusCmdResponse(null);
    // Deserialize message field [franka_interface_status]
    data.franka_interface_status = FrankaInterfaceStatus.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += FrankaInterfaceStatus.getMessageSize(object.franka_interface_status);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'franka_interface_msgs/GetCurrentFrankaInterfaceStatusCmdResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1d3fe55393993751ebdd77c7f87c1d1e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetCurrentFrankaInterfaceStatusCmdResponse(null);
    if (msg.franka_interface_status !== undefined) {
      resolved.franka_interface_status = FrankaInterfaceStatus.Resolve(msg.franka_interface_status)
    }
    else {
      resolved.franka_interface_status = new FrankaInterfaceStatus()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetCurrentFrankaInterfaceStatusCmdRequest,
  Response: GetCurrentFrankaInterfaceStatusCmdResponse,
  md5sum() { return '1d3fe55393993751ebdd77c7f87c1d1e'; },
  datatype() { return 'franka_interface_msgs/GetCurrentFrankaInterfaceStatusCmd'; }
};

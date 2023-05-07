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

class FrankaInterfaceStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.is_ready = null;
      this.error_description = null;
      this.is_fresh = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('is_ready')) {
        this.is_ready = initObj.is_ready
      }
      else {
        this.is_ready = false;
      }
      if (initObj.hasOwnProperty('error_description')) {
        this.error_description = initObj.error_description
      }
      else {
        this.error_description = '';
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
    // Serializes a message object of type FrankaInterfaceStatus
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [is_ready]
    bufferOffset = _serializer.bool(obj.is_ready, buffer, bufferOffset);
    // Serialize message field [error_description]
    bufferOffset = _serializer.string(obj.error_description, buffer, bufferOffset);
    // Serialize message field [is_fresh]
    bufferOffset = _serializer.bool(obj.is_fresh, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FrankaInterfaceStatus
    let len;
    let data = new FrankaInterfaceStatus(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [is_ready]
    data.is_ready = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [error_description]
    data.error_description = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [is_fresh]
    data.is_fresh = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.error_description);
    return length + 6;
  }

  static datatype() {
    // Returns string type for a message object
    return 'franka_interface_msgs/FrankaInterfaceStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b3b55f2e82de58761423a010b7b229a0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new FrankaInterfaceStatus(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.is_ready !== undefined) {
      resolved.is_ready = msg.is_ready;
    }
    else {
      resolved.is_ready = false
    }

    if (msg.error_description !== undefined) {
      resolved.error_description = msg.error_description;
    }
    else {
      resolved.error_description = ''
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

module.exports = FrankaInterfaceStatus;

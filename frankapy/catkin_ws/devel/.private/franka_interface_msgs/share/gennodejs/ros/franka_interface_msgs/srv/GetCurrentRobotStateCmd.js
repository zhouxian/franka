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

let RobotState = require('../msg/RobotState.js');

//-----------------------------------------------------------

class GetCurrentRobotStateCmdRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetCurrentRobotStateCmdRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetCurrentRobotStateCmdRequest
    let len;
    let data = new GetCurrentRobotStateCmdRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'franka_interface_msgs/GetCurrentRobotStateCmdRequest';
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
    const resolved = new GetCurrentRobotStateCmdRequest(null);
    return resolved;
    }
};

class GetCurrentRobotStateCmdResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.robot_state = null;
    }
    else {
      if (initObj.hasOwnProperty('robot_state')) {
        this.robot_state = initObj.robot_state
      }
      else {
        this.robot_state = new RobotState();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetCurrentRobotStateCmdResponse
    // Serialize message field [robot_state]
    bufferOffset = RobotState.serialize(obj.robot_state, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetCurrentRobotStateCmdResponse
    let len;
    let data = new GetCurrentRobotStateCmdResponse(null);
    // Deserialize message field [robot_state]
    data.robot_state = RobotState.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += RobotState.getMessageSize(object.robot_state);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'franka_interface_msgs/GetCurrentRobotStateCmdResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b518dbe0aba9b1925588d18e42fb85fa';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    RobotState robot_state
    
    ================================================================================
    MSG: franka_interface_msgs/RobotState
    # Franka robot state
    # For more information about each field, look at robot_state.h here: 
    # https://github.com/frankaemika/libfranka/blob/master/include/franka/robot_state.h
    std_msgs/Header header
    float64[16] pose_desired
    float64[16] O_T_EE 
    float64[16] O_T_EE_d
    float64[16] F_T_EE
    float64[16] EE_T_K
    float64 m_ee
    float64[9] I_ee
    float64[3] F_x_Cee
    float64 m_load
    float64[9] I_load
    float64[3] F_x_Cload
    float64 m_total
    float64[9] I_total
    float64[3] F_x_Ctotal
    float64[2] elbow
    float64[2] elbow_d
    float64[2] elbow_c
    float64[2] delbow_c
    float64[2] ddelbow_c
    float64[7] tau_J
    float64[7] tau_J_d
    float64[7] dtau_J
    float64[7] q
    float64[7] q_d
    float64[7] dq
    float64[7] dq_d
    float64[7] ddq_d
    float64[7] joint_contact
    float64[6] cartesian_contact
    float64[7] joint_collision
    float64[6] cartesian_collision
    float64[7] tau_ext_hat_filtered
    float64[6] O_F_ext_hat_K
    float64[6] K_F_ext_hat_K
    float64[6] O_dP_EE_d
    float64[16] O_T_EE_c
    float64[6] O_dP_EE_c
    float64[6] O_ddP_EE_c
    float64[7] theta
    float64[7] dtheta
    Errors current_errors
    Errors last_motion_errors
    float64 control_command_success_rate
    uint8 ROBOT_MODE_OTHER=0
    uint8 ROBOT_MODE_IDLE=1
    uint8 ROBOT_MODE_MOVE=2
    uint8 ROBOT_MODE_GUIDING=3
    uint8 ROBOT_MODE_REFLEX=4
    uint8 ROBOT_MODE_USER_STOPPED=5
    uint8 ROBOT_MODE_AUTOMATIC_ERROR_RECOVERY=6
    uint8 robot_mode
    float64 robot_time
    float64 gripper_width
    float64 gripper_max_width
    bool gripper_is_grasped
    uint16 gripper_temperature
    float64 gripper_time
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
    
    ================================================================================
    MSG: franka_interface_msgs/Errors
    bool joint_position_limits_violation
    bool cartesian_position_limits_violation
    bool self_collision_avoidance_violation
    bool joint_velocity_violation
    bool cartesian_velocity_violation
    bool force_control_safety_violation
    bool joint_reflex
    bool cartesian_reflex
    bool max_goal_pose_deviation_violation
    bool max_path_pose_deviation_violation
    bool cartesian_velocity_profile_safety_violation
    bool joint_position_motion_generator_start_pose_invalid
    bool joint_motion_generator_position_limits_violation
    bool joint_motion_generator_velocity_limits_violation
    bool joint_motion_generator_velocity_discontinuity
    bool joint_motion_generator_acceleration_discontinuity
    bool cartesian_position_motion_generator_start_pose_invalid
    bool cartesian_motion_generator_elbow_limit_violation
    bool cartesian_motion_generator_velocity_limits_violation
    bool cartesian_motion_generator_velocity_discontinuity
    bool cartesian_motion_generator_acceleration_discontinuity
    bool cartesian_motion_generator_elbow_sign_inconsistent
    bool cartesian_motion_generator_start_elbow_invalid
    bool cartesian_motion_generator_joint_position_limits_violation
    bool cartesian_motion_generator_joint_velocity_limits_violation
    bool cartesian_motion_generator_joint_velocity_discontinuity
    bool cartesian_motion_generator_joint_acceleration_discontinuity
    bool cartesian_position_motion_generator_invalid_frame
    bool force_controller_desired_force_tolerance_violation
    bool controller_torque_discontinuity
    bool start_elbow_sign_inconsistent
    bool communication_constraints_violation
    bool power_limit_violation
    bool joint_p2p_insufficient_torque_for_planning
    bool tau_j_range_violation
    bool instability_detected
    bool joint_move_in_wrong_direction
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetCurrentRobotStateCmdResponse(null);
    if (msg.robot_state !== undefined) {
      resolved.robot_state = RobotState.Resolve(msg.robot_state)
    }
    else {
      resolved.robot_state = new RobotState()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetCurrentRobotStateCmdRequest,
  Response: GetCurrentRobotStateCmdResponse,
  md5sum() { return 'b518dbe0aba9b1925588d18e42fb85fa'; },
  datatype() { return 'franka_interface_msgs/GetCurrentRobotStateCmd'; }
};

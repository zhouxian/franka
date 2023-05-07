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

class Errors {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.joint_position_limits_violation = null;
      this.cartesian_position_limits_violation = null;
      this.self_collision_avoidance_violation = null;
      this.joint_velocity_violation = null;
      this.cartesian_velocity_violation = null;
      this.force_control_safety_violation = null;
      this.joint_reflex = null;
      this.cartesian_reflex = null;
      this.max_goal_pose_deviation_violation = null;
      this.max_path_pose_deviation_violation = null;
      this.cartesian_velocity_profile_safety_violation = null;
      this.joint_position_motion_generator_start_pose_invalid = null;
      this.joint_motion_generator_position_limits_violation = null;
      this.joint_motion_generator_velocity_limits_violation = null;
      this.joint_motion_generator_velocity_discontinuity = null;
      this.joint_motion_generator_acceleration_discontinuity = null;
      this.cartesian_position_motion_generator_start_pose_invalid = null;
      this.cartesian_motion_generator_elbow_limit_violation = null;
      this.cartesian_motion_generator_velocity_limits_violation = null;
      this.cartesian_motion_generator_velocity_discontinuity = null;
      this.cartesian_motion_generator_acceleration_discontinuity = null;
      this.cartesian_motion_generator_elbow_sign_inconsistent = null;
      this.cartesian_motion_generator_start_elbow_invalid = null;
      this.cartesian_motion_generator_joint_position_limits_violation = null;
      this.cartesian_motion_generator_joint_velocity_limits_violation = null;
      this.cartesian_motion_generator_joint_velocity_discontinuity = null;
      this.cartesian_motion_generator_joint_acceleration_discontinuity = null;
      this.cartesian_position_motion_generator_invalid_frame = null;
      this.force_controller_desired_force_tolerance_violation = null;
      this.controller_torque_discontinuity = null;
      this.start_elbow_sign_inconsistent = null;
      this.communication_constraints_violation = null;
      this.power_limit_violation = null;
      this.joint_p2p_insufficient_torque_for_planning = null;
      this.tau_j_range_violation = null;
      this.instability_detected = null;
      this.joint_move_in_wrong_direction = null;
    }
    else {
      if (initObj.hasOwnProperty('joint_position_limits_violation')) {
        this.joint_position_limits_violation = initObj.joint_position_limits_violation
      }
      else {
        this.joint_position_limits_violation = false;
      }
      if (initObj.hasOwnProperty('cartesian_position_limits_violation')) {
        this.cartesian_position_limits_violation = initObj.cartesian_position_limits_violation
      }
      else {
        this.cartesian_position_limits_violation = false;
      }
      if (initObj.hasOwnProperty('self_collision_avoidance_violation')) {
        this.self_collision_avoidance_violation = initObj.self_collision_avoidance_violation
      }
      else {
        this.self_collision_avoidance_violation = false;
      }
      if (initObj.hasOwnProperty('joint_velocity_violation')) {
        this.joint_velocity_violation = initObj.joint_velocity_violation
      }
      else {
        this.joint_velocity_violation = false;
      }
      if (initObj.hasOwnProperty('cartesian_velocity_violation')) {
        this.cartesian_velocity_violation = initObj.cartesian_velocity_violation
      }
      else {
        this.cartesian_velocity_violation = false;
      }
      if (initObj.hasOwnProperty('force_control_safety_violation')) {
        this.force_control_safety_violation = initObj.force_control_safety_violation
      }
      else {
        this.force_control_safety_violation = false;
      }
      if (initObj.hasOwnProperty('joint_reflex')) {
        this.joint_reflex = initObj.joint_reflex
      }
      else {
        this.joint_reflex = false;
      }
      if (initObj.hasOwnProperty('cartesian_reflex')) {
        this.cartesian_reflex = initObj.cartesian_reflex
      }
      else {
        this.cartesian_reflex = false;
      }
      if (initObj.hasOwnProperty('max_goal_pose_deviation_violation')) {
        this.max_goal_pose_deviation_violation = initObj.max_goal_pose_deviation_violation
      }
      else {
        this.max_goal_pose_deviation_violation = false;
      }
      if (initObj.hasOwnProperty('max_path_pose_deviation_violation')) {
        this.max_path_pose_deviation_violation = initObj.max_path_pose_deviation_violation
      }
      else {
        this.max_path_pose_deviation_violation = false;
      }
      if (initObj.hasOwnProperty('cartesian_velocity_profile_safety_violation')) {
        this.cartesian_velocity_profile_safety_violation = initObj.cartesian_velocity_profile_safety_violation
      }
      else {
        this.cartesian_velocity_profile_safety_violation = false;
      }
      if (initObj.hasOwnProperty('joint_position_motion_generator_start_pose_invalid')) {
        this.joint_position_motion_generator_start_pose_invalid = initObj.joint_position_motion_generator_start_pose_invalid
      }
      else {
        this.joint_position_motion_generator_start_pose_invalid = false;
      }
      if (initObj.hasOwnProperty('joint_motion_generator_position_limits_violation')) {
        this.joint_motion_generator_position_limits_violation = initObj.joint_motion_generator_position_limits_violation
      }
      else {
        this.joint_motion_generator_position_limits_violation = false;
      }
      if (initObj.hasOwnProperty('joint_motion_generator_velocity_limits_violation')) {
        this.joint_motion_generator_velocity_limits_violation = initObj.joint_motion_generator_velocity_limits_violation
      }
      else {
        this.joint_motion_generator_velocity_limits_violation = false;
      }
      if (initObj.hasOwnProperty('joint_motion_generator_velocity_discontinuity')) {
        this.joint_motion_generator_velocity_discontinuity = initObj.joint_motion_generator_velocity_discontinuity
      }
      else {
        this.joint_motion_generator_velocity_discontinuity = false;
      }
      if (initObj.hasOwnProperty('joint_motion_generator_acceleration_discontinuity')) {
        this.joint_motion_generator_acceleration_discontinuity = initObj.joint_motion_generator_acceleration_discontinuity
      }
      else {
        this.joint_motion_generator_acceleration_discontinuity = false;
      }
      if (initObj.hasOwnProperty('cartesian_position_motion_generator_start_pose_invalid')) {
        this.cartesian_position_motion_generator_start_pose_invalid = initObj.cartesian_position_motion_generator_start_pose_invalid
      }
      else {
        this.cartesian_position_motion_generator_start_pose_invalid = false;
      }
      if (initObj.hasOwnProperty('cartesian_motion_generator_elbow_limit_violation')) {
        this.cartesian_motion_generator_elbow_limit_violation = initObj.cartesian_motion_generator_elbow_limit_violation
      }
      else {
        this.cartesian_motion_generator_elbow_limit_violation = false;
      }
      if (initObj.hasOwnProperty('cartesian_motion_generator_velocity_limits_violation')) {
        this.cartesian_motion_generator_velocity_limits_violation = initObj.cartesian_motion_generator_velocity_limits_violation
      }
      else {
        this.cartesian_motion_generator_velocity_limits_violation = false;
      }
      if (initObj.hasOwnProperty('cartesian_motion_generator_velocity_discontinuity')) {
        this.cartesian_motion_generator_velocity_discontinuity = initObj.cartesian_motion_generator_velocity_discontinuity
      }
      else {
        this.cartesian_motion_generator_velocity_discontinuity = false;
      }
      if (initObj.hasOwnProperty('cartesian_motion_generator_acceleration_discontinuity')) {
        this.cartesian_motion_generator_acceleration_discontinuity = initObj.cartesian_motion_generator_acceleration_discontinuity
      }
      else {
        this.cartesian_motion_generator_acceleration_discontinuity = false;
      }
      if (initObj.hasOwnProperty('cartesian_motion_generator_elbow_sign_inconsistent')) {
        this.cartesian_motion_generator_elbow_sign_inconsistent = initObj.cartesian_motion_generator_elbow_sign_inconsistent
      }
      else {
        this.cartesian_motion_generator_elbow_sign_inconsistent = false;
      }
      if (initObj.hasOwnProperty('cartesian_motion_generator_start_elbow_invalid')) {
        this.cartesian_motion_generator_start_elbow_invalid = initObj.cartesian_motion_generator_start_elbow_invalid
      }
      else {
        this.cartesian_motion_generator_start_elbow_invalid = false;
      }
      if (initObj.hasOwnProperty('cartesian_motion_generator_joint_position_limits_violation')) {
        this.cartesian_motion_generator_joint_position_limits_violation = initObj.cartesian_motion_generator_joint_position_limits_violation
      }
      else {
        this.cartesian_motion_generator_joint_position_limits_violation = false;
      }
      if (initObj.hasOwnProperty('cartesian_motion_generator_joint_velocity_limits_violation')) {
        this.cartesian_motion_generator_joint_velocity_limits_violation = initObj.cartesian_motion_generator_joint_velocity_limits_violation
      }
      else {
        this.cartesian_motion_generator_joint_velocity_limits_violation = false;
      }
      if (initObj.hasOwnProperty('cartesian_motion_generator_joint_velocity_discontinuity')) {
        this.cartesian_motion_generator_joint_velocity_discontinuity = initObj.cartesian_motion_generator_joint_velocity_discontinuity
      }
      else {
        this.cartesian_motion_generator_joint_velocity_discontinuity = false;
      }
      if (initObj.hasOwnProperty('cartesian_motion_generator_joint_acceleration_discontinuity')) {
        this.cartesian_motion_generator_joint_acceleration_discontinuity = initObj.cartesian_motion_generator_joint_acceleration_discontinuity
      }
      else {
        this.cartesian_motion_generator_joint_acceleration_discontinuity = false;
      }
      if (initObj.hasOwnProperty('cartesian_position_motion_generator_invalid_frame')) {
        this.cartesian_position_motion_generator_invalid_frame = initObj.cartesian_position_motion_generator_invalid_frame
      }
      else {
        this.cartesian_position_motion_generator_invalid_frame = false;
      }
      if (initObj.hasOwnProperty('force_controller_desired_force_tolerance_violation')) {
        this.force_controller_desired_force_tolerance_violation = initObj.force_controller_desired_force_tolerance_violation
      }
      else {
        this.force_controller_desired_force_tolerance_violation = false;
      }
      if (initObj.hasOwnProperty('controller_torque_discontinuity')) {
        this.controller_torque_discontinuity = initObj.controller_torque_discontinuity
      }
      else {
        this.controller_torque_discontinuity = false;
      }
      if (initObj.hasOwnProperty('start_elbow_sign_inconsistent')) {
        this.start_elbow_sign_inconsistent = initObj.start_elbow_sign_inconsistent
      }
      else {
        this.start_elbow_sign_inconsistent = false;
      }
      if (initObj.hasOwnProperty('communication_constraints_violation')) {
        this.communication_constraints_violation = initObj.communication_constraints_violation
      }
      else {
        this.communication_constraints_violation = false;
      }
      if (initObj.hasOwnProperty('power_limit_violation')) {
        this.power_limit_violation = initObj.power_limit_violation
      }
      else {
        this.power_limit_violation = false;
      }
      if (initObj.hasOwnProperty('joint_p2p_insufficient_torque_for_planning')) {
        this.joint_p2p_insufficient_torque_for_planning = initObj.joint_p2p_insufficient_torque_for_planning
      }
      else {
        this.joint_p2p_insufficient_torque_for_planning = false;
      }
      if (initObj.hasOwnProperty('tau_j_range_violation')) {
        this.tau_j_range_violation = initObj.tau_j_range_violation
      }
      else {
        this.tau_j_range_violation = false;
      }
      if (initObj.hasOwnProperty('instability_detected')) {
        this.instability_detected = initObj.instability_detected
      }
      else {
        this.instability_detected = false;
      }
      if (initObj.hasOwnProperty('joint_move_in_wrong_direction')) {
        this.joint_move_in_wrong_direction = initObj.joint_move_in_wrong_direction
      }
      else {
        this.joint_move_in_wrong_direction = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Errors
    // Serialize message field [joint_position_limits_violation]
    bufferOffset = _serializer.bool(obj.joint_position_limits_violation, buffer, bufferOffset);
    // Serialize message field [cartesian_position_limits_violation]
    bufferOffset = _serializer.bool(obj.cartesian_position_limits_violation, buffer, bufferOffset);
    // Serialize message field [self_collision_avoidance_violation]
    bufferOffset = _serializer.bool(obj.self_collision_avoidance_violation, buffer, bufferOffset);
    // Serialize message field [joint_velocity_violation]
    bufferOffset = _serializer.bool(obj.joint_velocity_violation, buffer, bufferOffset);
    // Serialize message field [cartesian_velocity_violation]
    bufferOffset = _serializer.bool(obj.cartesian_velocity_violation, buffer, bufferOffset);
    // Serialize message field [force_control_safety_violation]
    bufferOffset = _serializer.bool(obj.force_control_safety_violation, buffer, bufferOffset);
    // Serialize message field [joint_reflex]
    bufferOffset = _serializer.bool(obj.joint_reflex, buffer, bufferOffset);
    // Serialize message field [cartesian_reflex]
    bufferOffset = _serializer.bool(obj.cartesian_reflex, buffer, bufferOffset);
    // Serialize message field [max_goal_pose_deviation_violation]
    bufferOffset = _serializer.bool(obj.max_goal_pose_deviation_violation, buffer, bufferOffset);
    // Serialize message field [max_path_pose_deviation_violation]
    bufferOffset = _serializer.bool(obj.max_path_pose_deviation_violation, buffer, bufferOffset);
    // Serialize message field [cartesian_velocity_profile_safety_violation]
    bufferOffset = _serializer.bool(obj.cartesian_velocity_profile_safety_violation, buffer, bufferOffset);
    // Serialize message field [joint_position_motion_generator_start_pose_invalid]
    bufferOffset = _serializer.bool(obj.joint_position_motion_generator_start_pose_invalid, buffer, bufferOffset);
    // Serialize message field [joint_motion_generator_position_limits_violation]
    bufferOffset = _serializer.bool(obj.joint_motion_generator_position_limits_violation, buffer, bufferOffset);
    // Serialize message field [joint_motion_generator_velocity_limits_violation]
    bufferOffset = _serializer.bool(obj.joint_motion_generator_velocity_limits_violation, buffer, bufferOffset);
    // Serialize message field [joint_motion_generator_velocity_discontinuity]
    bufferOffset = _serializer.bool(obj.joint_motion_generator_velocity_discontinuity, buffer, bufferOffset);
    // Serialize message field [joint_motion_generator_acceleration_discontinuity]
    bufferOffset = _serializer.bool(obj.joint_motion_generator_acceleration_discontinuity, buffer, bufferOffset);
    // Serialize message field [cartesian_position_motion_generator_start_pose_invalid]
    bufferOffset = _serializer.bool(obj.cartesian_position_motion_generator_start_pose_invalid, buffer, bufferOffset);
    // Serialize message field [cartesian_motion_generator_elbow_limit_violation]
    bufferOffset = _serializer.bool(obj.cartesian_motion_generator_elbow_limit_violation, buffer, bufferOffset);
    // Serialize message field [cartesian_motion_generator_velocity_limits_violation]
    bufferOffset = _serializer.bool(obj.cartesian_motion_generator_velocity_limits_violation, buffer, bufferOffset);
    // Serialize message field [cartesian_motion_generator_velocity_discontinuity]
    bufferOffset = _serializer.bool(obj.cartesian_motion_generator_velocity_discontinuity, buffer, bufferOffset);
    // Serialize message field [cartesian_motion_generator_acceleration_discontinuity]
    bufferOffset = _serializer.bool(obj.cartesian_motion_generator_acceleration_discontinuity, buffer, bufferOffset);
    // Serialize message field [cartesian_motion_generator_elbow_sign_inconsistent]
    bufferOffset = _serializer.bool(obj.cartesian_motion_generator_elbow_sign_inconsistent, buffer, bufferOffset);
    // Serialize message field [cartesian_motion_generator_start_elbow_invalid]
    bufferOffset = _serializer.bool(obj.cartesian_motion_generator_start_elbow_invalid, buffer, bufferOffset);
    // Serialize message field [cartesian_motion_generator_joint_position_limits_violation]
    bufferOffset = _serializer.bool(obj.cartesian_motion_generator_joint_position_limits_violation, buffer, bufferOffset);
    // Serialize message field [cartesian_motion_generator_joint_velocity_limits_violation]
    bufferOffset = _serializer.bool(obj.cartesian_motion_generator_joint_velocity_limits_violation, buffer, bufferOffset);
    // Serialize message field [cartesian_motion_generator_joint_velocity_discontinuity]
    bufferOffset = _serializer.bool(obj.cartesian_motion_generator_joint_velocity_discontinuity, buffer, bufferOffset);
    // Serialize message field [cartesian_motion_generator_joint_acceleration_discontinuity]
    bufferOffset = _serializer.bool(obj.cartesian_motion_generator_joint_acceleration_discontinuity, buffer, bufferOffset);
    // Serialize message field [cartesian_position_motion_generator_invalid_frame]
    bufferOffset = _serializer.bool(obj.cartesian_position_motion_generator_invalid_frame, buffer, bufferOffset);
    // Serialize message field [force_controller_desired_force_tolerance_violation]
    bufferOffset = _serializer.bool(obj.force_controller_desired_force_tolerance_violation, buffer, bufferOffset);
    // Serialize message field [controller_torque_discontinuity]
    bufferOffset = _serializer.bool(obj.controller_torque_discontinuity, buffer, bufferOffset);
    // Serialize message field [start_elbow_sign_inconsistent]
    bufferOffset = _serializer.bool(obj.start_elbow_sign_inconsistent, buffer, bufferOffset);
    // Serialize message field [communication_constraints_violation]
    bufferOffset = _serializer.bool(obj.communication_constraints_violation, buffer, bufferOffset);
    // Serialize message field [power_limit_violation]
    bufferOffset = _serializer.bool(obj.power_limit_violation, buffer, bufferOffset);
    // Serialize message field [joint_p2p_insufficient_torque_for_planning]
    bufferOffset = _serializer.bool(obj.joint_p2p_insufficient_torque_for_planning, buffer, bufferOffset);
    // Serialize message field [tau_j_range_violation]
    bufferOffset = _serializer.bool(obj.tau_j_range_violation, buffer, bufferOffset);
    // Serialize message field [instability_detected]
    bufferOffset = _serializer.bool(obj.instability_detected, buffer, bufferOffset);
    // Serialize message field [joint_move_in_wrong_direction]
    bufferOffset = _serializer.bool(obj.joint_move_in_wrong_direction, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Errors
    let len;
    let data = new Errors(null);
    // Deserialize message field [joint_position_limits_violation]
    data.joint_position_limits_violation = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [cartesian_position_limits_violation]
    data.cartesian_position_limits_violation = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [self_collision_avoidance_violation]
    data.self_collision_avoidance_violation = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [joint_velocity_violation]
    data.joint_velocity_violation = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [cartesian_velocity_violation]
    data.cartesian_velocity_violation = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [force_control_safety_violation]
    data.force_control_safety_violation = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [joint_reflex]
    data.joint_reflex = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [cartesian_reflex]
    data.cartesian_reflex = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [max_goal_pose_deviation_violation]
    data.max_goal_pose_deviation_violation = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [max_path_pose_deviation_violation]
    data.max_path_pose_deviation_violation = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [cartesian_velocity_profile_safety_violation]
    data.cartesian_velocity_profile_safety_violation = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [joint_position_motion_generator_start_pose_invalid]
    data.joint_position_motion_generator_start_pose_invalid = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [joint_motion_generator_position_limits_violation]
    data.joint_motion_generator_position_limits_violation = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [joint_motion_generator_velocity_limits_violation]
    data.joint_motion_generator_velocity_limits_violation = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [joint_motion_generator_velocity_discontinuity]
    data.joint_motion_generator_velocity_discontinuity = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [joint_motion_generator_acceleration_discontinuity]
    data.joint_motion_generator_acceleration_discontinuity = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [cartesian_position_motion_generator_start_pose_invalid]
    data.cartesian_position_motion_generator_start_pose_invalid = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [cartesian_motion_generator_elbow_limit_violation]
    data.cartesian_motion_generator_elbow_limit_violation = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [cartesian_motion_generator_velocity_limits_violation]
    data.cartesian_motion_generator_velocity_limits_violation = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [cartesian_motion_generator_velocity_discontinuity]
    data.cartesian_motion_generator_velocity_discontinuity = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [cartesian_motion_generator_acceleration_discontinuity]
    data.cartesian_motion_generator_acceleration_discontinuity = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [cartesian_motion_generator_elbow_sign_inconsistent]
    data.cartesian_motion_generator_elbow_sign_inconsistent = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [cartesian_motion_generator_start_elbow_invalid]
    data.cartesian_motion_generator_start_elbow_invalid = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [cartesian_motion_generator_joint_position_limits_violation]
    data.cartesian_motion_generator_joint_position_limits_violation = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [cartesian_motion_generator_joint_velocity_limits_violation]
    data.cartesian_motion_generator_joint_velocity_limits_violation = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [cartesian_motion_generator_joint_velocity_discontinuity]
    data.cartesian_motion_generator_joint_velocity_discontinuity = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [cartesian_motion_generator_joint_acceleration_discontinuity]
    data.cartesian_motion_generator_joint_acceleration_discontinuity = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [cartesian_position_motion_generator_invalid_frame]
    data.cartesian_position_motion_generator_invalid_frame = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [force_controller_desired_force_tolerance_violation]
    data.force_controller_desired_force_tolerance_violation = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [controller_torque_discontinuity]
    data.controller_torque_discontinuity = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [start_elbow_sign_inconsistent]
    data.start_elbow_sign_inconsistent = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [communication_constraints_violation]
    data.communication_constraints_violation = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [power_limit_violation]
    data.power_limit_violation = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [joint_p2p_insufficient_torque_for_planning]
    data.joint_p2p_insufficient_torque_for_planning = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [tau_j_range_violation]
    data.tau_j_range_violation = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [instability_detected]
    data.instability_detected = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [joint_move_in_wrong_direction]
    data.joint_move_in_wrong_direction = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 37;
  }

  static datatype() {
    // Returns string type for a message object
    return 'franka_interface_msgs/Errors';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b93c14a039f7f69d7321c619961e4708';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new Errors(null);
    if (msg.joint_position_limits_violation !== undefined) {
      resolved.joint_position_limits_violation = msg.joint_position_limits_violation;
    }
    else {
      resolved.joint_position_limits_violation = false
    }

    if (msg.cartesian_position_limits_violation !== undefined) {
      resolved.cartesian_position_limits_violation = msg.cartesian_position_limits_violation;
    }
    else {
      resolved.cartesian_position_limits_violation = false
    }

    if (msg.self_collision_avoidance_violation !== undefined) {
      resolved.self_collision_avoidance_violation = msg.self_collision_avoidance_violation;
    }
    else {
      resolved.self_collision_avoidance_violation = false
    }

    if (msg.joint_velocity_violation !== undefined) {
      resolved.joint_velocity_violation = msg.joint_velocity_violation;
    }
    else {
      resolved.joint_velocity_violation = false
    }

    if (msg.cartesian_velocity_violation !== undefined) {
      resolved.cartesian_velocity_violation = msg.cartesian_velocity_violation;
    }
    else {
      resolved.cartesian_velocity_violation = false
    }

    if (msg.force_control_safety_violation !== undefined) {
      resolved.force_control_safety_violation = msg.force_control_safety_violation;
    }
    else {
      resolved.force_control_safety_violation = false
    }

    if (msg.joint_reflex !== undefined) {
      resolved.joint_reflex = msg.joint_reflex;
    }
    else {
      resolved.joint_reflex = false
    }

    if (msg.cartesian_reflex !== undefined) {
      resolved.cartesian_reflex = msg.cartesian_reflex;
    }
    else {
      resolved.cartesian_reflex = false
    }

    if (msg.max_goal_pose_deviation_violation !== undefined) {
      resolved.max_goal_pose_deviation_violation = msg.max_goal_pose_deviation_violation;
    }
    else {
      resolved.max_goal_pose_deviation_violation = false
    }

    if (msg.max_path_pose_deviation_violation !== undefined) {
      resolved.max_path_pose_deviation_violation = msg.max_path_pose_deviation_violation;
    }
    else {
      resolved.max_path_pose_deviation_violation = false
    }

    if (msg.cartesian_velocity_profile_safety_violation !== undefined) {
      resolved.cartesian_velocity_profile_safety_violation = msg.cartesian_velocity_profile_safety_violation;
    }
    else {
      resolved.cartesian_velocity_profile_safety_violation = false
    }

    if (msg.joint_position_motion_generator_start_pose_invalid !== undefined) {
      resolved.joint_position_motion_generator_start_pose_invalid = msg.joint_position_motion_generator_start_pose_invalid;
    }
    else {
      resolved.joint_position_motion_generator_start_pose_invalid = false
    }

    if (msg.joint_motion_generator_position_limits_violation !== undefined) {
      resolved.joint_motion_generator_position_limits_violation = msg.joint_motion_generator_position_limits_violation;
    }
    else {
      resolved.joint_motion_generator_position_limits_violation = false
    }

    if (msg.joint_motion_generator_velocity_limits_violation !== undefined) {
      resolved.joint_motion_generator_velocity_limits_violation = msg.joint_motion_generator_velocity_limits_violation;
    }
    else {
      resolved.joint_motion_generator_velocity_limits_violation = false
    }

    if (msg.joint_motion_generator_velocity_discontinuity !== undefined) {
      resolved.joint_motion_generator_velocity_discontinuity = msg.joint_motion_generator_velocity_discontinuity;
    }
    else {
      resolved.joint_motion_generator_velocity_discontinuity = false
    }

    if (msg.joint_motion_generator_acceleration_discontinuity !== undefined) {
      resolved.joint_motion_generator_acceleration_discontinuity = msg.joint_motion_generator_acceleration_discontinuity;
    }
    else {
      resolved.joint_motion_generator_acceleration_discontinuity = false
    }

    if (msg.cartesian_position_motion_generator_start_pose_invalid !== undefined) {
      resolved.cartesian_position_motion_generator_start_pose_invalid = msg.cartesian_position_motion_generator_start_pose_invalid;
    }
    else {
      resolved.cartesian_position_motion_generator_start_pose_invalid = false
    }

    if (msg.cartesian_motion_generator_elbow_limit_violation !== undefined) {
      resolved.cartesian_motion_generator_elbow_limit_violation = msg.cartesian_motion_generator_elbow_limit_violation;
    }
    else {
      resolved.cartesian_motion_generator_elbow_limit_violation = false
    }

    if (msg.cartesian_motion_generator_velocity_limits_violation !== undefined) {
      resolved.cartesian_motion_generator_velocity_limits_violation = msg.cartesian_motion_generator_velocity_limits_violation;
    }
    else {
      resolved.cartesian_motion_generator_velocity_limits_violation = false
    }

    if (msg.cartesian_motion_generator_velocity_discontinuity !== undefined) {
      resolved.cartesian_motion_generator_velocity_discontinuity = msg.cartesian_motion_generator_velocity_discontinuity;
    }
    else {
      resolved.cartesian_motion_generator_velocity_discontinuity = false
    }

    if (msg.cartesian_motion_generator_acceleration_discontinuity !== undefined) {
      resolved.cartesian_motion_generator_acceleration_discontinuity = msg.cartesian_motion_generator_acceleration_discontinuity;
    }
    else {
      resolved.cartesian_motion_generator_acceleration_discontinuity = false
    }

    if (msg.cartesian_motion_generator_elbow_sign_inconsistent !== undefined) {
      resolved.cartesian_motion_generator_elbow_sign_inconsistent = msg.cartesian_motion_generator_elbow_sign_inconsistent;
    }
    else {
      resolved.cartesian_motion_generator_elbow_sign_inconsistent = false
    }

    if (msg.cartesian_motion_generator_start_elbow_invalid !== undefined) {
      resolved.cartesian_motion_generator_start_elbow_invalid = msg.cartesian_motion_generator_start_elbow_invalid;
    }
    else {
      resolved.cartesian_motion_generator_start_elbow_invalid = false
    }

    if (msg.cartesian_motion_generator_joint_position_limits_violation !== undefined) {
      resolved.cartesian_motion_generator_joint_position_limits_violation = msg.cartesian_motion_generator_joint_position_limits_violation;
    }
    else {
      resolved.cartesian_motion_generator_joint_position_limits_violation = false
    }

    if (msg.cartesian_motion_generator_joint_velocity_limits_violation !== undefined) {
      resolved.cartesian_motion_generator_joint_velocity_limits_violation = msg.cartesian_motion_generator_joint_velocity_limits_violation;
    }
    else {
      resolved.cartesian_motion_generator_joint_velocity_limits_violation = false
    }

    if (msg.cartesian_motion_generator_joint_velocity_discontinuity !== undefined) {
      resolved.cartesian_motion_generator_joint_velocity_discontinuity = msg.cartesian_motion_generator_joint_velocity_discontinuity;
    }
    else {
      resolved.cartesian_motion_generator_joint_velocity_discontinuity = false
    }

    if (msg.cartesian_motion_generator_joint_acceleration_discontinuity !== undefined) {
      resolved.cartesian_motion_generator_joint_acceleration_discontinuity = msg.cartesian_motion_generator_joint_acceleration_discontinuity;
    }
    else {
      resolved.cartesian_motion_generator_joint_acceleration_discontinuity = false
    }

    if (msg.cartesian_position_motion_generator_invalid_frame !== undefined) {
      resolved.cartesian_position_motion_generator_invalid_frame = msg.cartesian_position_motion_generator_invalid_frame;
    }
    else {
      resolved.cartesian_position_motion_generator_invalid_frame = false
    }

    if (msg.force_controller_desired_force_tolerance_violation !== undefined) {
      resolved.force_controller_desired_force_tolerance_violation = msg.force_controller_desired_force_tolerance_violation;
    }
    else {
      resolved.force_controller_desired_force_tolerance_violation = false
    }

    if (msg.controller_torque_discontinuity !== undefined) {
      resolved.controller_torque_discontinuity = msg.controller_torque_discontinuity;
    }
    else {
      resolved.controller_torque_discontinuity = false
    }

    if (msg.start_elbow_sign_inconsistent !== undefined) {
      resolved.start_elbow_sign_inconsistent = msg.start_elbow_sign_inconsistent;
    }
    else {
      resolved.start_elbow_sign_inconsistent = false
    }

    if (msg.communication_constraints_violation !== undefined) {
      resolved.communication_constraints_violation = msg.communication_constraints_violation;
    }
    else {
      resolved.communication_constraints_violation = false
    }

    if (msg.power_limit_violation !== undefined) {
      resolved.power_limit_violation = msg.power_limit_violation;
    }
    else {
      resolved.power_limit_violation = false
    }

    if (msg.joint_p2p_insufficient_torque_for_planning !== undefined) {
      resolved.joint_p2p_insufficient_torque_for_planning = msg.joint_p2p_insufficient_torque_for_planning;
    }
    else {
      resolved.joint_p2p_insufficient_torque_for_planning = false
    }

    if (msg.tau_j_range_violation !== undefined) {
      resolved.tau_j_range_violation = msg.tau_j_range_violation;
    }
    else {
      resolved.tau_j_range_violation = false
    }

    if (msg.instability_detected !== undefined) {
      resolved.instability_detected = msg.instability_detected;
    }
    else {
      resolved.instability_detected = false
    }

    if (msg.joint_move_in_wrong_direction !== undefined) {
      resolved.joint_move_in_wrong_direction = msg.joint_move_in_wrong_direction;
    }
    else {
      resolved.joint_move_in_wrong_direction = false
    }

    return resolved;
    }
};

module.exports = Errors;

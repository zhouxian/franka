// Auto-generated. Do not edit!

// (in-package franka_interface_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Errors = require('./Errors.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class RobotState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.pose_desired = null;
      this.O_T_EE = null;
      this.O_T_EE_d = null;
      this.F_T_EE = null;
      this.EE_T_K = null;
      this.m_ee = null;
      this.I_ee = null;
      this.F_x_Cee = null;
      this.m_load = null;
      this.I_load = null;
      this.F_x_Cload = null;
      this.m_total = null;
      this.I_total = null;
      this.F_x_Ctotal = null;
      this.elbow = null;
      this.elbow_d = null;
      this.elbow_c = null;
      this.delbow_c = null;
      this.ddelbow_c = null;
      this.tau_J = null;
      this.tau_J_d = null;
      this.dtau_J = null;
      this.q = null;
      this.q_d = null;
      this.dq = null;
      this.dq_d = null;
      this.ddq_d = null;
      this.joint_contact = null;
      this.cartesian_contact = null;
      this.joint_collision = null;
      this.cartesian_collision = null;
      this.tau_ext_hat_filtered = null;
      this.O_F_ext_hat_K = null;
      this.K_F_ext_hat_K = null;
      this.O_dP_EE_d = null;
      this.O_T_EE_c = null;
      this.O_dP_EE_c = null;
      this.O_ddP_EE_c = null;
      this.theta = null;
      this.dtheta = null;
      this.current_errors = null;
      this.last_motion_errors = null;
      this.control_command_success_rate = null;
      this.robot_mode = null;
      this.robot_time = null;
      this.gripper_width = null;
      this.gripper_max_width = null;
      this.gripper_is_grasped = null;
      this.gripper_temperature = null;
      this.gripper_time = null;
      this.is_fresh = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('pose_desired')) {
        this.pose_desired = initObj.pose_desired
      }
      else {
        this.pose_desired = new Array(16).fill(0);
      }
      if (initObj.hasOwnProperty('O_T_EE')) {
        this.O_T_EE = initObj.O_T_EE
      }
      else {
        this.O_T_EE = new Array(16).fill(0);
      }
      if (initObj.hasOwnProperty('O_T_EE_d')) {
        this.O_T_EE_d = initObj.O_T_EE_d
      }
      else {
        this.O_T_EE_d = new Array(16).fill(0);
      }
      if (initObj.hasOwnProperty('F_T_EE')) {
        this.F_T_EE = initObj.F_T_EE
      }
      else {
        this.F_T_EE = new Array(16).fill(0);
      }
      if (initObj.hasOwnProperty('EE_T_K')) {
        this.EE_T_K = initObj.EE_T_K
      }
      else {
        this.EE_T_K = new Array(16).fill(0);
      }
      if (initObj.hasOwnProperty('m_ee')) {
        this.m_ee = initObj.m_ee
      }
      else {
        this.m_ee = 0.0;
      }
      if (initObj.hasOwnProperty('I_ee')) {
        this.I_ee = initObj.I_ee
      }
      else {
        this.I_ee = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('F_x_Cee')) {
        this.F_x_Cee = initObj.F_x_Cee
      }
      else {
        this.F_x_Cee = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('m_load')) {
        this.m_load = initObj.m_load
      }
      else {
        this.m_load = 0.0;
      }
      if (initObj.hasOwnProperty('I_load')) {
        this.I_load = initObj.I_load
      }
      else {
        this.I_load = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('F_x_Cload')) {
        this.F_x_Cload = initObj.F_x_Cload
      }
      else {
        this.F_x_Cload = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('m_total')) {
        this.m_total = initObj.m_total
      }
      else {
        this.m_total = 0.0;
      }
      if (initObj.hasOwnProperty('I_total')) {
        this.I_total = initObj.I_total
      }
      else {
        this.I_total = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('F_x_Ctotal')) {
        this.F_x_Ctotal = initObj.F_x_Ctotal
      }
      else {
        this.F_x_Ctotal = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('elbow')) {
        this.elbow = initObj.elbow
      }
      else {
        this.elbow = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('elbow_d')) {
        this.elbow_d = initObj.elbow_d
      }
      else {
        this.elbow_d = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('elbow_c')) {
        this.elbow_c = initObj.elbow_c
      }
      else {
        this.elbow_c = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('delbow_c')) {
        this.delbow_c = initObj.delbow_c
      }
      else {
        this.delbow_c = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('ddelbow_c')) {
        this.ddelbow_c = initObj.ddelbow_c
      }
      else {
        this.ddelbow_c = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('tau_J')) {
        this.tau_J = initObj.tau_J
      }
      else {
        this.tau_J = new Array(7).fill(0);
      }
      if (initObj.hasOwnProperty('tau_J_d')) {
        this.tau_J_d = initObj.tau_J_d
      }
      else {
        this.tau_J_d = new Array(7).fill(0);
      }
      if (initObj.hasOwnProperty('dtau_J')) {
        this.dtau_J = initObj.dtau_J
      }
      else {
        this.dtau_J = new Array(7).fill(0);
      }
      if (initObj.hasOwnProperty('q')) {
        this.q = initObj.q
      }
      else {
        this.q = new Array(7).fill(0);
      }
      if (initObj.hasOwnProperty('q_d')) {
        this.q_d = initObj.q_d
      }
      else {
        this.q_d = new Array(7).fill(0);
      }
      if (initObj.hasOwnProperty('dq')) {
        this.dq = initObj.dq
      }
      else {
        this.dq = new Array(7).fill(0);
      }
      if (initObj.hasOwnProperty('dq_d')) {
        this.dq_d = initObj.dq_d
      }
      else {
        this.dq_d = new Array(7).fill(0);
      }
      if (initObj.hasOwnProperty('ddq_d')) {
        this.ddq_d = initObj.ddq_d
      }
      else {
        this.ddq_d = new Array(7).fill(0);
      }
      if (initObj.hasOwnProperty('joint_contact')) {
        this.joint_contact = initObj.joint_contact
      }
      else {
        this.joint_contact = new Array(7).fill(0);
      }
      if (initObj.hasOwnProperty('cartesian_contact')) {
        this.cartesian_contact = initObj.cartesian_contact
      }
      else {
        this.cartesian_contact = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('joint_collision')) {
        this.joint_collision = initObj.joint_collision
      }
      else {
        this.joint_collision = new Array(7).fill(0);
      }
      if (initObj.hasOwnProperty('cartesian_collision')) {
        this.cartesian_collision = initObj.cartesian_collision
      }
      else {
        this.cartesian_collision = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('tau_ext_hat_filtered')) {
        this.tau_ext_hat_filtered = initObj.tau_ext_hat_filtered
      }
      else {
        this.tau_ext_hat_filtered = new Array(7).fill(0);
      }
      if (initObj.hasOwnProperty('O_F_ext_hat_K')) {
        this.O_F_ext_hat_K = initObj.O_F_ext_hat_K
      }
      else {
        this.O_F_ext_hat_K = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('K_F_ext_hat_K')) {
        this.K_F_ext_hat_K = initObj.K_F_ext_hat_K
      }
      else {
        this.K_F_ext_hat_K = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('O_dP_EE_d')) {
        this.O_dP_EE_d = initObj.O_dP_EE_d
      }
      else {
        this.O_dP_EE_d = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('O_T_EE_c')) {
        this.O_T_EE_c = initObj.O_T_EE_c
      }
      else {
        this.O_T_EE_c = new Array(16).fill(0);
      }
      if (initObj.hasOwnProperty('O_dP_EE_c')) {
        this.O_dP_EE_c = initObj.O_dP_EE_c
      }
      else {
        this.O_dP_EE_c = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('O_ddP_EE_c')) {
        this.O_ddP_EE_c = initObj.O_ddP_EE_c
      }
      else {
        this.O_ddP_EE_c = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('theta')) {
        this.theta = initObj.theta
      }
      else {
        this.theta = new Array(7).fill(0);
      }
      if (initObj.hasOwnProperty('dtheta')) {
        this.dtheta = initObj.dtheta
      }
      else {
        this.dtheta = new Array(7).fill(0);
      }
      if (initObj.hasOwnProperty('current_errors')) {
        this.current_errors = initObj.current_errors
      }
      else {
        this.current_errors = new Errors();
      }
      if (initObj.hasOwnProperty('last_motion_errors')) {
        this.last_motion_errors = initObj.last_motion_errors
      }
      else {
        this.last_motion_errors = new Errors();
      }
      if (initObj.hasOwnProperty('control_command_success_rate')) {
        this.control_command_success_rate = initObj.control_command_success_rate
      }
      else {
        this.control_command_success_rate = 0.0;
      }
      if (initObj.hasOwnProperty('robot_mode')) {
        this.robot_mode = initObj.robot_mode
      }
      else {
        this.robot_mode = 0;
      }
      if (initObj.hasOwnProperty('robot_time')) {
        this.robot_time = initObj.robot_time
      }
      else {
        this.robot_time = 0.0;
      }
      if (initObj.hasOwnProperty('gripper_width')) {
        this.gripper_width = initObj.gripper_width
      }
      else {
        this.gripper_width = 0.0;
      }
      if (initObj.hasOwnProperty('gripper_max_width')) {
        this.gripper_max_width = initObj.gripper_max_width
      }
      else {
        this.gripper_max_width = 0.0;
      }
      if (initObj.hasOwnProperty('gripper_is_grasped')) {
        this.gripper_is_grasped = initObj.gripper_is_grasped
      }
      else {
        this.gripper_is_grasped = false;
      }
      if (initObj.hasOwnProperty('gripper_temperature')) {
        this.gripper_temperature = initObj.gripper_temperature
      }
      else {
        this.gripper_temperature = 0;
      }
      if (initObj.hasOwnProperty('gripper_time')) {
        this.gripper_time = initObj.gripper_time
      }
      else {
        this.gripper_time = 0.0;
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
    // Serializes a message object of type RobotState
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Check that the constant length array field [pose_desired] has the right length
    if (obj.pose_desired.length !== 16) {
      throw new Error('Unable to serialize array field pose_desired - length must be 16')
    }
    // Serialize message field [pose_desired]
    bufferOffset = _arraySerializer.float64(obj.pose_desired, buffer, bufferOffset, 16);
    // Check that the constant length array field [O_T_EE] has the right length
    if (obj.O_T_EE.length !== 16) {
      throw new Error('Unable to serialize array field O_T_EE - length must be 16')
    }
    // Serialize message field [O_T_EE]
    bufferOffset = _arraySerializer.float64(obj.O_T_EE, buffer, bufferOffset, 16);
    // Check that the constant length array field [O_T_EE_d] has the right length
    if (obj.O_T_EE_d.length !== 16) {
      throw new Error('Unable to serialize array field O_T_EE_d - length must be 16')
    }
    // Serialize message field [O_T_EE_d]
    bufferOffset = _arraySerializer.float64(obj.O_T_EE_d, buffer, bufferOffset, 16);
    // Check that the constant length array field [F_T_EE] has the right length
    if (obj.F_T_EE.length !== 16) {
      throw new Error('Unable to serialize array field F_T_EE - length must be 16')
    }
    // Serialize message field [F_T_EE]
    bufferOffset = _arraySerializer.float64(obj.F_T_EE, buffer, bufferOffset, 16);
    // Check that the constant length array field [EE_T_K] has the right length
    if (obj.EE_T_K.length !== 16) {
      throw new Error('Unable to serialize array field EE_T_K - length must be 16')
    }
    // Serialize message field [EE_T_K]
    bufferOffset = _arraySerializer.float64(obj.EE_T_K, buffer, bufferOffset, 16);
    // Serialize message field [m_ee]
    bufferOffset = _serializer.float64(obj.m_ee, buffer, bufferOffset);
    // Check that the constant length array field [I_ee] has the right length
    if (obj.I_ee.length !== 9) {
      throw new Error('Unable to serialize array field I_ee - length must be 9')
    }
    // Serialize message field [I_ee]
    bufferOffset = _arraySerializer.float64(obj.I_ee, buffer, bufferOffset, 9);
    // Check that the constant length array field [F_x_Cee] has the right length
    if (obj.F_x_Cee.length !== 3) {
      throw new Error('Unable to serialize array field F_x_Cee - length must be 3')
    }
    // Serialize message field [F_x_Cee]
    bufferOffset = _arraySerializer.float64(obj.F_x_Cee, buffer, bufferOffset, 3);
    // Serialize message field [m_load]
    bufferOffset = _serializer.float64(obj.m_load, buffer, bufferOffset);
    // Check that the constant length array field [I_load] has the right length
    if (obj.I_load.length !== 9) {
      throw new Error('Unable to serialize array field I_load - length must be 9')
    }
    // Serialize message field [I_load]
    bufferOffset = _arraySerializer.float64(obj.I_load, buffer, bufferOffset, 9);
    // Check that the constant length array field [F_x_Cload] has the right length
    if (obj.F_x_Cload.length !== 3) {
      throw new Error('Unable to serialize array field F_x_Cload - length must be 3')
    }
    // Serialize message field [F_x_Cload]
    bufferOffset = _arraySerializer.float64(obj.F_x_Cload, buffer, bufferOffset, 3);
    // Serialize message field [m_total]
    bufferOffset = _serializer.float64(obj.m_total, buffer, bufferOffset);
    // Check that the constant length array field [I_total] has the right length
    if (obj.I_total.length !== 9) {
      throw new Error('Unable to serialize array field I_total - length must be 9')
    }
    // Serialize message field [I_total]
    bufferOffset = _arraySerializer.float64(obj.I_total, buffer, bufferOffset, 9);
    // Check that the constant length array field [F_x_Ctotal] has the right length
    if (obj.F_x_Ctotal.length !== 3) {
      throw new Error('Unable to serialize array field F_x_Ctotal - length must be 3')
    }
    // Serialize message field [F_x_Ctotal]
    bufferOffset = _arraySerializer.float64(obj.F_x_Ctotal, buffer, bufferOffset, 3);
    // Check that the constant length array field [elbow] has the right length
    if (obj.elbow.length !== 2) {
      throw new Error('Unable to serialize array field elbow - length must be 2')
    }
    // Serialize message field [elbow]
    bufferOffset = _arraySerializer.float64(obj.elbow, buffer, bufferOffset, 2);
    // Check that the constant length array field [elbow_d] has the right length
    if (obj.elbow_d.length !== 2) {
      throw new Error('Unable to serialize array field elbow_d - length must be 2')
    }
    // Serialize message field [elbow_d]
    bufferOffset = _arraySerializer.float64(obj.elbow_d, buffer, bufferOffset, 2);
    // Check that the constant length array field [elbow_c] has the right length
    if (obj.elbow_c.length !== 2) {
      throw new Error('Unable to serialize array field elbow_c - length must be 2')
    }
    // Serialize message field [elbow_c]
    bufferOffset = _arraySerializer.float64(obj.elbow_c, buffer, bufferOffset, 2);
    // Check that the constant length array field [delbow_c] has the right length
    if (obj.delbow_c.length !== 2) {
      throw new Error('Unable to serialize array field delbow_c - length must be 2')
    }
    // Serialize message field [delbow_c]
    bufferOffset = _arraySerializer.float64(obj.delbow_c, buffer, bufferOffset, 2);
    // Check that the constant length array field [ddelbow_c] has the right length
    if (obj.ddelbow_c.length !== 2) {
      throw new Error('Unable to serialize array field ddelbow_c - length must be 2')
    }
    // Serialize message field [ddelbow_c]
    bufferOffset = _arraySerializer.float64(obj.ddelbow_c, buffer, bufferOffset, 2);
    // Check that the constant length array field [tau_J] has the right length
    if (obj.tau_J.length !== 7) {
      throw new Error('Unable to serialize array field tau_J - length must be 7')
    }
    // Serialize message field [tau_J]
    bufferOffset = _arraySerializer.float64(obj.tau_J, buffer, bufferOffset, 7);
    // Check that the constant length array field [tau_J_d] has the right length
    if (obj.tau_J_d.length !== 7) {
      throw new Error('Unable to serialize array field tau_J_d - length must be 7')
    }
    // Serialize message field [tau_J_d]
    bufferOffset = _arraySerializer.float64(obj.tau_J_d, buffer, bufferOffset, 7);
    // Check that the constant length array field [dtau_J] has the right length
    if (obj.dtau_J.length !== 7) {
      throw new Error('Unable to serialize array field dtau_J - length must be 7')
    }
    // Serialize message field [dtau_J]
    bufferOffset = _arraySerializer.float64(obj.dtau_J, buffer, bufferOffset, 7);
    // Check that the constant length array field [q] has the right length
    if (obj.q.length !== 7) {
      throw new Error('Unable to serialize array field q - length must be 7')
    }
    // Serialize message field [q]
    bufferOffset = _arraySerializer.float64(obj.q, buffer, bufferOffset, 7);
    // Check that the constant length array field [q_d] has the right length
    if (obj.q_d.length !== 7) {
      throw new Error('Unable to serialize array field q_d - length must be 7')
    }
    // Serialize message field [q_d]
    bufferOffset = _arraySerializer.float64(obj.q_d, buffer, bufferOffset, 7);
    // Check that the constant length array field [dq] has the right length
    if (obj.dq.length !== 7) {
      throw new Error('Unable to serialize array field dq - length must be 7')
    }
    // Serialize message field [dq]
    bufferOffset = _arraySerializer.float64(obj.dq, buffer, bufferOffset, 7);
    // Check that the constant length array field [dq_d] has the right length
    if (obj.dq_d.length !== 7) {
      throw new Error('Unable to serialize array field dq_d - length must be 7')
    }
    // Serialize message field [dq_d]
    bufferOffset = _arraySerializer.float64(obj.dq_d, buffer, bufferOffset, 7);
    // Check that the constant length array field [ddq_d] has the right length
    if (obj.ddq_d.length !== 7) {
      throw new Error('Unable to serialize array field ddq_d - length must be 7')
    }
    // Serialize message field [ddq_d]
    bufferOffset = _arraySerializer.float64(obj.ddq_d, buffer, bufferOffset, 7);
    // Check that the constant length array field [joint_contact] has the right length
    if (obj.joint_contact.length !== 7) {
      throw new Error('Unable to serialize array field joint_contact - length must be 7')
    }
    // Serialize message field [joint_contact]
    bufferOffset = _arraySerializer.float64(obj.joint_contact, buffer, bufferOffset, 7);
    // Check that the constant length array field [cartesian_contact] has the right length
    if (obj.cartesian_contact.length !== 6) {
      throw new Error('Unable to serialize array field cartesian_contact - length must be 6')
    }
    // Serialize message field [cartesian_contact]
    bufferOffset = _arraySerializer.float64(obj.cartesian_contact, buffer, bufferOffset, 6);
    // Check that the constant length array field [joint_collision] has the right length
    if (obj.joint_collision.length !== 7) {
      throw new Error('Unable to serialize array field joint_collision - length must be 7')
    }
    // Serialize message field [joint_collision]
    bufferOffset = _arraySerializer.float64(obj.joint_collision, buffer, bufferOffset, 7);
    // Check that the constant length array field [cartesian_collision] has the right length
    if (obj.cartesian_collision.length !== 6) {
      throw new Error('Unable to serialize array field cartesian_collision - length must be 6')
    }
    // Serialize message field [cartesian_collision]
    bufferOffset = _arraySerializer.float64(obj.cartesian_collision, buffer, bufferOffset, 6);
    // Check that the constant length array field [tau_ext_hat_filtered] has the right length
    if (obj.tau_ext_hat_filtered.length !== 7) {
      throw new Error('Unable to serialize array field tau_ext_hat_filtered - length must be 7')
    }
    // Serialize message field [tau_ext_hat_filtered]
    bufferOffset = _arraySerializer.float64(obj.tau_ext_hat_filtered, buffer, bufferOffset, 7);
    // Check that the constant length array field [O_F_ext_hat_K] has the right length
    if (obj.O_F_ext_hat_K.length !== 6) {
      throw new Error('Unable to serialize array field O_F_ext_hat_K - length must be 6')
    }
    // Serialize message field [O_F_ext_hat_K]
    bufferOffset = _arraySerializer.float64(obj.O_F_ext_hat_K, buffer, bufferOffset, 6);
    // Check that the constant length array field [K_F_ext_hat_K] has the right length
    if (obj.K_F_ext_hat_K.length !== 6) {
      throw new Error('Unable to serialize array field K_F_ext_hat_K - length must be 6')
    }
    // Serialize message field [K_F_ext_hat_K]
    bufferOffset = _arraySerializer.float64(obj.K_F_ext_hat_K, buffer, bufferOffset, 6);
    // Check that the constant length array field [O_dP_EE_d] has the right length
    if (obj.O_dP_EE_d.length !== 6) {
      throw new Error('Unable to serialize array field O_dP_EE_d - length must be 6')
    }
    // Serialize message field [O_dP_EE_d]
    bufferOffset = _arraySerializer.float64(obj.O_dP_EE_d, buffer, bufferOffset, 6);
    // Check that the constant length array field [O_T_EE_c] has the right length
    if (obj.O_T_EE_c.length !== 16) {
      throw new Error('Unable to serialize array field O_T_EE_c - length must be 16')
    }
    // Serialize message field [O_T_EE_c]
    bufferOffset = _arraySerializer.float64(obj.O_T_EE_c, buffer, bufferOffset, 16);
    // Check that the constant length array field [O_dP_EE_c] has the right length
    if (obj.O_dP_EE_c.length !== 6) {
      throw new Error('Unable to serialize array field O_dP_EE_c - length must be 6')
    }
    // Serialize message field [O_dP_EE_c]
    bufferOffset = _arraySerializer.float64(obj.O_dP_EE_c, buffer, bufferOffset, 6);
    // Check that the constant length array field [O_ddP_EE_c] has the right length
    if (obj.O_ddP_EE_c.length !== 6) {
      throw new Error('Unable to serialize array field O_ddP_EE_c - length must be 6')
    }
    // Serialize message field [O_ddP_EE_c]
    bufferOffset = _arraySerializer.float64(obj.O_ddP_EE_c, buffer, bufferOffset, 6);
    // Check that the constant length array field [theta] has the right length
    if (obj.theta.length !== 7) {
      throw new Error('Unable to serialize array field theta - length must be 7')
    }
    // Serialize message field [theta]
    bufferOffset = _arraySerializer.float64(obj.theta, buffer, bufferOffset, 7);
    // Check that the constant length array field [dtheta] has the right length
    if (obj.dtheta.length !== 7) {
      throw new Error('Unable to serialize array field dtheta - length must be 7')
    }
    // Serialize message field [dtheta]
    bufferOffset = _arraySerializer.float64(obj.dtheta, buffer, bufferOffset, 7);
    // Serialize message field [current_errors]
    bufferOffset = Errors.serialize(obj.current_errors, buffer, bufferOffset);
    // Serialize message field [last_motion_errors]
    bufferOffset = Errors.serialize(obj.last_motion_errors, buffer, bufferOffset);
    // Serialize message field [control_command_success_rate]
    bufferOffset = _serializer.float64(obj.control_command_success_rate, buffer, bufferOffset);
    // Serialize message field [robot_mode]
    bufferOffset = _serializer.uint8(obj.robot_mode, buffer, bufferOffset);
    // Serialize message field [robot_time]
    bufferOffset = _serializer.float64(obj.robot_time, buffer, bufferOffset);
    // Serialize message field [gripper_width]
    bufferOffset = _serializer.float64(obj.gripper_width, buffer, bufferOffset);
    // Serialize message field [gripper_max_width]
    bufferOffset = _serializer.float64(obj.gripper_max_width, buffer, bufferOffset);
    // Serialize message field [gripper_is_grasped]
    bufferOffset = _serializer.bool(obj.gripper_is_grasped, buffer, bufferOffset);
    // Serialize message field [gripper_temperature]
    bufferOffset = _serializer.uint16(obj.gripper_temperature, buffer, bufferOffset);
    // Serialize message field [gripper_time]
    bufferOffset = _serializer.float64(obj.gripper_time, buffer, bufferOffset);
    // Serialize message field [is_fresh]
    bufferOffset = _serializer.bool(obj.is_fresh, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RobotState
    let len;
    let data = new RobotState(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [pose_desired]
    data.pose_desired = _arrayDeserializer.float64(buffer, bufferOffset, 16)
    // Deserialize message field [O_T_EE]
    data.O_T_EE = _arrayDeserializer.float64(buffer, bufferOffset, 16)
    // Deserialize message field [O_T_EE_d]
    data.O_T_EE_d = _arrayDeserializer.float64(buffer, bufferOffset, 16)
    // Deserialize message field [F_T_EE]
    data.F_T_EE = _arrayDeserializer.float64(buffer, bufferOffset, 16)
    // Deserialize message field [EE_T_K]
    data.EE_T_K = _arrayDeserializer.float64(buffer, bufferOffset, 16)
    // Deserialize message field [m_ee]
    data.m_ee = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [I_ee]
    data.I_ee = _arrayDeserializer.float64(buffer, bufferOffset, 9)
    // Deserialize message field [F_x_Cee]
    data.F_x_Cee = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [m_load]
    data.m_load = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [I_load]
    data.I_load = _arrayDeserializer.float64(buffer, bufferOffset, 9)
    // Deserialize message field [F_x_Cload]
    data.F_x_Cload = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [m_total]
    data.m_total = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [I_total]
    data.I_total = _arrayDeserializer.float64(buffer, bufferOffset, 9)
    // Deserialize message field [F_x_Ctotal]
    data.F_x_Ctotal = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [elbow]
    data.elbow = _arrayDeserializer.float64(buffer, bufferOffset, 2)
    // Deserialize message field [elbow_d]
    data.elbow_d = _arrayDeserializer.float64(buffer, bufferOffset, 2)
    // Deserialize message field [elbow_c]
    data.elbow_c = _arrayDeserializer.float64(buffer, bufferOffset, 2)
    // Deserialize message field [delbow_c]
    data.delbow_c = _arrayDeserializer.float64(buffer, bufferOffset, 2)
    // Deserialize message field [ddelbow_c]
    data.ddelbow_c = _arrayDeserializer.float64(buffer, bufferOffset, 2)
    // Deserialize message field [tau_J]
    data.tau_J = _arrayDeserializer.float64(buffer, bufferOffset, 7)
    // Deserialize message field [tau_J_d]
    data.tau_J_d = _arrayDeserializer.float64(buffer, bufferOffset, 7)
    // Deserialize message field [dtau_J]
    data.dtau_J = _arrayDeserializer.float64(buffer, bufferOffset, 7)
    // Deserialize message field [q]
    data.q = _arrayDeserializer.float64(buffer, bufferOffset, 7)
    // Deserialize message field [q_d]
    data.q_d = _arrayDeserializer.float64(buffer, bufferOffset, 7)
    // Deserialize message field [dq]
    data.dq = _arrayDeserializer.float64(buffer, bufferOffset, 7)
    // Deserialize message field [dq_d]
    data.dq_d = _arrayDeserializer.float64(buffer, bufferOffset, 7)
    // Deserialize message field [ddq_d]
    data.ddq_d = _arrayDeserializer.float64(buffer, bufferOffset, 7)
    // Deserialize message field [joint_contact]
    data.joint_contact = _arrayDeserializer.float64(buffer, bufferOffset, 7)
    // Deserialize message field [cartesian_contact]
    data.cartesian_contact = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [joint_collision]
    data.joint_collision = _arrayDeserializer.float64(buffer, bufferOffset, 7)
    // Deserialize message field [cartesian_collision]
    data.cartesian_collision = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [tau_ext_hat_filtered]
    data.tau_ext_hat_filtered = _arrayDeserializer.float64(buffer, bufferOffset, 7)
    // Deserialize message field [O_F_ext_hat_K]
    data.O_F_ext_hat_K = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [K_F_ext_hat_K]
    data.K_F_ext_hat_K = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [O_dP_EE_d]
    data.O_dP_EE_d = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [O_T_EE_c]
    data.O_T_EE_c = _arrayDeserializer.float64(buffer, bufferOffset, 16)
    // Deserialize message field [O_dP_EE_c]
    data.O_dP_EE_c = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [O_ddP_EE_c]
    data.O_ddP_EE_c = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [theta]
    data.theta = _arrayDeserializer.float64(buffer, bufferOffset, 7)
    // Deserialize message field [dtheta]
    data.dtheta = _arrayDeserializer.float64(buffer, bufferOffset, 7)
    // Deserialize message field [current_errors]
    data.current_errors = Errors.deserialize(buffer, bufferOffset);
    // Deserialize message field [last_motion_errors]
    data.last_motion_errors = Errors.deserialize(buffer, bufferOffset);
    // Deserialize message field [control_command_success_rate]
    data.control_command_success_rate = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [robot_mode]
    data.robot_mode = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [robot_time]
    data.robot_time = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [gripper_width]
    data.gripper_width = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [gripper_max_width]
    data.gripper_max_width = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [gripper_is_grasped]
    data.gripper_is_grasped = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [gripper_temperature]
    data.gripper_temperature = _deserializer.uint16(buffer, bufferOffset);
    // Deserialize message field [gripper_time]
    data.gripper_time = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [is_fresh]
    data.is_fresh = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 2343;
  }

  static datatype() {
    // Returns string type for a message object
    return 'franka_interface_msgs/RobotState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1bd13b23161f53b273eafd78a98128f7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new RobotState(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.pose_desired !== undefined) {
      resolved.pose_desired = msg.pose_desired;
    }
    else {
      resolved.pose_desired = new Array(16).fill(0)
    }

    if (msg.O_T_EE !== undefined) {
      resolved.O_T_EE = msg.O_T_EE;
    }
    else {
      resolved.O_T_EE = new Array(16).fill(0)
    }

    if (msg.O_T_EE_d !== undefined) {
      resolved.O_T_EE_d = msg.O_T_EE_d;
    }
    else {
      resolved.O_T_EE_d = new Array(16).fill(0)
    }

    if (msg.F_T_EE !== undefined) {
      resolved.F_T_EE = msg.F_T_EE;
    }
    else {
      resolved.F_T_EE = new Array(16).fill(0)
    }

    if (msg.EE_T_K !== undefined) {
      resolved.EE_T_K = msg.EE_T_K;
    }
    else {
      resolved.EE_T_K = new Array(16).fill(0)
    }

    if (msg.m_ee !== undefined) {
      resolved.m_ee = msg.m_ee;
    }
    else {
      resolved.m_ee = 0.0
    }

    if (msg.I_ee !== undefined) {
      resolved.I_ee = msg.I_ee;
    }
    else {
      resolved.I_ee = new Array(9).fill(0)
    }

    if (msg.F_x_Cee !== undefined) {
      resolved.F_x_Cee = msg.F_x_Cee;
    }
    else {
      resolved.F_x_Cee = new Array(3).fill(0)
    }

    if (msg.m_load !== undefined) {
      resolved.m_load = msg.m_load;
    }
    else {
      resolved.m_load = 0.0
    }

    if (msg.I_load !== undefined) {
      resolved.I_load = msg.I_load;
    }
    else {
      resolved.I_load = new Array(9).fill(0)
    }

    if (msg.F_x_Cload !== undefined) {
      resolved.F_x_Cload = msg.F_x_Cload;
    }
    else {
      resolved.F_x_Cload = new Array(3).fill(0)
    }

    if (msg.m_total !== undefined) {
      resolved.m_total = msg.m_total;
    }
    else {
      resolved.m_total = 0.0
    }

    if (msg.I_total !== undefined) {
      resolved.I_total = msg.I_total;
    }
    else {
      resolved.I_total = new Array(9).fill(0)
    }

    if (msg.F_x_Ctotal !== undefined) {
      resolved.F_x_Ctotal = msg.F_x_Ctotal;
    }
    else {
      resolved.F_x_Ctotal = new Array(3).fill(0)
    }

    if (msg.elbow !== undefined) {
      resolved.elbow = msg.elbow;
    }
    else {
      resolved.elbow = new Array(2).fill(0)
    }

    if (msg.elbow_d !== undefined) {
      resolved.elbow_d = msg.elbow_d;
    }
    else {
      resolved.elbow_d = new Array(2).fill(0)
    }

    if (msg.elbow_c !== undefined) {
      resolved.elbow_c = msg.elbow_c;
    }
    else {
      resolved.elbow_c = new Array(2).fill(0)
    }

    if (msg.delbow_c !== undefined) {
      resolved.delbow_c = msg.delbow_c;
    }
    else {
      resolved.delbow_c = new Array(2).fill(0)
    }

    if (msg.ddelbow_c !== undefined) {
      resolved.ddelbow_c = msg.ddelbow_c;
    }
    else {
      resolved.ddelbow_c = new Array(2).fill(0)
    }

    if (msg.tau_J !== undefined) {
      resolved.tau_J = msg.tau_J;
    }
    else {
      resolved.tau_J = new Array(7).fill(0)
    }

    if (msg.tau_J_d !== undefined) {
      resolved.tau_J_d = msg.tau_J_d;
    }
    else {
      resolved.tau_J_d = new Array(7).fill(0)
    }

    if (msg.dtau_J !== undefined) {
      resolved.dtau_J = msg.dtau_J;
    }
    else {
      resolved.dtau_J = new Array(7).fill(0)
    }

    if (msg.q !== undefined) {
      resolved.q = msg.q;
    }
    else {
      resolved.q = new Array(7).fill(0)
    }

    if (msg.q_d !== undefined) {
      resolved.q_d = msg.q_d;
    }
    else {
      resolved.q_d = new Array(7).fill(0)
    }

    if (msg.dq !== undefined) {
      resolved.dq = msg.dq;
    }
    else {
      resolved.dq = new Array(7).fill(0)
    }

    if (msg.dq_d !== undefined) {
      resolved.dq_d = msg.dq_d;
    }
    else {
      resolved.dq_d = new Array(7).fill(0)
    }

    if (msg.ddq_d !== undefined) {
      resolved.ddq_d = msg.ddq_d;
    }
    else {
      resolved.ddq_d = new Array(7).fill(0)
    }

    if (msg.joint_contact !== undefined) {
      resolved.joint_contact = msg.joint_contact;
    }
    else {
      resolved.joint_contact = new Array(7).fill(0)
    }

    if (msg.cartesian_contact !== undefined) {
      resolved.cartesian_contact = msg.cartesian_contact;
    }
    else {
      resolved.cartesian_contact = new Array(6).fill(0)
    }

    if (msg.joint_collision !== undefined) {
      resolved.joint_collision = msg.joint_collision;
    }
    else {
      resolved.joint_collision = new Array(7).fill(0)
    }

    if (msg.cartesian_collision !== undefined) {
      resolved.cartesian_collision = msg.cartesian_collision;
    }
    else {
      resolved.cartesian_collision = new Array(6).fill(0)
    }

    if (msg.tau_ext_hat_filtered !== undefined) {
      resolved.tau_ext_hat_filtered = msg.tau_ext_hat_filtered;
    }
    else {
      resolved.tau_ext_hat_filtered = new Array(7).fill(0)
    }

    if (msg.O_F_ext_hat_K !== undefined) {
      resolved.O_F_ext_hat_K = msg.O_F_ext_hat_K;
    }
    else {
      resolved.O_F_ext_hat_K = new Array(6).fill(0)
    }

    if (msg.K_F_ext_hat_K !== undefined) {
      resolved.K_F_ext_hat_K = msg.K_F_ext_hat_K;
    }
    else {
      resolved.K_F_ext_hat_K = new Array(6).fill(0)
    }

    if (msg.O_dP_EE_d !== undefined) {
      resolved.O_dP_EE_d = msg.O_dP_EE_d;
    }
    else {
      resolved.O_dP_EE_d = new Array(6).fill(0)
    }

    if (msg.O_T_EE_c !== undefined) {
      resolved.O_T_EE_c = msg.O_T_EE_c;
    }
    else {
      resolved.O_T_EE_c = new Array(16).fill(0)
    }

    if (msg.O_dP_EE_c !== undefined) {
      resolved.O_dP_EE_c = msg.O_dP_EE_c;
    }
    else {
      resolved.O_dP_EE_c = new Array(6).fill(0)
    }

    if (msg.O_ddP_EE_c !== undefined) {
      resolved.O_ddP_EE_c = msg.O_ddP_EE_c;
    }
    else {
      resolved.O_ddP_EE_c = new Array(6).fill(0)
    }

    if (msg.theta !== undefined) {
      resolved.theta = msg.theta;
    }
    else {
      resolved.theta = new Array(7).fill(0)
    }

    if (msg.dtheta !== undefined) {
      resolved.dtheta = msg.dtheta;
    }
    else {
      resolved.dtheta = new Array(7).fill(0)
    }

    if (msg.current_errors !== undefined) {
      resolved.current_errors = Errors.Resolve(msg.current_errors)
    }
    else {
      resolved.current_errors = new Errors()
    }

    if (msg.last_motion_errors !== undefined) {
      resolved.last_motion_errors = Errors.Resolve(msg.last_motion_errors)
    }
    else {
      resolved.last_motion_errors = new Errors()
    }

    if (msg.control_command_success_rate !== undefined) {
      resolved.control_command_success_rate = msg.control_command_success_rate;
    }
    else {
      resolved.control_command_success_rate = 0.0
    }

    if (msg.robot_mode !== undefined) {
      resolved.robot_mode = msg.robot_mode;
    }
    else {
      resolved.robot_mode = 0
    }

    if (msg.robot_time !== undefined) {
      resolved.robot_time = msg.robot_time;
    }
    else {
      resolved.robot_time = 0.0
    }

    if (msg.gripper_width !== undefined) {
      resolved.gripper_width = msg.gripper_width;
    }
    else {
      resolved.gripper_width = 0.0
    }

    if (msg.gripper_max_width !== undefined) {
      resolved.gripper_max_width = msg.gripper_max_width;
    }
    else {
      resolved.gripper_max_width = 0.0
    }

    if (msg.gripper_is_grasped !== undefined) {
      resolved.gripper_is_grasped = msg.gripper_is_grasped;
    }
    else {
      resolved.gripper_is_grasped = false
    }

    if (msg.gripper_temperature !== undefined) {
      resolved.gripper_temperature = msg.gripper_temperature;
    }
    else {
      resolved.gripper_temperature = 0
    }

    if (msg.gripper_time !== undefined) {
      resolved.gripper_time = msg.gripper_time;
    }
    else {
      resolved.gripper_time = 0.0
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

// Constants for message
RobotState.Constants = {
  ROBOT_MODE_OTHER: 0,
  ROBOT_MODE_IDLE: 1,
  ROBOT_MODE_MOVE: 2,
  ROBOT_MODE_GUIDING: 3,
  ROBOT_MODE_REFLEX: 4,
  ROBOT_MODE_USER_STOPPED: 5,
  ROBOT_MODE_AUTOMATIC_ERROR_RECOVERY: 6,
}

module.exports = RobotState;

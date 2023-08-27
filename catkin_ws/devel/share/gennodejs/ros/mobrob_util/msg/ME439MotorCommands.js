// Auto-generated. Do not edit!

// (in-package mobrob_util.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class ME439MotorCommands {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.cmd0 = null;
      this.cmd1 = null;
    }
    else {
      if (initObj.hasOwnProperty('cmd0')) {
        this.cmd0 = initObj.cmd0
      }
      else {
        this.cmd0 = 0;
      }
      if (initObj.hasOwnProperty('cmd1')) {
        this.cmd1 = initObj.cmd1
      }
      else {
        this.cmd1 = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ME439MotorCommands
    // Serialize message field [cmd0]
    bufferOffset = _serializer.int16(obj.cmd0, buffer, bufferOffset);
    // Serialize message field [cmd1]
    bufferOffset = _serializer.int16(obj.cmd1, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ME439MotorCommands
    let len;
    let data = new ME439MotorCommands(null);
    // Deserialize message field [cmd0]
    data.cmd0 = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [cmd1]
    data.cmd1 = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mobrob_util/ME439MotorCommands';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '853bb91b9c3f09306e8c71787dc35bdf';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int16 cmd0
    int16 cmd1
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ME439MotorCommands(null);
    if (msg.cmd0 !== undefined) {
      resolved.cmd0 = msg.cmd0;
    }
    else {
      resolved.cmd0 = 0
    }

    if (msg.cmd1 !== undefined) {
      resolved.cmd1 = msg.cmd1;
    }
    else {
      resolved.cmd1 = 0
    }

    return resolved;
    }
};

module.exports = ME439MotorCommands;

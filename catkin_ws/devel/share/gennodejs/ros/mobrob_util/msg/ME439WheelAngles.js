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

class ME439WheelAngles {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ang_left = null;
      this.ang_right = null;
    }
    else {
      if (initObj.hasOwnProperty('ang_left')) {
        this.ang_left = initObj.ang_left
      }
      else {
        this.ang_left = 0.0;
      }
      if (initObj.hasOwnProperty('ang_right')) {
        this.ang_right = initObj.ang_right
      }
      else {
        this.ang_right = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ME439WheelAngles
    // Serialize message field [ang_left]
    bufferOffset = _serializer.float64(obj.ang_left, buffer, bufferOffset);
    // Serialize message field [ang_right]
    bufferOffset = _serializer.float64(obj.ang_right, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ME439WheelAngles
    let len;
    let data = new ME439WheelAngles(null);
    // Deserialize message field [ang_left]
    data.ang_left = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [ang_right]
    data.ang_right = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mobrob_util/ME439WheelAngles';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8e9b84200a184649eafa090ea3ef1869';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 ang_left
    float64 ang_right
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ME439WheelAngles(null);
    if (msg.ang_left !== undefined) {
      resolved.ang_left = msg.ang_left;
    }
    else {
      resolved.ang_left = 0.0
    }

    if (msg.ang_right !== undefined) {
      resolved.ang_right = msg.ang_right;
    }
    else {
      resolved.ang_right = 0.0
    }

    return resolved;
    }
};

module.exports = ME439WheelAngles;

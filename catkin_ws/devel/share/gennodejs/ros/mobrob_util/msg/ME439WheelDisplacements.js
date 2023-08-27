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

class ME439WheelDisplacements {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.d_left = null;
      this.d_right = null;
    }
    else {
      if (initObj.hasOwnProperty('d_left')) {
        this.d_left = initObj.d_left
      }
      else {
        this.d_left = 0.0;
      }
      if (initObj.hasOwnProperty('d_right')) {
        this.d_right = initObj.d_right
      }
      else {
        this.d_right = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ME439WheelDisplacements
    // Serialize message field [d_left]
    bufferOffset = _serializer.float64(obj.d_left, buffer, bufferOffset);
    // Serialize message field [d_right]
    bufferOffset = _serializer.float64(obj.d_right, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ME439WheelDisplacements
    let len;
    let data = new ME439WheelDisplacements(null);
    // Deserialize message field [d_left]
    data.d_left = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [d_right]
    data.d_right = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mobrob_util/ME439WheelDisplacements';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0c76547e5c694fd23ef9ed1874ecca4a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 d_left
    float64 d_right
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ME439WheelDisplacements(null);
    if (msg.d_left !== undefined) {
      resolved.d_left = msg.d_left;
    }
    else {
      resolved.d_left = 0.0
    }

    if (msg.d_right !== undefined) {
      resolved.d_right = msg.d_right;
    }
    else {
      resolved.d_right = 0.0
    }

    return resolved;
    }
};

module.exports = ME439WheelDisplacements;

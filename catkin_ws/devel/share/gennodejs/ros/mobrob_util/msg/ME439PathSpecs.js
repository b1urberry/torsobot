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

class ME439PathSpecs {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.x0 = null;
      this.y0 = null;
      this.theta0 = null;
      this.Radius = null;
      this.Length = null;
    }
    else {
      if (initObj.hasOwnProperty('x0')) {
        this.x0 = initObj.x0
      }
      else {
        this.x0 = 0.0;
      }
      if (initObj.hasOwnProperty('y0')) {
        this.y0 = initObj.y0
      }
      else {
        this.y0 = 0.0;
      }
      if (initObj.hasOwnProperty('theta0')) {
        this.theta0 = initObj.theta0
      }
      else {
        this.theta0 = 0.0;
      }
      if (initObj.hasOwnProperty('Radius')) {
        this.Radius = initObj.Radius
      }
      else {
        this.Radius = 0.0;
      }
      if (initObj.hasOwnProperty('Length')) {
        this.Length = initObj.Length
      }
      else {
        this.Length = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ME439PathSpecs
    // Serialize message field [x0]
    bufferOffset = _serializer.float32(obj.x0, buffer, bufferOffset);
    // Serialize message field [y0]
    bufferOffset = _serializer.float32(obj.y0, buffer, bufferOffset);
    // Serialize message field [theta0]
    bufferOffset = _serializer.float32(obj.theta0, buffer, bufferOffset);
    // Serialize message field [Radius]
    bufferOffset = _serializer.float32(obj.Radius, buffer, bufferOffset);
    // Serialize message field [Length]
    bufferOffset = _serializer.float32(obj.Length, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ME439PathSpecs
    let len;
    let data = new ME439PathSpecs(null);
    // Deserialize message field [x0]
    data.x0 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [y0]
    data.y0 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [theta0]
    data.theta0 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [Radius]
    data.Radius = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [Length]
    data.Length = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mobrob_util/ME439PathSpecs';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '278b13eb1b2cd154f8602d6118f2c62d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 x0
    float32 y0
    float32 theta0
    float32 Radius
    float32 Length
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ME439PathSpecs(null);
    if (msg.x0 !== undefined) {
      resolved.x0 = msg.x0;
    }
    else {
      resolved.x0 = 0.0
    }

    if (msg.y0 !== undefined) {
      resolved.y0 = msg.y0;
    }
    else {
      resolved.y0 = 0.0
    }

    if (msg.theta0 !== undefined) {
      resolved.theta0 = msg.theta0;
    }
    else {
      resolved.theta0 = 0.0
    }

    if (msg.Radius !== undefined) {
      resolved.Radius = msg.Radius;
    }
    else {
      resolved.Radius = 0.0
    }

    if (msg.Length !== undefined) {
      resolved.Length = msg.Length;
    }
    else {
      resolved.Length = 0.0
    }

    return resolved;
    }
};

module.exports = ME439PathSpecs;

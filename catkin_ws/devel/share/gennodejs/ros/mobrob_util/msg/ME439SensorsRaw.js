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

class ME439SensorsRaw {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.e0 = null;
      this.e1 = null;
      this.a0 = null;
      this.a1 = null;
      this.a2 = null;
      this.a3 = null;
      this.a4 = null;
      this.a5 = null;
      this.u0 = null;
      this.u1 = null;
      this.u2 = null;
      this.t = null;
    }
    else {
      if (initObj.hasOwnProperty('e0')) {
        this.e0 = initObj.e0
      }
      else {
        this.e0 = 0;
      }
      if (initObj.hasOwnProperty('e1')) {
        this.e1 = initObj.e1
      }
      else {
        this.e1 = 0;
      }
      if (initObj.hasOwnProperty('a0')) {
        this.a0 = initObj.a0
      }
      else {
        this.a0 = 0;
      }
      if (initObj.hasOwnProperty('a1')) {
        this.a1 = initObj.a1
      }
      else {
        this.a1 = 0;
      }
      if (initObj.hasOwnProperty('a2')) {
        this.a2 = initObj.a2
      }
      else {
        this.a2 = 0;
      }
      if (initObj.hasOwnProperty('a3')) {
        this.a3 = initObj.a3
      }
      else {
        this.a3 = 0;
      }
      if (initObj.hasOwnProperty('a4')) {
        this.a4 = initObj.a4
      }
      else {
        this.a4 = 0;
      }
      if (initObj.hasOwnProperty('a5')) {
        this.a5 = initObj.a5
      }
      else {
        this.a5 = 0;
      }
      if (initObj.hasOwnProperty('u0')) {
        this.u0 = initObj.u0
      }
      else {
        this.u0 = 0;
      }
      if (initObj.hasOwnProperty('u1')) {
        this.u1 = initObj.u1
      }
      else {
        this.u1 = 0;
      }
      if (initObj.hasOwnProperty('u2')) {
        this.u2 = initObj.u2
      }
      else {
        this.u2 = 0;
      }
      if (initObj.hasOwnProperty('t')) {
        this.t = initObj.t
      }
      else {
        this.t = {secs: 0, nsecs: 0};
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ME439SensorsRaw
    // Serialize message field [e0]
    bufferOffset = _serializer.int64(obj.e0, buffer, bufferOffset);
    // Serialize message field [e1]
    bufferOffset = _serializer.int64(obj.e1, buffer, bufferOffset);
    // Serialize message field [a0]
    bufferOffset = _serializer.int16(obj.a0, buffer, bufferOffset);
    // Serialize message field [a1]
    bufferOffset = _serializer.int16(obj.a1, buffer, bufferOffset);
    // Serialize message field [a2]
    bufferOffset = _serializer.int16(obj.a2, buffer, bufferOffset);
    // Serialize message field [a3]
    bufferOffset = _serializer.int16(obj.a3, buffer, bufferOffset);
    // Serialize message field [a4]
    bufferOffset = _serializer.int16(obj.a4, buffer, bufferOffset);
    // Serialize message field [a5]
    bufferOffset = _serializer.int16(obj.a5, buffer, bufferOffset);
    // Serialize message field [u0]
    bufferOffset = _serializer.int32(obj.u0, buffer, bufferOffset);
    // Serialize message field [u1]
    bufferOffset = _serializer.int32(obj.u1, buffer, bufferOffset);
    // Serialize message field [u2]
    bufferOffset = _serializer.int32(obj.u2, buffer, bufferOffset);
    // Serialize message field [t]
    bufferOffset = _serializer.time(obj.t, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ME439SensorsRaw
    let len;
    let data = new ME439SensorsRaw(null);
    // Deserialize message field [e0]
    data.e0 = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [e1]
    data.e1 = _deserializer.int64(buffer, bufferOffset);
    // Deserialize message field [a0]
    data.a0 = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [a1]
    data.a1 = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [a2]
    data.a2 = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [a3]
    data.a3 = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [a4]
    data.a4 = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [a5]
    data.a5 = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [u0]
    data.u0 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [u1]
    data.u1 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [u2]
    data.u2 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [t]
    data.t = _deserializer.time(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 48;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mobrob_util/ME439SensorsRaw';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e832476c98d8752e8bb419b8adb5fe2a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int64 e0
    int64 e1
    int16 a0
    int16 a1
    int16 a2
    int16 a3
    int16 a4
    int16 a5
    int32 u0
    int32 u1
    int32 u2
    time t
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ME439SensorsRaw(null);
    if (msg.e0 !== undefined) {
      resolved.e0 = msg.e0;
    }
    else {
      resolved.e0 = 0
    }

    if (msg.e1 !== undefined) {
      resolved.e1 = msg.e1;
    }
    else {
      resolved.e1 = 0
    }

    if (msg.a0 !== undefined) {
      resolved.a0 = msg.a0;
    }
    else {
      resolved.a0 = 0
    }

    if (msg.a1 !== undefined) {
      resolved.a1 = msg.a1;
    }
    else {
      resolved.a1 = 0
    }

    if (msg.a2 !== undefined) {
      resolved.a2 = msg.a2;
    }
    else {
      resolved.a2 = 0
    }

    if (msg.a3 !== undefined) {
      resolved.a3 = msg.a3;
    }
    else {
      resolved.a3 = 0
    }

    if (msg.a4 !== undefined) {
      resolved.a4 = msg.a4;
    }
    else {
      resolved.a4 = 0
    }

    if (msg.a5 !== undefined) {
      resolved.a5 = msg.a5;
    }
    else {
      resolved.a5 = 0
    }

    if (msg.u0 !== undefined) {
      resolved.u0 = msg.u0;
    }
    else {
      resolved.u0 = 0
    }

    if (msg.u1 !== undefined) {
      resolved.u1 = msg.u1;
    }
    else {
      resolved.u1 = 0
    }

    if (msg.u2 !== undefined) {
      resolved.u2 = msg.u2;
    }
    else {
      resolved.u2 = 0
    }

    if (msg.t !== undefined) {
      resolved.t = msg.t;
    }
    else {
      resolved.t = {secs: 0, nsecs: 0}
    }

    return resolved;
    }
};

module.exports = ME439SensorsRaw;

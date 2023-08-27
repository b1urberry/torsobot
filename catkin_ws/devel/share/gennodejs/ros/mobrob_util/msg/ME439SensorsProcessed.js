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

class ME439SensorsProcessed {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.e0radians = null;
      this.e1radians = null;
      this.e0meters = null;
      this.e1meters = null;
      this.e0radpersec = null;
      this.e1radpersec = null;
      this.e0meterspersec = null;
      this.e1meterspersec = null;
      this.a0 = null;
      this.a1 = null;
      this.a2 = null;
      this.a3 = null;
      this.a4 = null;
      this.a5 = null;
      this.u0meters = null;
      this.u1meters = null;
      this.u2meters = null;
      this.dt = null;
    }
    else {
      if (initObj.hasOwnProperty('e0radians')) {
        this.e0radians = initObj.e0radians
      }
      else {
        this.e0radians = 0.0;
      }
      if (initObj.hasOwnProperty('e1radians')) {
        this.e1radians = initObj.e1radians
      }
      else {
        this.e1radians = 0.0;
      }
      if (initObj.hasOwnProperty('e0meters')) {
        this.e0meters = initObj.e0meters
      }
      else {
        this.e0meters = 0.0;
      }
      if (initObj.hasOwnProperty('e1meters')) {
        this.e1meters = initObj.e1meters
      }
      else {
        this.e1meters = 0.0;
      }
      if (initObj.hasOwnProperty('e0radpersec')) {
        this.e0radpersec = initObj.e0radpersec
      }
      else {
        this.e0radpersec = 0.0;
      }
      if (initObj.hasOwnProperty('e1radpersec')) {
        this.e1radpersec = initObj.e1radpersec
      }
      else {
        this.e1radpersec = 0.0;
      }
      if (initObj.hasOwnProperty('e0meterspersec')) {
        this.e0meterspersec = initObj.e0meterspersec
      }
      else {
        this.e0meterspersec = 0.0;
      }
      if (initObj.hasOwnProperty('e1meterspersec')) {
        this.e1meterspersec = initObj.e1meterspersec
      }
      else {
        this.e1meterspersec = 0.0;
      }
      if (initObj.hasOwnProperty('a0')) {
        this.a0 = initObj.a0
      }
      else {
        this.a0 = 0.0;
      }
      if (initObj.hasOwnProperty('a1')) {
        this.a1 = initObj.a1
      }
      else {
        this.a1 = 0.0;
      }
      if (initObj.hasOwnProperty('a2')) {
        this.a2 = initObj.a2
      }
      else {
        this.a2 = 0.0;
      }
      if (initObj.hasOwnProperty('a3')) {
        this.a3 = initObj.a3
      }
      else {
        this.a3 = 0.0;
      }
      if (initObj.hasOwnProperty('a4')) {
        this.a4 = initObj.a4
      }
      else {
        this.a4 = 0.0;
      }
      if (initObj.hasOwnProperty('a5')) {
        this.a5 = initObj.a5
      }
      else {
        this.a5 = 0.0;
      }
      if (initObj.hasOwnProperty('u0meters')) {
        this.u0meters = initObj.u0meters
      }
      else {
        this.u0meters = 0.0;
      }
      if (initObj.hasOwnProperty('u1meters')) {
        this.u1meters = initObj.u1meters
      }
      else {
        this.u1meters = 0.0;
      }
      if (initObj.hasOwnProperty('u2meters')) {
        this.u2meters = initObj.u2meters
      }
      else {
        this.u2meters = 0.0;
      }
      if (initObj.hasOwnProperty('dt')) {
        this.dt = initObj.dt
      }
      else {
        this.dt = {secs: 0, nsecs: 0};
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ME439SensorsProcessed
    // Serialize message field [e0radians]
    bufferOffset = _serializer.float32(obj.e0radians, buffer, bufferOffset);
    // Serialize message field [e1radians]
    bufferOffset = _serializer.float32(obj.e1radians, buffer, bufferOffset);
    // Serialize message field [e0meters]
    bufferOffset = _serializer.float32(obj.e0meters, buffer, bufferOffset);
    // Serialize message field [e1meters]
    bufferOffset = _serializer.float32(obj.e1meters, buffer, bufferOffset);
    // Serialize message field [e0radpersec]
    bufferOffset = _serializer.float32(obj.e0radpersec, buffer, bufferOffset);
    // Serialize message field [e1radpersec]
    bufferOffset = _serializer.float32(obj.e1radpersec, buffer, bufferOffset);
    // Serialize message field [e0meterspersec]
    bufferOffset = _serializer.float32(obj.e0meterspersec, buffer, bufferOffset);
    // Serialize message field [e1meterspersec]
    bufferOffset = _serializer.float32(obj.e1meterspersec, buffer, bufferOffset);
    // Serialize message field [a0]
    bufferOffset = _serializer.float32(obj.a0, buffer, bufferOffset);
    // Serialize message field [a1]
    bufferOffset = _serializer.float32(obj.a1, buffer, bufferOffset);
    // Serialize message field [a2]
    bufferOffset = _serializer.float32(obj.a2, buffer, bufferOffset);
    // Serialize message field [a3]
    bufferOffset = _serializer.float32(obj.a3, buffer, bufferOffset);
    // Serialize message field [a4]
    bufferOffset = _serializer.float32(obj.a4, buffer, bufferOffset);
    // Serialize message field [a5]
    bufferOffset = _serializer.float32(obj.a5, buffer, bufferOffset);
    // Serialize message field [u0meters]
    bufferOffset = _serializer.float32(obj.u0meters, buffer, bufferOffset);
    // Serialize message field [u1meters]
    bufferOffset = _serializer.float32(obj.u1meters, buffer, bufferOffset);
    // Serialize message field [u2meters]
    bufferOffset = _serializer.float32(obj.u2meters, buffer, bufferOffset);
    // Serialize message field [dt]
    bufferOffset = _serializer.duration(obj.dt, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ME439SensorsProcessed
    let len;
    let data = new ME439SensorsProcessed(null);
    // Deserialize message field [e0radians]
    data.e0radians = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [e1radians]
    data.e1radians = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [e0meters]
    data.e0meters = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [e1meters]
    data.e1meters = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [e0radpersec]
    data.e0radpersec = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [e1radpersec]
    data.e1radpersec = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [e0meterspersec]
    data.e0meterspersec = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [e1meterspersec]
    data.e1meterspersec = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [a0]
    data.a0 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [a1]
    data.a1 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [a2]
    data.a2 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [a3]
    data.a3 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [a4]
    data.a4 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [a5]
    data.a5 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [u0meters]
    data.u0meters = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [u1meters]
    data.u1meters = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [u2meters]
    data.u2meters = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [dt]
    data.dt = _deserializer.duration(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 76;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mobrob_util/ME439SensorsProcessed';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '44afd1f89023a5ad6799a84d47e3ad41';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 e0radians
    float32 e1radians
    float32 e0meters
    float32 e1meters
    float32 e0radpersec
    float32 e1radpersec
    float32 e0meterspersec
    float32 e1meterspersec
    float32 a0
    float32 a1
    float32 a2
    float32 a3
    float32 a4
    float32 a5
    float32 u0meters
    float32 u1meters
    float32 u2meters
    duration dt
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ME439SensorsProcessed(null);
    if (msg.e0radians !== undefined) {
      resolved.e0radians = msg.e0radians;
    }
    else {
      resolved.e0radians = 0.0
    }

    if (msg.e1radians !== undefined) {
      resolved.e1radians = msg.e1radians;
    }
    else {
      resolved.e1radians = 0.0
    }

    if (msg.e0meters !== undefined) {
      resolved.e0meters = msg.e0meters;
    }
    else {
      resolved.e0meters = 0.0
    }

    if (msg.e1meters !== undefined) {
      resolved.e1meters = msg.e1meters;
    }
    else {
      resolved.e1meters = 0.0
    }

    if (msg.e0radpersec !== undefined) {
      resolved.e0radpersec = msg.e0radpersec;
    }
    else {
      resolved.e0radpersec = 0.0
    }

    if (msg.e1radpersec !== undefined) {
      resolved.e1radpersec = msg.e1radpersec;
    }
    else {
      resolved.e1radpersec = 0.0
    }

    if (msg.e0meterspersec !== undefined) {
      resolved.e0meterspersec = msg.e0meterspersec;
    }
    else {
      resolved.e0meterspersec = 0.0
    }

    if (msg.e1meterspersec !== undefined) {
      resolved.e1meterspersec = msg.e1meterspersec;
    }
    else {
      resolved.e1meterspersec = 0.0
    }

    if (msg.a0 !== undefined) {
      resolved.a0 = msg.a0;
    }
    else {
      resolved.a0 = 0.0
    }

    if (msg.a1 !== undefined) {
      resolved.a1 = msg.a1;
    }
    else {
      resolved.a1 = 0.0
    }

    if (msg.a2 !== undefined) {
      resolved.a2 = msg.a2;
    }
    else {
      resolved.a2 = 0.0
    }

    if (msg.a3 !== undefined) {
      resolved.a3 = msg.a3;
    }
    else {
      resolved.a3 = 0.0
    }

    if (msg.a4 !== undefined) {
      resolved.a4 = msg.a4;
    }
    else {
      resolved.a4 = 0.0
    }

    if (msg.a5 !== undefined) {
      resolved.a5 = msg.a5;
    }
    else {
      resolved.a5 = 0.0
    }

    if (msg.u0meters !== undefined) {
      resolved.u0meters = msg.u0meters;
    }
    else {
      resolved.u0meters = 0.0
    }

    if (msg.u1meters !== undefined) {
      resolved.u1meters = msg.u1meters;
    }
    else {
      resolved.u1meters = 0.0
    }

    if (msg.u2meters !== undefined) {
      resolved.u2meters = msg.u2meters;
    }
    else {
      resolved.u2meters = 0.0
    }

    if (msg.dt !== undefined) {
      resolved.dt = msg.dt;
    }
    else {
      resolved.dt = {secs: 0, nsecs: 0}
    }

    return resolved;
    }
};

module.exports = ME439SensorsProcessed;

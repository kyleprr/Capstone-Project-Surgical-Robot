// Auto-generated. Do not edit!

// (in-package surgical_robot.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Tracker {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.x = null;
      this.y = null;
      this.z = null;
      this.error_x = null;
      this.error_y = null;
      this.error_z = null;
      this.flag1 = null;
      this.flag2 = null;
      this.flag3 = null;
    }
    else {
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = 0.0;
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = 0.0;
      }
      if (initObj.hasOwnProperty('z')) {
        this.z = initObj.z
      }
      else {
        this.z = 0.0;
      }
      if (initObj.hasOwnProperty('error_x')) {
        this.error_x = initObj.error_x
      }
      else {
        this.error_x = 0.0;
      }
      if (initObj.hasOwnProperty('error_y')) {
        this.error_y = initObj.error_y
      }
      else {
        this.error_y = 0.0;
      }
      if (initObj.hasOwnProperty('error_z')) {
        this.error_z = initObj.error_z
      }
      else {
        this.error_z = 0.0;
      }
      if (initObj.hasOwnProperty('flag1')) {
        this.flag1 = initObj.flag1
      }
      else {
        this.flag1 = false;
      }
      if (initObj.hasOwnProperty('flag2')) {
        this.flag2 = initObj.flag2
      }
      else {
        this.flag2 = false;
      }
      if (initObj.hasOwnProperty('flag3')) {
        this.flag3 = initObj.flag3
      }
      else {
        this.flag3 = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Tracker
    // Serialize message field [x]
    bufferOffset = _serializer.float64(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.float64(obj.y, buffer, bufferOffset);
    // Serialize message field [z]
    bufferOffset = _serializer.float64(obj.z, buffer, bufferOffset);
    // Serialize message field [error_x]
    bufferOffset = _serializer.float64(obj.error_x, buffer, bufferOffset);
    // Serialize message field [error_y]
    bufferOffset = _serializer.float64(obj.error_y, buffer, bufferOffset);
    // Serialize message field [error_z]
    bufferOffset = _serializer.float64(obj.error_z, buffer, bufferOffset);
    // Serialize message field [flag1]
    bufferOffset = _serializer.bool(obj.flag1, buffer, bufferOffset);
    // Serialize message field [flag2]
    bufferOffset = _serializer.bool(obj.flag2, buffer, bufferOffset);
    // Serialize message field [flag3]
    bufferOffset = _serializer.bool(obj.flag3, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Tracker
    let len;
    let data = new Tracker(null);
    // Deserialize message field [x]
    data.x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [z]
    data.z = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [error_x]
    data.error_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [error_y]
    data.error_y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [error_z]
    data.error_z = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [flag1]
    data.flag1 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [flag2]
    data.flag2 = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [flag3]
    data.flag3 = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 51;
  }

  static datatype() {
    // Returns string type for a message object
    return 'surgical_robot/Tracker';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b283d7d9a91916dac4010d28ee78ee60';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # message type to describe the tracking information of the blocks
    # to be published as a topic
    
    float64 x  # x coordinate in the world
    float64 y  # y coordinate in the world
    float64 z  # z coordinate in the world
    float64 error_x
    float64 error_y
    float64 error_z
    bool flag1
    bool flag2
    bool flag3
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Tracker(null);
    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = 0.0
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = 0.0
    }

    if (msg.z !== undefined) {
      resolved.z = msg.z;
    }
    else {
      resolved.z = 0.0
    }

    if (msg.error_x !== undefined) {
      resolved.error_x = msg.error_x;
    }
    else {
      resolved.error_x = 0.0
    }

    if (msg.error_y !== undefined) {
      resolved.error_y = msg.error_y;
    }
    else {
      resolved.error_y = 0.0
    }

    if (msg.error_z !== undefined) {
      resolved.error_z = msg.error_z;
    }
    else {
      resolved.error_z = 0.0
    }

    if (msg.flag1 !== undefined) {
      resolved.flag1 = msg.flag1;
    }
    else {
      resolved.flag1 = false
    }

    if (msg.flag2 !== undefined) {
      resolved.flag2 = msg.flag2;
    }
    else {
      resolved.flag2 = false
    }

    if (msg.flag3 !== undefined) {
      resolved.flag3 = msg.flag3;
    }
    else {
      resolved.flag3 = false
    }

    return resolved;
    }
};

module.exports = Tracker;

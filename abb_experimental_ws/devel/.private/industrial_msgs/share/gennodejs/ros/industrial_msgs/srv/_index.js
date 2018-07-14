
"use strict";

let GetRobotInfo = require('./GetRobotInfo.js')
let StartMotion = require('./StartMotion.js')
let SetDrivePower = require('./SetDrivePower.js')
let CmdJointTrajectory = require('./CmdJointTrajectory.js')
let StopMotion = require('./StopMotion.js')
let SetRemoteLoggerLevel = require('./SetRemoteLoggerLevel.js')

module.exports = {
  GetRobotInfo: GetRobotInfo,
  StartMotion: StartMotion,
  SetDrivePower: SetDrivePower,
  CmdJointTrajectory: CmdJointTrajectory,
  StopMotion: StopMotion,
  SetRemoteLoggerLevel: SetRemoteLoggerLevel,
};

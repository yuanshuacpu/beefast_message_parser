// Copyright (c) 2024, Ontoptech Inc.
// All rights reserved.
// Author: Rockey Shao

#ifndef BEEFAST_MESSAGE_PARSER_ROBOT_MESSAGE_HPP_
#define BEEFAST_MESSAGE_PARSER_ROBOT_MESSAGE_HPP_

#include <sstream>
#include <iostream>
namespace beefast_message_parser
{
  // 下位机消息功能标识位
  const uint8_t kMessageMotionFunctionId = 0x02;     //修改
  const uint8_t kMessageAlarmFunctionId = 0x60;

  const uint8_t kStop = 1; 
  const uint8_t kBackwardAndLeft = 2;
  const uint8_t kBackwardAndRight = 3;
  const uint8_t kForward = 4;

  struct __attribute__((packed)) AlarmMessage {
    uint8_t   collsion_data;
    uint8_t   checksum;
  };

  struct __attribute__((packed)) MotionMessage {   //
    // int16_t  left_encoder;
    // int16_t  right_encoder;
    int16_t velocity_left;
    int16_t velocity_right;
    int16_t  angle_x; // 角度
    int16_t  angle_y;
    int16_t  angle_z;
    int16_t  gyro_x;  // 角速度
    int16_t  gyro_y;
    int16_t  gyro_z;
    int16_t  accel_x; // 线加速度
    int16_t  accel_y;
    int16_t  accel_z;
    int32_t  quat_w;  // 四元组
    int32_t  quat_x;
    int32_t  quat_y;
    int32_t  quat_z;
    int8_t   checksum;
  };

  // std::string output_motion_message(const MotionMessage& message) {
  //   std::stringstream ss;  
  //   ss << "left_encoder:" << message.left_encoder << " right_encoder: " 
  //      << message.right_encoder << " angle_x: " << message.angle_x
  //      <<" angle_y:" << message.angle_y << " angle_z:" << message.angle_z
  //      << " gyro_x:" << message.gyro_x << " gyro_y:" << message.gyro_y
  //      << " gyro_z:" << message.gyro_z << " accel_x:" << message.accel_x
  //      << " accel_y:" << message.accel_y << " accel_z:" << message.accel_z
  //      << " quat_w:" << message.quat_w << " quat_x:" << message.quat_x 
  //      << " quat_y:" << message.quat_y << " quat_z:" << message.quat_z
  //      << " checksum:" << message.checksum;
  //   return ss.str();
  // }

}  // namespace beefast_message_parser
#endif // BEEFAST_MESSAGE_PARSER_ROBOT_MESSAGE_HPP_
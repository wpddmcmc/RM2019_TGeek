
/**
 * Robomaster Vision program of Autocar
 * Copyright (c) 2019, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */

#pragma once

// SDK
#include "serial_device.h" 

// ROS
#include <ros/ros.h>
#include "roborts_msgs/InfoFromCar.h"
#include "roborts_msgs/InfoToCar.h"

/*
 * serial read proto
 */
typedef struct
{
  uint8_t sof;                    // 帧头 = 0A
  uint8_t robot_id;               // 机器人id
  uint8_t cmd_mode;               // 命令模式
  int16_t yaw_angle;              // 单位：角度 x 1000
  int16_t pitch_angle;            // 单位：角度 x 1000
  int16_t yaw_rate;               // 单位：度/秒 x 1000
  int16_t pitch_rate;             // 单位：度/秒 x 1000
  uint8_t end;                    // 帧尾 = B0
}__attribute__((packed)) data_read_t;

/*
 * serial write proto
 */
typedef struct
{
  uint8_t sof;                     // 帧头 = 0C
  uint8_t task_id;                 // 任务id，1 装甲识别 2 打符 3 弹药箱
  uint8_t vision_data_status;      // 数据状态
  int16_t vision_data_x;           // 单位: mm
  int16_t vision_data_y;           // 单位: mm
  int16_t vision_data_z;           // 单位: mm
  uint8_t cmd_mode;                // 控制模式，1 绝对角度 2 相对角度 3 绝对速度 4 相对速度
  int16_t cmd_gimbal_yaw;          // 云台控制，单位同接收
  int16_t cmd_gimbal_pitch;        // 云台控制，单位同接收
  uint8_t end;                     // 帧尾 = D0
}__attribute__((packed)) data_write_t;


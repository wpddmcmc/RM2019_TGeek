/**
 * Robomaster Vision program of Autocar
 * Copyright (c) 2019, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */

#include "serial_node.h"
#include <thread>

#define PRINT_READ_INFO
#define PRINT_WRITE_INFO

namespace roborts_serial
{

class SerialNode
{
  roborts_sdk::SerialDevice serial;

  //thread
  std::thread read_thread_;
  std::thread write_thread_;

  //data
  data_read_t data_read;
  data_write_t data_write;

  // ros
  ros::NodeHandle ros_nh;
  ros::Publisher  ros_pub_info;
  ros::Subscriber ros_sub_cmd;

  roborts_msgs::InfoFromCar pubData;
  
public:
  SerialNode(): serial("/dev/robomaster",115200)
  {
    ros_pub_info = ros_nh.advertise<roborts_msgs::InfoFromCar>("car_info", 100);
    ros_sub_cmd  = ros_nh.subscribe("vision_data", 1, &SerialNode::UpdateMsg, this);

    read_thread_ = std::thread(&SerialNode::ReadExecutor, this);
    read_thread_ = std::thread(&SerialNode::WriteExecutor, this);
  }

  void UpdateMsg(const roborts_msgs::InfoToCar::ConstPtr & info)
  {
    // todo: 可能要加锁
    data_write.sof                = 0x0C;
    data_write.end                = 0xD0;
    data_write.task_id            = info->task_id;
    data_write.vision_data_status = info->vision_data_status;
    data_write.vision_data_x      = info->vision_data_x;
    data_write.vision_data_y      = info->vision_data_y;
    data_write.vision_data_z      = info->vision_data_z;
    data_write.cmd_mode           = info->cmd_mode;
    
    switch(data_write.cmd_mode) 
    {
    case 1: // 绝对角度
      data_write.cmd_gimbal_yaw   = (info->cmd_gimbal_yaw   + pubData.yaw_angle)  * 1000;
      data_write.cmd_gimbal_pitch = (info->cmd_gimbal_pitch + pubData.pitch_angle)* 1000;
      break;
    case 2: // 相对角度
      data_write.cmd_gimbal_yaw   = info->cmd_gimbal_yaw   * 1000;
      data_write.cmd_gimbal_pitch = info->cmd_gimbal_pitch * 1000;
      break;
    case 3: // 绝对速度
      data_write.cmd_gimbal_yaw   = (info->cmd_gimbal_yaw   + pubData.yaw_rate)   * 1000;
      data_write.cmd_gimbal_pitch = (info->cmd_gimbal_pitch + pubData.pitch_rate) * 1000;
      break;
    case 4: // 相对速度
      data_write.cmd_gimbal_yaw   = info->cmd_gimbal_yaw   * 1000;
      data_write.cmd_gimbal_pitch = info->cmd_gimbal_pitch * 1000;
      break;
    default:
      ROS_ERROR("cmd_mode error. gimbal control failed.");
      break;
    }
  }

  void ReadExecutor()
  {
    ROS_WARN("Serial ReadExecutor init OK.");

    int len = sizeof(data_read_t);
    uint8_t buff_read[len];
    uint8_t buff_read_fix[len];

    while(ros::ok())
    {
      serial.Read(buff_read, len);  // 设备未上电时，Read()暂停

      for(int i = 0; i < len; ++i)
      {
        if(buff_read[i] == 0x0A)
        {
          memcpy(buff_read_fix, buff_read + i, len - i);
          memcpy(buff_read_fix + len - i, buff_read, i);
          memcpy(&data_read, buff_read_fix, len);        
        }
      }

      if(data_read.sof == 0x0A && data_read.end == 0xB0)
      {
        pubData.robot_id    = data_read.robot_id;
        pubData.cmd_mode    = data_read.cmd_mode;
        pubData.yaw_angle   = data_read.yaw_angle   / 1000.;
        pubData.pitch_angle = data_read.pitch_angle / 1000.;
        pubData.yaw_rate    = data_read.yaw_rate    / 1000.;
        pubData.pitch_rate  = data_read.pitch_angle / 1000.;

#ifdef PRINT_READ_INFO
        ROS_WARN("serial reading...");
        ROS_INFO("robot_id: %d",    data_read.robot_id);
        ROS_INFO("cmd_mode: %d",    data_read.cmd_mode);
        ROS_INFO("yaw_angle: %d",   data_read.yaw_angle);
        ROS_INFO("pitch_angle: %d", data_read.pitch_angle);
        ROS_INFO("yaw_rate: %d",    data_read.yaw_rate);
        ROS_INFO("pitch_rate: %d",  data_read.pitch_rate);
#endif
      }
      
      ros_pub_info.publish(pubData);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  void WriteExecutor()
  {
    ROS_WARN("Serial WriteExecutor init OK.");

    int len = sizeof(data_write_t);
    uint8_t buff_write[len];

    while(ros::ok())
    {
      memcpy(buff_write, &data_write, len);
      serial.Write(buff_write, len);

#ifdef PRINT_WRITE_INFO
      ROS_WARN("serial writing...");
      ROS_INFO("task_id: %d",             data_write.task_id);
      ROS_INFO("vision_data_status: %d",  data_write.vision_data_status);
      ROS_INFO("vision_data_x: %d",       data_write.vision_data_x);
      ROS_INFO("vision_data_y: %d",       data_write.vision_data_y);
      ROS_INFO("vision_data_z: %d",       data_write.vision_data_z);
      ROS_INFO("cmd_mode: %d",            data_write.cmd_mode);
      ROS_INFO("cmd_gimbal_yaw: %d",      data_write.cmd_gimbal_yaw);
      ROS_INFO("cmd_gimbal_pitch: %d",    data_write.cmd_gimbal_pitch);
#endif

    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

};

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "serial_Write");
    
    roborts_serial::SerialNode serialnode;

    ros::AsyncSpinner async_spinner(2);
    async_spinner.start();
    ros::waitForShutdown();

  return 0;
}

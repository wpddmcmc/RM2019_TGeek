/*-------------------------------------------------------------
brief  :  Mercure Camera driver 
author :  XDU Robomaster, skystarry
date   :  2019.05.29
---------------------------------------------------------------*/

#pragma once

#include <ros/ros.h>
#include <ros/package.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "DxImageProc.h"
#include "GxIAPI.h"

#include <opencv2/opencv.hpp>
#include <chrono>

namespace camera{

#define ACQ_TRANSFER_SIZE       (64 * 1024)
#define ACQ_TRANSFER_NUMBER_URB  64

#define ACQ_FRAME_WIDTH          1920 
#define ACQ_FRAME_HEIGHT         1200

struct Param
{
  int exp_auto_;
  int w_auto_;
  int gain_auto_;
  
  double exp_time_; // us
  double w_red_;
  double w_green_;
  double w_blue_;
  double gain_;
};

class MercureDriver
{
  GX_STATUS     status_;
  GX_DEV_HANDLE device_;

  PGX_FRAME_BUFFER pFrameBuffer_;
  uint8_t* rgbImagebuf_; 
  cv::Mat Image_;

  uint64_t bufferNum_;
  
  Param param_;

  ros::NodeHandle nh_;
  sensor_msgs::ImagePtr img_msg;
  image_transport::Publisher img_pubs_;

public:
  explicit MercureDriver();
  GX_STATUS init_sdk();
  void GetVision();
  void ReadCamera();
  void LoadParam();
  ~MercureDriver();
};

} // namespace camera
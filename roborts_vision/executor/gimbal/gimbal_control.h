
#pragma once

#include <ros/ros.h>
#include <ros/package.h>

#include <iostream>
#include <opencv2/opencv.hpp>

constexpr float GRAVITY = 9.78;

/**
 * @brief The class can make a transformation: the 3D position of enemy -->  pitch,yaw angle of gimbal.
 * For more derails, see projectile_model.pdf
 * TODO: add enemy motion estimation
 */

class GimbalContrl
{
  /**
   * @brief Calculate the actual y value with air resistance
   * @param x the distanc
   * @param v Projectile velocity
   * @param angle Pitch angle
   * @return The actual y value in the gimbal coordinate
   */
  float BulletModel(float x,float v,float angle);
  /**
   * @brief Get the gimbal control angle
   * @param x Distance from enemy(the armor selected to shoot) to gimbal
   * @param y Value of y in gimbal coordinate.
   * @param v Projectile velocity
   * @return Gimbal pitch angle
   */
  float GetPitch(float x,float y,float v);

 public:
  GimbalContrl()
  {
    std::string file_name = ros::package::getPath("roborts_vision") + "/gimbal/gimbal.xml";
    cv::FileStorage fs(file_name, cv::FileStorage::READ);
    if(!fs.isOpened())
        ROS_ERROR ("Cannot open armor param file, please check if the file is exist");

    // algorithm info
    fs["offset_x"] >> offset_.x;
    fs["offset_y"] >> offset_.y;
    fs["offset_z"] >> offset_.z;

    fs["offset_yaw"]   >> offset_yaw_;
    fs["offset_pitch"] >> offset_pitch_;

    fs["init_v"] >> init_v_;
    fs["init_k"] >> init_k_;

  }
  /**
   * @brief Get the gimbal control info.
   * @param postion Enemy position(actually it should be the target armor).
   * @param pitch Input and output actual pitch angle
   * @param yaw Input and output actual yaw angle
   */
  void SolveContrlAgnle(cv::Point3f &postion, float &yaw, float &pitch);

 private:
  //! Transformation matrix between camera coordinate system and gimbal coordinate system.
  //! Translation unit: cm
  cv::Point3f offset_;
  //! Rotation matrix unit: degree
  float offset_pitch_;
  float offset_yaw_;

  //! Initial value
  float init_v_;
  float init_k_;

};


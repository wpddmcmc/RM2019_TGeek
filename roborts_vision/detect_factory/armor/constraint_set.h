/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#pragma once

#include <ros/ros.h>
#include <ros/package.h>

#include <vector>
#include <list>

#include <opencv2/opencv.hpp>
#include "cv_toolbox.h"


enum Armor_Twist { STILL = 1, LOW_MOVE = 2, MID_MOVE = 3, FAST_MOVE = 4 }; // 速度信息

enum ArmorType { BIGARMOR = 1, SMALLARMOR = 2 };

/**
 *  This class describes the armor information, including maximum bounding box, vertex, standard deviation.
 */
class ArmorInfo {
 public:
  ArmorInfo(cv::RotatedRect armor_rect, Armor_Twist st) {
    rect = armor_rect;
    state = st;
  }
 public:
  cv::RotatedRect rect;
  Armor_Twist state;
  ArmorType type_;
};

/**
 * @brief This class achieved functions that can help to detect armors of RoboMaster vehicle.
 */
class ConstraintSet {
 public:
  ConstraintSet();

  void Detect(uint8_t& , std::vector<ArmorInfo>& );
  
  void DetectLights(const cv::Mat &src, std::vector<cv::RotatedRect> &lights);
 
  void FilterLights(std::vector<cv::RotatedRect> &lights);
 
  void PossibleArmors(const std::vector<cv::RotatedRect> &lights, std::vector<ArmorInfo> &armors);
  
  void FilterArmors(std::vector<ArmorInfo> &armors);
  
  ArmorInfo SlectFinalArmor(std::vector<ArmorInfo> &armors);

  void CalcControlInfo(const ArmorInfo & armor, cv::Point3f &target_3d);

  void CalcArmorInfo(std::vector<cv::Point2f> &armor_points, cv::RotatedRect left_light, cv::RotatedRect right_light);
  
  void SolveArmorCoordinate(const float width, const float height);
  
  ~ConstraintSet();

  cv::Mat src_img_;   // 用于调试
  cv::Mat gray_img_;  // 用于调试
  
private:

  std::shared_ptr<CVToolbox> cv_toolbox_;
  
  // Parameters come form .prototxt file
  bool enable_debug_;
  bool using_hsv_;
  bool enemy_color_;

  //! Use for debug
  cv::Mat show_lights_before_filter_;
  cv::Mat show_lights_after_filter_;
  cv::Mat show_armors_befor_filter_;
  cv::Mat show_armors_after_filter_;

  // image threshold parameters
	float light_threshold_;
  float blue_threshold_;
  float red_threshold_;

  // light threshold parameters
  float light_min_area_;
	float light_max_area_;
	float light_min_angle_;
  float light_max_angle_;
	float light_min_angle_diff_;
	float light_max_angle_diff_;
	float light_min_aspect_ratio_;
	float light_max_aspect_ratio_;

	// armor threshold parameters
	float light_max_width_diff_;
	float light_max_height_diff_;
  float armor_min_area_;
	float armor_max_area_;
	float armor_min_angle_;
  float armor_max_angle_;
  float armor_light_angle_diff_;
	float armor_min_ratio_;
	float armor_max_ratio_;
	float armor_min_aspect_ratio_;
	float armor_max_aspect_ratio_;
	float filter_armor_area_;

};


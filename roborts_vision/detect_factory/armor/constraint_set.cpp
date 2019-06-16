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

#include "constraint_set.h"


// 方便输出一些调试信息
#define LINE(str)  //std::cout << "Code Line:" << __LINE__ << "\t" << str << std::endl;
#define LINE_INFO(str,str2) //std::cout << "Code Line:" << __LINE__ << "\t" << str <<":\t"<< str2 << std::endl;

ConstraintSet::ConstraintSet()
{
  std::string file_name = ros::package::getPath("roborts_vision") + "/detect_factory/armor/constraint_set.xml";
  cv::FileStorage fs(file_name, cv::FileStorage::READ);
  if(!fs.isOpened())
      ROS_ERROR ("Cannot open armor param file, please check if the file is exist");

  // algorithm info
  fs["enable_debug"] >> enable_debug_;
  fs["enemy_color"] >> enemy_color_;
  fs["using_hsv"] >> using_hsv_;

	// image threshold parameters
  fs["light_threshold"] >> light_threshold_;
  fs["blue_threshold"] >> blue_threshold_;
  fs["red_threshold"] >> red_threshold_;

  // light threshold parameters
  fs["light_min_area"] >> light_min_area_;
	fs["light_max_area"] >> light_max_area_;
	fs["light_min_angle"] >> light_min_angle_;
  fs["light_max_angle"] >> light_max_angle_;  
	fs["light_min_angle_diff"] >> light_min_angle_diff_; 
	fs["light_max_angle_diff"] >> light_max_angle_diff_;
	fs["light_min_aspect_ratio"] >> light_min_aspect_ratio_;
	fs["light_max_aspect_ratio"] >> light_max_aspect_ratio_;

	// armor threshold parameters
	fs["light_max_width_diff"] >> light_max_width_diff_;
	fs["light_max_height_diff"] >> light_max_height_diff_;
  fs["armor_min_area"] >> armor_min_area_;
	fs["armor_max_area"] >> armor_max_area_;
	fs["armor_min_angle"] >> armor_min_angle_;
  fs["armor_max_angle"] >> armor_max_angle_;
  fs["armor_light_angle_diff"] >> armor_light_angle_diff_;
	fs["armor_min_ratio"] >> armor_min_ratio_;
	fs["armor_max_ratio"] >> armor_max_ratio_;
	fs["armor_min_aspect_ratio"] >> armor_min_aspect_ratio_;
	fs["armor_max_aspect_ratio"] >> armor_max_aspect_ratio_;
	fs["filter_armor_area"] >> filter_armor_area_;
}

void ConstraintSet::Detect(uint8_t &vision_data_status, std::vector<ArmorInfo>& armors) 
{	
  std::vector<cv::RotatedRect> lights;

  cv::cvtColor(src_img_, gray_img_, CV_BGR2GRAY);
  if (enable_debug_) {
      show_lights_before_filter_ = cv::Mat::zeros(src_img_.size(), CV_8UC3);
  	  show_lights_after_filter_  = cv::Mat::zeros(src_img_.size(), CV_8UC3);
  	  show_armors_befor_filter_  = src_img_.clone();
  	  show_armors_after_filter_  = src_img_.clone();
      cv::waitKey(1);
    }

    DetectLights(src_img_, lights);
		//ROS_INFO("detect lights number: %d", lights.size());
		if(lights.size()>100) return;

    FilterLights(lights);
    PossibleArmors(lights, armors);
    FilterArmors(armors);
		
    if(!armors.empty()) {
      vision_data_status = 1;
    } else
      vision_data_status = 0;
}

void ConstraintSet::DetectLights(const cv::Mat &src, std::vector<cv::RotatedRect> &lights) 
{
	cv::Mat subtract_color_img;
	cv::Mat binary_brightness_img;
  cv::Mat binary_color_img;
  cv::Mat binary_light_img;

  if(using_hsv_) {
    binary_color_img = cv_toolbox_->DistillationColor(src, enemy_color_, using_hsv_);
    cv::threshold(gray_img_, binary_brightness_img, light_threshold_, 255, CV_THRESH_BINARY);
  }else {
    cv::Mat subtract_color_img;
	  std::vector<cv::Mat> bgr_channel;
	  cv::split(src, bgr_channel);
	
	  if (enemy_color_ == 0) // red
		  cv::subtract(bgr_channel[2], bgr_channel[1], subtract_color_img);
	  else
		  cv::subtract(bgr_channel[0], bgr_channel[1], subtract_color_img);
		
    cv::threshold(gray_img_, binary_brightness_img, light_threshold_, 255, CV_THRESH_BINARY);
    
    float thresh;
    if (enemy_color_ == 0) // red
      thresh = red_threshold_;
    else
      thresh = blue_threshold_;

    cv::threshold(subtract_color_img, binary_color_img, thresh, 255, CV_THRESH_BINARY);
    binary_light_img = binary_color_img & binary_brightness_img;
	
  }
  std::vector<std::vector<cv::Point>> contours_light;
	cv::findContours(binary_light_img, contours_light, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

  std::vector<std::vector<cv::Point>> contours_brightness;
	cv::findContours(binary_brightness_img, contours_brightness, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
  
  lights.reserve(contours_brightness.size());

  for (unsigned int i = 0; i < contours_light.size(); ++i) {
    for (unsigned int j = 0; j < contours_brightness.size(); ++j) {
      	if (cv::pointPolygonTest(contours_brightness[j], contours_light[i][0], true) >= 0.0) 
        {
          cv::RotatedRect single_light = cv::minAreaRect(contours_brightness[j]);
          lights.push_back(single_light);
          if (enable_debug_)
            cv_toolbox_->DrawRotatedRect(show_lights_before_filter_, single_light, cv::Scalar(0,255,0), 1);
          break;
        }
    } // for j loop
  } // for i loop

  //if (enable_debug_)
    //cv::imshow("show_lights_before_filter", show_lights_before_filter_);
}

void ConstraintSet::FilterLights(std::vector<cv::RotatedRect> &lights) 
{
  std::vector<cv::RotatedRect> light_rects;

#pragma omp parallel for 
  for(uchar i = 0; i < lights.size(); i++ ){
  	cv_toolbox_->adjustRect(lights[i]); 	
  }
  
  for (const auto &armor_light : lights)
	{
    auto rect = std::minmax(armor_light.size.width, armor_light.size.height);
  	auto light_aspect_ratio = rect.second / rect.first;
    auto angle = armor_light.angle;

		if ( // 80 <= abs(angle) && abs(angle) <= 90   // 高速水平移动的灯条,带有拖影  // 特殊情况,无论横竖, 旧版本有这一行代码
		      light_aspect_ratio <= 2.5
		   && armor_light.size.area() > light_min_area_ // 1.0
		   && armor_light.size.area() < light_max_area_ * src_img_.size().height * src_img_.size().width) // 0.04
		{
			light_rects.push_back(armor_light); // 高速水平移动的灯条
      if (enable_debug_)
		  	cv_toolbox_->DrawRotatedRect(show_lights_after_filter_, armor_light, cv::Scalar(255,0,0), 1);
		}
     // 针对灯条细小的情况, 没有最大比例的判断, 较为理想的灯条
		else if(armor_light.size.area() >= light_min_area_ // 1.0
		   		&& armor_light.size.area() < 100000  //light_max_area_ * src_img_.size().height * src_img_.size().width // 0.04
		   		&& abs(angle) < light_max_angle_) // 与垂直的偏角17.5 , 这里是可以取消/2的,进一步细化
		{
			light_rects.push_back(armor_light); // 接近于垂直的灯条, 由于阈值不够合理, 细小的灯条
      if (enable_debug_)
			  cv_toolbox_->DrawRotatedRect(show_lights_after_filter_, armor_light, cv::Scalar(0,255,0), 1);
	  }
      // 检测最为平凡的情况
    else if (//light_aspect_ratio < _para.light_max_aspect_ratio  // 6.8
              armor_light.size.area() >= light_min_area_ // 1.0
			     && armor_light.size.area() < light_max_area_ * src_img_.size().height * src_img_.size().width // 0.04
			     && abs(angle) < light_max_angle_) // 与垂直的偏角35 
    {
      light_rects.push_back(armor_light);
      if (enable_debug_)
			  cv_toolbox_->DrawRotatedRect(show_lights_after_filter_, armor_light, cv::Scalar(0,0,255), 1);
		}
  }
  if (enable_debug_)
		cv::imshow("lights_after_filter", show_lights_after_filter_);
	
  lights = light_rects;
}

void ConstraintSet::PossibleArmors(const std::vector<cv::RotatedRect> &lights, std::vector<ArmorInfo> &armor_vector) {
    for (int i = 0; i < lights.size(); ++i) {
    for (int j = i; j < lights.size(); ++j) {
			auto rect1 = std::minmax(lights[i].size.width, lights[i].size.height);
    	auto light_aspect_ratio1 = rect1.second / rect1.first;
			auto rect2 = std::minmax(lights[j].size.width, lights[j].size.height);
    	auto light_aspect_ratio2 = rect2.second / rect2.first;

			auto angle_diff  = abs(lights[i].angle - lights[j].angle);
			auto height_diff = abs(lights[i].size.height - lights[j].size.height) / std::max(lights[i].size.height, lights[j].size.height);
			auto width_diff  = abs(lights[i].size.width - lights[j].size.width) / std::max(lights[i].size.width, lights[j].size.width);

// 快速平移的情况 Fast Move
	// 2个严格平行
			if (1 < light_aspect_ratio1 && light_aspect_ratio1 <= 2.5
			    && 1 < light_aspect_ratio2 && light_aspect_ratio2 <= 2.5
			    && abs(lights[i].angle) == 90 && abs(lights[j].angle) == 90     // 角度为0
			    && static_cast<int>(abs(angle_diff)) % 180 == 0
			    && height_diff < 0.5 && width_diff < 0.5)	
			{
				cv::RotatedRect possible_rect;
				if (lights[i].center.x < lights[j].center.x)
					possible_rect = cv_toolbox_->boundingRRectFast(lights[i],lights[j]);
				else
					possible_rect = cv_toolbox_->boundingRRectFast(lights[j],lights[i]);
				
				auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
				auto armor_angle = possible_rect.angle;
				auto armor_area = possible_rect.size.area();
				// auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

				if (armor_area > armor_min_area_ 
					&& armor_ratio < 4.5                // 经验参数
					&& abs(armor_angle) < 20 )          // 经验参数
				{	
					Armor_Twist armor_twist = FAST_MOVE;
					LINE("[快速移动] armor")
					ArmorInfo armor(possible_rect, armor_twist);
					armor_vector.push_back(armor);
				} // get_armor
			}// 2个严格平行
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// 2个当中有一个略不平行
			else if (1 < light_aspect_ratio1 && light_aspect_ratio1 <= 2.5
			     	 && 1 < light_aspect_ratio2 && light_aspect_ratio2 <= 2.5
			    	 && ( (abs(lights[i].angle) == 90 && abs(lights[j].angle) > 80) || (abs(lights[i].angle) > 80 && abs(lights[j].angle) == 90))
			    	 && height_diff < 0.5 && width_diff < 0.5)		
			{
				cv::RotatedRect possible_rect;
				if (lights[i].center.x < lights[j].center.x)
					possible_rect = cv_toolbox_->boundingRRectFast(lights[i],lights[j]);
				else
					possible_rect = cv_toolbox_->boundingRRectFast(lights[j],lights[i]);
				
				auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
				auto armor_angle = possible_rect.angle;
				auto armor_area = possible_rect.size.area();
				
				// auto armor_light_angle_diff = abs(lights[i].angle) == 90 ? 
				//						 	     abs(armor_angle) + abs(armor_angle - lights[j].angle - 90); // 左右灯条的积累差

				if (armor_area > armor_min_area_
				    && armor_ratio < 4.5                // 经验参数
					&& abs(armor_angle) < 20 )          // 经验参数
				{	
					Armor_Twist armor_twist = FAST_MOVE;
					LINE("[快速移动] armor")
					ArmorInfo armor(possible_rect, armor_twist);
					armor_vector.push_back(armor);
				} // get_armor
			}// 2个当中有一个略不平行
// 快速平移的情况 Fast Move

// 中速平移的情况 Mid Move
		// 2个严格平行
			else if ( ( (light_aspect_ratio1 == 1 && light_aspect_ratio2 <= 1.5) 
					 || (light_aspect_ratio1 <= 1.5 && light_aspect_ratio2 == 1) )   // 其中一个为正方形
					 && static_cast<int>(abs(angle_diff)) % 90 == 0               	 // 角度差为0
					 && static_cast<int>(abs(lights[i].angle)) % 90 == 0          	 // 角度为0 或 90
					 && static_cast<int>(abs(lights[j].angle)) % 90 == 0          	 // 角度为0 或 90
					 && height_diff < 0.5 && width_diff < 0.5)               	     // 形状差异
			{
				cv::RotatedRect possible_rect;
				if (lights[i].center.x < lights[j].center.x)
					possible_rect = cv_toolbox_->boundingRRectFast(lights[i], lights[j]);
				else
					possible_rect = cv_toolbox_->boundingRRectFast(lights[j], lights[i]);

				auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
				auto armor_angle = possible_rect.angle;
				auto armor_area = possible_rect.size.area();
				// auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

				if (armor_area > armor_min_area_	    // 经验参数
					&& armor_ratio < 4                      // 经验参数 （步兵应该只有3, 英雄可能会到5）
					&& abs(armor_angle) <  20 )             // 经验参数
					{
						Armor_Twist armor_twist = MID_MOVE;
						LINE("[中等速度] armor")
						ArmorInfo armor(possible_rect, armor_twist);
						armor_vector.push_back(armor);
					} // get_armor
			}
			// 2个严格平行

			// 1个竖着 1个横着
			else if (1 < light_aspect_ratio1 && light_aspect_ratio1 < 1.3
					 && 1 < light_aspect_ratio2 && light_aspect_ratio2 < 1.3
					 && static_cast<int>(abs(angle_diff)) % 180 == 90           // 角度差为0
					 && ((abs(lights[i].angle) == 0 && abs(lights[j].angle) == 90) || (abs(lights[i].angle) == 90 && abs(lights[j].angle) == 0))  // 角度1个为0 1个为90
					 && height_diff < 0.5 && width_diff < 0.5)               	  // 形状差异
			{
				cv::RotatedRect possible_rect;
				if (lights[i].center.x < lights[j].center.x)
					possible_rect = cv_toolbox_->boundingRRectFast(lights[i], lights[j]);
				else
					possible_rect = cv_toolbox_->boundingRRectFast(lights[j], lights[i]);

				auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
				auto armor_angle = possible_rect.angle;
				auto armor_area = possible_rect.size.area();
				// auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

				if (armor_area > armor_min_area_			// 经验参数
					&& armor_ratio < 4                      // 经验参数 （步兵应该只有3, 英雄可能会到5）
					&& abs(armor_angle) <  20 )             // 经验参数
					{
						Armor_Twist armor_twist = MID_MOVE;
						LINE("[中等速度] armor")
						ArmorInfo armor(possible_rect, armor_twist);
						armor_vector.push_back(armor);
					} // get_armor
			}// 1个竖着 1个横着
// 中速平移的情况 Mid Move

// 慢速移动的情况 Low Move
		// 都是竖着的
			else if (1 < light_aspect_ratio1 && light_aspect_ratio1 < 1.5
					 && 1 < light_aspect_ratio2 && light_aspect_ratio2 < 1.5
					 && static_cast<int>(abs(angle_diff)) % 180 == 0               // 角度差为0
					 && abs(lights[i].angle) == 0 && abs(lights[j].angle) == 0     // 角度为0 或 180
					 && height_diff < 0.5 && width_diff < 0.5)                  	 // 形状差异
			{
				cv::RotatedRect possible_rect;
				if (lights[i].center.x < lights[j].center.x)
					possible_rect = cv_toolbox_->boundingRRectSlow(lights[i], lights[j]);
				else
					possible_rect = cv_toolbox_->boundingRRectSlow(lights[j], lights[i]);

				auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
				auto armor_angle = possible_rect.angle;
				auto armor_area = possible_rect.size.area();
				// auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

				if (armor_area > armor_min_area_	 // 经验参数
					&& armor_ratio < 4                   // 经验参数 （步兵应该只有3, 英雄可能会到5）
					&& abs(armor_angle) <  20 )          // 经验参数
					{
						Armor_Twist armor_twist = LOW_MOVE;
						LINE("[缓慢移动] armor")
						ArmorInfo armor(possible_rect, armor_twist);
						armor_vector.push_back(armor);
					} // get_armor
			}
		// 都是竖着的

		// 其中一块略有倾斜
			else if (1 < light_aspect_ratio1 && light_aspect_ratio1 < 1.5
					 && 1 < light_aspect_ratio2 && light_aspect_ratio2 < 1.5
					 &&	((abs(lights[i].angle) == 0  && abs(lights[j].angle) < 10) || (abs(lights[i].angle) < 10  && abs(lights[j].angle) == 0)) // 角度为0 或 180
					 && height_diff < 0.5 && width_diff < 0.5)                  	 // 形状差异
			{
				cv::RotatedRect possible_rect;
				if (lights[i].center.x < lights[j].center.x)
					possible_rect = cv_toolbox_->boundingRRectFast(lights[i], lights[j]);
				else
					possible_rect = cv_toolbox_->boundingRRectFast(lights[j], lights[i]);

				auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
				auto armor_angle = possible_rect.angle;
				auto armor_area = possible_rect.size.area();
				auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

				if (armor_area > armor_min_area_			// 经验参数
					&& armor_ratio < 4                      // 经验参数 （步兵应该只有3, 英雄可能会到5）
					&& abs(armor_angle) <  20               // 经验参数
					&& armor_light_angle_diff < 20 )
					{
						Armor_Twist armor_twist = LOW_MOVE;
						LINE("[缓慢移动] armor")
						ArmorInfo armor(possible_rect, armor_twist);
						armor_vector.push_back(armor);
					} // get_armor
			}
		// 其中一块略有倾斜

// 慢速移动的情况 Low Move

// 平凡的情况
	// 灯条近乎平行,至少在同一侧
			else if (lights[i].angle * lights[j].angle >= 0          // 灯条近乎同侧 , 或者有一个为0
				     && abs(angle_diff) < 30                           //  light_max_angle_diff_   // 20   // 18   这些都要换成相对值
					// && height_diff < _para.light_max_height_diff      // 20  不需要宽度
					)
			{
				cv::RotatedRect possible_rect;
				// 2灯条近乎平行 中速移动
				if (1 < light_aspect_ratio1 && light_aspect_ratio1 <= 1.5
					&& 1 < light_aspect_ratio2 && light_aspect_ratio2 <= 1.5
					&& abs(light_aspect_ratio1 - light_aspect_ratio2) < 0.5 )
				{
					// 2灯条近乎平行 中速移动
					if (abs(lights[i].angle) > 60 && abs(lights[j].angle) > 60)
					{
						if (lights[i].center.x < lights[j].center.x)
							possible_rect = cv_toolbox_->boundingRRectFast(lights[i], lights[j]);
						else
				    	possible_rect = cv_toolbox_->boundingRRectFast(lights[j], lights[i]);

						auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
						auto armor_angle = possible_rect.angle;
						auto armor_area = possible_rect.size.area();
						auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

						if (armor_area > armor_min_area_
							&& armor_ratio > 1                                       // _para.small_armor_min_ratio   // 1.5
				   			&& armor_ratio < armor_max_aspect_ratio_                 // 3.0
				   			&& abs(armor_angle) < armor_max_angle_
				   			&& armor_light_angle_diff < armor_light_angle_diff_ // 应该要更为严格
						){
							Armor_Twist armor_twist = MID_MOVE;
							LINE("armor同侧,这是中速移动的armor,1-1.5,平躺")
							ArmorInfo armor(possible_rect, armor_twist);
							armor_vector.push_back(armor);
						}
					}// 2灯条近乎平行 中速移动

					// 2灯条近乎平行 慢速移动
					else if (abs(lights[i].angle) < 30 && abs(lights[j].angle) < 30)
					{
						if (lights[i].center.x < lights[j].center.x)
							possible_rect = cv_toolbox_->boundingRRectSlow(lights[i], lights[j]);
						else
				    	possible_rect = cv_toolbox_->boundingRRectSlow(lights[j], lights[i]);

						auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
						auto armor_angle = possible_rect.angle;
						auto armor_area = possible_rect.size.area();
						auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

						if (armor_area > armor_min_area_
							&& armor_ratio > 1                                       // _para.small_armor_min_ratio   // 1.5
				   			&& armor_ratio < armor_max_aspect_ratio_                   // 3.0
				   			&& abs(armor_angle) < armor_max_angle_
				   			&& armor_light_angle_diff < armor_light_angle_diff_ // 应该要更为严格
						){
							Armor_Twist armor_twist = LOW_MOVE;
							LINE("[缓慢移动] armor")
							ArmorInfo armor(possible_rect, armor_twist);
							armor_vector.push_back(armor);
						}
					}// 2灯条近乎平行 慢速移动
				}// 2灯条近乎平行 中速移动

				// 2灯条近乎平行 快速移动
				else if (1 < light_aspect_ratio1 && light_aspect_ratio1 < 2.5
						 && 1 < light_aspect_ratio2 && light_aspect_ratio2 < 2.5
						 && abs(light_aspect_ratio1 - light_aspect_ratio2) < 1)
				{
					// 2灯条近乎平行 快速移动
					if (abs(lights[i].angle) > 60 && abs(lights[j].angle) > 60)
					{
						if (lights[i].center.x < lights[j].center.x)
							possible_rect = cv_toolbox_->boundingRRectFast(lights[i], lights[j]);
						else
				    	possible_rect = cv_toolbox_->boundingRRectFast(lights[j], lights[i]);

						auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
						auto armor_angle = possible_rect.angle;
						auto armor_area = possible_rect.size.area();
						
						if (armor_area > armor_min_area_
							&& armor_ratio > 1                       // _para.small_armor_min_ratio   // 1.5
				   			&& armor_ratio < armor_max_aspect_ratio_   // 3.0
				   			&& abs(armor_angle) < armor_max_angle_
						){
							Armor_Twist armor_twist = FAST_MOVE;
							LINE("[快速移动] armor")
							ArmorInfo armor(possible_rect, armor_twist);
							armor_vector.push_back(armor);
						}
					} // 2灯条近乎平行 快速移动

					// 2灯条近乎平行 慢速移动
					else if (abs(lights[i].angle) < 30 && abs(lights[j].angle) < 30)
					{
						if (lights[i].center.x < lights[j].center.x)
							possible_rect = cv_toolbox_->boundingRRectSlow(lights[i], lights[j]);
						else
				    	possible_rect = cv_toolbox_->boundingRRectSlow(lights[j], lights[i]);

						auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
						auto armor_angle = possible_rect.angle;
						auto armor_area = possible_rect.size.area();
						auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

						if (armor_area > armor_min_area_
							&& armor_ratio > 1                                       // _para.small_armor_min_ratio   // 1.5
				   			&& armor_ratio < armor_max_aspect_ratio_                   // 3.0
				   			&& abs(armor_angle) < armor_max_angle_
				   			&& armor_light_angle_diff < armor_light_angle_diff_ // 应该要更为严格
						){
							Armor_Twist armor_twist = LOW_MOVE;
							LINE("[缓慢移动] armor")
							ArmorInfo armor(possible_rect, armor_twist);
							armor_vector.push_back(armor);
						}
					}// 2灯条近乎平行 慢速移动
				}// 2灯条近乎平行 快速移动

				else if (light_min_aspect_ratio_ < light_aspect_ratio1    // && light_aspect_ratio1 < _para.light_max_aspect_ratio
						 && light_min_aspect_ratio_ < light_aspect_ratio2 // && light_aspect_ratio2 < _para.light_max_aspect_ratio
						 && (lights[i].center.y + lights[i].size.height / 2) > (lights[j].center.y - lights[j].size.height / 2)
						 && (lights[j].center.y + lights[j].size.height / 2) > (lights[i].center.y - lights[i].size.height / 2)
						 && abs(lights[i].angle) < 30 && abs(lights[j].angle) < 30)
				{
					if (lights[i].center.x < lights[j].center.x)
						possible_rect = cv_toolbox_->boundingRRect(lights[i], lights[j]);
					else
				    possible_rect = cv_toolbox_->boundingRRect(lights[j], lights[i]);

					auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
					auto armor_angle = possible_rect.angle;
					auto armor_area = possible_rect.size.area();
					auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

					if (armor_area > armor_min_area_
						&& armor_ratio > 1                                         // _para.small_armor_min_ratio   // 1.5
				   		&& armor_ratio < 4.5                                       // armor_max_aspect_ratio_   // 3.0
				   		&& abs(armor_angle) < armor_max_angle_
				   		&& armor_light_angle_diff < armor_light_angle_diff_ ) // 应该要更为严格
					{
						LINE_INFO("angle_1",lights[i].angle)
						LINE_INFO("angle_2",lights[j].angle)
						LINE_INFO("angle_diff",angle_diff)
						LINE_INFO("height_diff", height_diff)
						LINE_INFO("width_diff",width_diff)
						LINE_INFO("armor_ratio",armor_ratio)
						LINE_INFO("armor_angle",armor_angle)
						LINE_INFO("armor_light_angle_diff",armor_light_angle_diff)
						
						Armor_Twist armor_twist = STILL;
						LINE("[几乎静止] armor")
						ArmorInfo armor(possible_rect, armor_twist);
						armor_vector.push_back(armor);
					}
				}
			} // 灯条严格平行
			
// 灯条(误差) 并不同侧
			else if (abs(angle_diff) < light_max_angle_diff_ )     // 40
			{
				cv::RotatedRect possible_rect;
				// 2灯条 中速移动
				if (1 < light_aspect_ratio1 && light_aspect_ratio1 <= 1.5
					&& 1 < light_aspect_ratio2 && light_aspect_ratio2 <= 1.5
					&& abs(light_aspect_ratio1 - light_aspect_ratio2) < 0.5 )
				{
					// 2灯条不平行 慢速移动
					if (abs(lights[i].angle) < 30 && abs(lights[j].angle) < 30)
					{
						if (lights[i].center.x < lights[j].center.x)
							possible_rect = cv_toolbox_->boundingRRectSlow(lights[i], lights[j]);
						else
				    	possible_rect = cv_toolbox_->boundingRRectSlow(lights[j], lights[i]);

						auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
						auto armor_angle = possible_rect.angle;
						auto armor_area = possible_rect.size.area();
						auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

						if (armor_area > armor_min_area_
							&& armor_ratio > 1  // _para.small_armor_min_ratio   // 1.5
				   			&& armor_ratio < armor_max_aspect_ratio_   // 3.0
				   			&& abs(armor_angle) < armor_max_angle_
				   			&& armor_light_angle_diff < armor_light_angle_diff_ // 应该要更为严格
						)
						{
							Armor_Twist armor_twist = MID_MOVE;
							LINE("armor不同侧, 中速的armor, 竖直")
							ArmorInfo armor(possible_rect, armor_twist);
							armor_vector.push_back(armor);
						}
					}// 2灯条不平行 慢速移动
				}// 2灯条 中速移动

				// 2灯条近乎平行 快速移动
				else if (1 < light_aspect_ratio1 && light_aspect_ratio1 < 2.5
						 && 1 < light_aspect_ratio2 && light_aspect_ratio2 < 2.5
						 && abs(light_aspect_ratio1 - light_aspect_ratio2) < 1)
				{
					if (abs(lights[i].angle) < 30 && abs(lights[j].angle) < 30)
					{
						if (lights[i].center.x < lights[j].center.x)
							possible_rect = cv_toolbox_->boundingRRectSlow(lights[i], lights[j]);
						else
				    	possible_rect = cv_toolbox_->boundingRRectSlow(lights[j], lights[i]);

						auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
						auto armor_angle = possible_rect.angle;
						auto armor_area = possible_rect.size.area();
						auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

						if (armor_area > armor_min_area_
							&& armor_ratio > 1 // _para.small_armor_min_ratio   // 1.5
				   			&& armor_ratio < armor_max_aspect_ratio_   // 3.0
				   			&& abs(armor_angle) < armor_max_angle_
				   			&& armor_light_angle_diff < armor_light_angle_diff_ )// 应该要更为严格
						{
							Armor_Twist armor_twist = LOW_MOVE;
							LINE("[缓慢移动] armor")
							ArmorInfo armor(possible_rect, armor_twist);
							armor_vector.push_back(armor);
						}
					}// 2灯条近乎平行 慢速移动
				}// 2灯条近乎平行 快速移动

				else if (light_min_aspect_ratio_ < light_aspect_ratio1 //&& light_aspect_ratio1 < _para.light_max_aspect_ratio
						 && light_min_aspect_ratio_ < light_aspect_ratio2 //&& light_aspect_ratio2 < _para.light_max_aspect_ratio
						 && (lights[i].center.y + lights[i].size.height / 2) > (lights[j].center.y - lights[j].size.height / 2)
						 && (lights[j].center.y + lights[j].size.height / 2) > (lights[i].center.y - lights[i].size.height / 2))
				{
					if (lights[i].center.x < lights[j].center.x)
						possible_rect = cv_toolbox_->boundingRRect(lights[i], lights[j]);
					else
				    possible_rect = cv_toolbox_->boundingRRect(lights[j], lights[i]);

					auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
					auto armor_angle = possible_rect.angle;
					auto armor_area = possible_rect.size.area();
					auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

					if (armor_area > armor_min_area_
						&& armor_ratio > 1 // _para.small_armor_min_ratio   // 1.5
				   		&& armor_ratio < 4.5 // armor_max_aspect_ratio_   // 3.0
				   		&& abs(armor_angle) < armor_max_angle_
				   		&& armor_light_angle_diff < armor_light_angle_diff_) // 应该要更为严格
					{
						LINE_INFO("angle_1",lights[i].angle)
						LINE_INFO("angle_2",lights[j].angle)
						LINE_INFO("angle_diff",angle_diff)
						LINE_INFO("height_diff", height_diff)
						LINE_INFO("width_diff",width_diff)
						LINE_INFO("armor_ratio",armor_ratio)
						LINE_INFO("armor_angle",armor_angle)
						LINE_INFO("armor_light_angle_diff",armor_light_angle_diff)
						Armor_Twist armor_twist = STILL;
						LINE("[几乎静止] armor")
						ArmorInfo armor(possible_rect, armor_twist);
						armor_vector.push_back(armor);
					}
				}
			} // 灯条(误差) 并不同侧
			else {
				cv::RotatedRect possible_rect;
				// 2灯条不平行 中速移动
				if (1 < light_aspect_ratio1 && light_aspect_ratio1 < 1.5
					&& 1 < light_aspect_ratio2 && light_aspect_ratio2 < 1.5
					&& abs(light_aspect_ratio1 - light_aspect_ratio2) < 0.5 )
				{
					// 2灯条不平行 中速移动
					if (abs(lights[i].angle) > 60 &&  + abs(lights[j].angle) > 60)
					{
						if (lights[i].center.x < lights[j].center.x)
							possible_rect = cv_toolbox_->boundingRRectFast(lights[i], lights[j]);
						else
				    	possible_rect = cv_toolbox_->boundingRRectFast(lights[j], lights[i]);

						auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
						auto armor_angle = possible_rect.angle;
						auto armor_area = possible_rect.size.area();
						//auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

						if (armor_area > armor_min_area_
							&& armor_ratio > 1 // _para.small_armor_min_ratio   // 1.5
				   			&& armor_ratio < armor_max_aspect_ratio_   // 3.0
				   			&& abs(armor_angle) < armor_max_angle_
				   			// && armor_light_angle_diff < armor_light_angle_diff_ // 应该要更为严格
						){
							Armor_Twist armor_twist = MID_MOVE;
							LINE("[中速运动] armor")
							ArmorInfo armor(possible_rect, armor_twist);
							armor_vector.push_back(armor);
						}
					}// 2灯条不平行 中速移动
				}
				else if (1 < light_aspect_ratio1 && light_aspect_ratio1 < 2.5
						 && 1 < light_aspect_ratio2 && light_aspect_ratio2 < 2.5
						 && abs(light_aspect_ratio1 - light_aspect_ratio2) < 1)
				{
					if (abs(lights[i].angle) > 60 && abs(lights[j].angle) > 60)
					{
						if (lights[i].center.x < lights[j].center.x)
							possible_rect = cv_toolbox_->boundingRRectSlow(lights[i], lights[j]);
						else
				    	possible_rect = cv_toolbox_->boundingRRectSlow(lights[j], lights[i]);

						auto armor_ratio = possible_rect.size.width / possible_rect.size.height;
						auto armor_angle = possible_rect.angle;
						auto armor_area = possible_rect.size.area();
						auto armor_light_angle_diff = abs(armor_angle - lights[i].angle) + abs(armor_angle - lights[j].angle); // 左右灯条的积累差

						if (armor_area > armor_min_area_
							&& armor_ratio > 1 // _para.small_armor_min_ratio   // 1.5
				   			&& armor_ratio < armor_max_aspect_ratio_   // 3.0
				   			&& abs(armor_angle) < armor_max_angle_
				   			&& armor_light_angle_diff < armor_light_angle_diff_ // 应该要更为严格
						){
							Armor_Twist armor_twist = LOW_MOVE;
							LINE("[慢速运动] armor")
							ArmorInfo armor(possible_rect, armor_twist);
							armor_vector.push_back(armor);
						}
					}// 2灯条近乎平行 慢速移动
				}// 2灯条近乎平行 快速移动
			}
				
		} // for j loop
	} // for i loop

  if (enable_debug_)
  {
    for (int i = 0; i != armor_vector.size(); ++i)
      cv_toolbox_->DrawRotatedRect(show_armors_befor_filter_, armor_vector[i].rect, cv::Scalar(0,255,0), 2);
    //cv::imshow("armors_before_filter", show_armors_befor_filter_);
  }
}

void ConstraintSet::FilterArmors(std::vector<ArmorInfo> &armors) {
	std::vector<bool> is_armor(armors.size(), true);

	for (int i = 0; i < armors.size(); i++) {
	  for (int j = i + 1; j < armors.size(); j++) {
			int ojbk = cv_toolbox_->armorToarmorTest(armors[i].rect, armors[j].rect);
			if (ojbk == 1) is_armor[i] = false;
			else if(ojbk == 2) is_armor[j] = false;
		}
	}

	double dis;
#pragma omp parallel
  for (int i = 0; i < armors.size(); i++) {
#pragma omp for
    for (int j = i + 1; j < armors.size(); j++) //  && (is_armor[j] == true)
    {
      dis = POINT_DIST(armors.at(i).rect.center,armors.at(j).rect.center);

      if (dis <= std::max(armors.at(i).rect.size.width, armors.at(i).rect.size.height) + 
                std::max(armors.at(j).rect.size.width, armors.at(j).rect.size.height) )
      {
        double armor_ratio1 = std::max(armors.at(i).rect.size.width, armors.at(i).rect.size.height) / 
                              std::min(armors.at(i).rect.size.width, armors.at(i).rect.size.height); 
        double armor_ratio2 = std::max(armors.at(j).rect.size.width, armors.at(j).rect.size.height) / 
                              std::min(armors.at(j).rect.size.width, armors.at(j).rect.size.height); 

        if (is_armor[i] == true && is_armor[j] == true)
        {
          if (abs(armor_ratio1 - armor_ratio2) > 0.4)
          {
            if(armor_ratio1 - armor_ratio2 > 0)
              is_armor[i] = false;
            else
              is_armor[j] = false;
          }
          else if (abs(abs(armors.at(i).rect.angle) - abs(armors.at(j).rect.angle)) > 15) 
          {
            if (abs(armors.at(i).rect.angle) > abs(armors.at(j).rect.angle))
              is_armor[i] = false;
            else
              is_armor[j] = false;
          }
          else if (armors.at(i).rect.size.area() - armors.at(j).rect.size.area() > 100)
          {
            is_armor[i] = false;
          }
          /*else{
            float val_i = get_armor_roi(armors[i].rect);
            float val_j = get_armor_roi(armors[j].rect);

            if(val_i > 0 && val_j < 0)
            {
              is_armor[j] = false;
            }
            else if(val_i < 0 && val_j > 0)
            {
              is_armor[i] = false;
            }
            else {
              is_armor[i] = false;
              is_armor[j] = false; 
            }*/
        }// if both
      }    // dis < dist
    }	 		 // for j
  } 			 // for i

	std::vector<ArmorInfo> filter_rects;

	for( int i = 0; i < is_armor.size();++i)
		if(is_armor[i]) filter_rects.push_back(armors[i]);
	
	armors = filter_rects;

	//if (enable_debug_)
  {
    for (int i = 0; i != armors.size(); ++i)
      cv_toolbox_->DrawRotatedRect(src_img_, armors[i].rect, cv::Scalar(0,255,0), 2);
    cv::imshow("armors_after_filter", src_img_);
  }

}

ConstraintSet::~ConstraintSet() {

}



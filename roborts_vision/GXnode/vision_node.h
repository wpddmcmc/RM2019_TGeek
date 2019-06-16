/*******************************************************************************************************************
Copyright 2017 Dajiang Innovations Technology Co., Ltd (DJI)

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
documentation files(the "Software"), to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense, and / or sell copies of the Software, and 
to permit persons to whom the Software is furnished to do so, subject to the following conditions : 

The above copyright notice and this permission notice shall be included in all copies or substantial portions of
the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
IN THE SOFTWARE.
*******************************************************************************************************************/

/**
 * Robomaster Vision program of Autocar
 * Copyright (c) 2018, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */

/**
 * Robomaster Vision program of RM2019
 * Copyright (c) 2019, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */

#include <iostream>
#include <vector>
#include <string>
#include <chrono>

// ros
#include <ros/ros.h>
#include <ros/package.h>

#include "roborts_msgs/InfoFromCar.h"
#include "roborts_msgs/InfoToCar.h"

// opencv
#include "opencv2/opencv.hpp"
#include "uvc/RMVideoCapture.hpp"

// thread
#include <thread>

// vision task
#include "armor/constraint_set.h"
//#include "RuneDetector.h"

class VisionNode {

	bool show_window_;

    // thread
	std::thread producer_thread_;
	std::thread consumer_thread_;

	uint8_t robot_id_;     // 机器人id
	uint8_t cmd_mode_;     // 命令模式

	ros::NodeHandle  ros_nh_;               // handle
  	ros::Subscriber  ros_sub_car_info_;     // 订阅电控数据
	ros::Publisher   ros_pub_vison_data_;   // 发布视觉数据
	
	roborts_msgs::InfoToCar ros_info_to_car_;
	

	void ImageProducer();
	void ImageConsumer();
	void VisionTask();
	void CmdCallBack(const roborts_msgs::InfoFromCar::ConstPtr &msg);

public:
	VisionNode()
	{
		ros_nh_             = ros::NodeHandle();
		ros_sub_car_info_   = ros_nh_.subscribe("car_info", 1, &VisionNode::CmdCallBack, this);
		ros_pub_vison_data_ = ros_nh_.advertise<roborts_msgs::InfoToCar>("vision_data", 10);

		std::string file_name = ros::package::getPath("roborts_vision") + "/node/vision_param.xml";
  		cv::FileStorage fs(file_name, cv::FileStorage::READ);
  		if(!fs.isOpened())
      		ROS_ERROR ("Cannot open vision param file, please check if the file is exist");

		fs["show_window"] >> show_window_;

		producer_thread_ = std::thread(&VisionNode::ImageProducer, this);
		consumer_thread_ = std::thread(&VisionNode::ImageConsumer, this);
	}
};

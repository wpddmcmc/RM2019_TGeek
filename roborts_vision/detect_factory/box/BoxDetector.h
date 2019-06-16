#pragma once
#include <iostream>//

using namespace std;

#include <sstream>//

#include <iostream>//

#include <fstream>//
#include <algorithm>//
#include <cstring>//字符处理
#include<opencv2/imgproc/imgproc.hpp>//
#include<opencv2/core/core.hpp>//
#include<opencv2/highgui/highgui.hpp>//
using namespace cv;
#include<librealsense2/rs.hpp>
#include<librealsense2/rsutil.h>
class MyClass
{
public:
	//MyClass();
	//~MyClass(void);
public:
    Mat align_Depth2Color(Mat depth, Mat color, rs2::pipeline_profile profile);
	float get_depth();
	float d;float center_m;float miss_dis;
	float miss_left;float miss_right;
	bool check;
	// Mat depth_image;
	// Mat color_image;
	// Mat result;
private:
	float get_depth_scale(rs2::device dev);
	
	rs2::colorizer c;                          // Helper to colorize depth images
//创建数据管道
	rs2::pipeline pipe;
	rs2::config pipe_config;
};

//MyClass::MyClass()
//{
//	
//	
//}
//
//MyClass::~MyClass()
//{
//	//align_Depth2Color(Mat depth, Mat color, rs2::pipeline_profile profile);
//}
#include"align.h"
//调用视频流
//获取深度像素对应长度单位（米）的换算比例
float MyClass::get_depth_scale(rs2::device dev){
	// Go over the device's sensors
	for (rs2::sensor& sensor : dev.query_sensors()){
		// Check if the sensor if a depth sensor
		if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>()){
		    return dpt.get_depth_scale();
		}
	}
	throw std::runtime_error("Device does not have a depth sensor");
}
//深度图对齐到彩色图函数
Mat MyClass::align_Depth2Color(Mat depth, Mat color, rs2::pipeline_profile profile) {
	//声明数据流
	auto depth_stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
	auto color_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
	//获取内参
	const auto intrinDepth = depth_stream.get_intrinsics();
	const auto intrinColor = color_stream.get_intrinsics();
	//ֱ直接获取从深度摄像头坐标系到彩色摄像头坐标系的欧式变换矩阵
	//auto  extrinDepth2Color=depth_stream.get_extrinsics_to(color_stream);
	rs2_extrinsics  extrinDepth2Color;
	rs2_error *error;
	rs2_get_extrinsics(depth_stream, color_stream, &extrinDepth2Color, &error);
	//平面点定义
	float pd_uv[2], pc_uv[2];
	//空间点定义
	float Pdc3[3], Pcc3[3];
	//获取深度像素与现实单位比例（默认1mm）
	float depth_scale = get_depth_scale(profile.get_device());
	//    uint16_t depth_max=0;
	//    for(int row=0;row<depth.rows;row++){
	//        for(int col=0;col<depth.cols;col++){
	//            if(depth_max<depth.at<uint16_t>(row,col))
	//                depth_max=depth.at<uint16_t>(row,col);
	//        }
	//    }
	int y = 0, x = 0;
	//初始化结果
	Mat result = Mat::zeros(color.rows, color.cols, CV_8UC3);
    uint16_t center_value= depth.at<uint16_t>(depth.rows/2, depth.cols/2);
	center_m = center_value * depth_scale;//中心点距离换算
	d=center_m;
	check=false;
	miss_dis=depth.cols/2;//中偏
	miss_left=depth.cols/2;//左边界
	miss_right=depth.cols/2;//右边界
	//对深度图像遍历
	for (int row = 0; row < depth.rows/2; row++) {
		for (int col = 0; col < depth.cols; col++) {
			//将当前的（x,y）放入数组pd_uv，表示当前深度图的点
			pd_uv[0]=col;
			pd_uv[1]=row;
			//取当前点对应的深度值
			uint16_t depth_value = depth.at<uint16_t>(row, col);
			//换算到米
			float depth_m = depth_value * depth_scale;
			//将深度图的像素点根据内参转换到深度摄像头坐标系下的三维点
			rs2_deproject_pixel_to_point(Pdc3, &intrinDepth, pd_uv, depth_m);
			//将深度摄像头坐标系的三维点转化到彩色摄像头坐标系下
			rs2_transform_point_to_point(Pcc3, &extrinDepth2Color, Pdc3);
			//将彩色摄像头坐标系下的深度三维点映射到二维平面上
			rs2_project_point_to_pixel(pc_uv, &intrinColor, Pcc3);
			//获得映射后的（u,v）
			x = (int)pc_uv[0];
			y = (int)pc_uv[1];
			//            if(x<0||x>color.cols)
			//                continue;
			//            if(y<0||y>color.rows)
			//                continue;
			//最值限定
			x = x < 0 ? 0 : x;
			x = x > depth.cols - 1 ? depth.cols - 1 : x;
			y = y < 0 ? 0 : y;
			y = y > depth.rows - 1 ? depth.rows - 1 : y;
			//将成功映射的点用彩色图对应点的RGB数据覆盖
			for (int k = 0; k < 3; k++) {
			//只显示0.45米距离内的东西
			if (depth_m!=0&&depth_m<0.45){
			    result.at<cv::Vec3b>(y, x)[k]=color.at<cv::Vec3b>(y, x)[k];
			    d=(depth_m+d)/2;
			    miss_dis=(depth.cols/2-col+miss_dis)/2;
				check=true;
			    }
			}
	    }
	    
	
	}
    return result;
}
float MyClass::get_depth(){
	float depth=d;
	return depth;

}
#include<opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>  
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>  
#include <vector>
#include <iostream>//标准输入输出库
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <algorithm> 
#include "align.h"
#include <LinuxSerial.hpp>
#include <thread>
#include <unistd.h>
#include <mutex>
using namespace std;
using namespace cv;
std::mutex mtx;
float oriX;float oriY;//中心点
float dis;float miss;
int center_idx = 0; int center_idy = 0;int cir_r;
Mat gray; Mat frame; Mat edge; Mat result;
IplImage gray2 = IplImage(gray); CvArr* arr = (CvArr*)&gray2;
bool check;

CLinuxSerial serial(0);

void receive(void);
void send();

void send(void){
	while(1){
        mtx.lock();
	    int length = 8;
     	unsigned char *temp = new unsigned char[8];//动态创建一个数组
	    unsigned char ef[8] = { 0xAD,0,0,0,0,0,0,0xAE };
	    if (check)
	    {
	    	ef[1] = check;
	    	ef[2] = dis;
	    	ef[3] = miss;
	    }	
	    else
	    {
	    	ef[2] = 0;
	    	ef[3] = 0;
	    }
	    cout << serial.WriteData(ef, 8) << endl;	
	    mtx.unlock();
    }
}
void receive(void) { 
	MyClass Myalign;
	const char* depth_win = "depth_Image";
	namedWindow(depth_win, WINDOW_AUTOSIZE);
	const char* color_win = "color_Image";
	namedWindow(color_win, WINDOW_AUTOSIZE);
    //深度图像颜色map
	rs2::colorizer c;                          // Helper to colorize depth images
    rs2::pipeline pipe;                        //创建数据管道
    rs2::config pipe_config;
	pipe_config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    pipe_config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    //start()函数返回数据管道的profile
    rs2::pipeline_profile profile = pipe.start(pipe_config);
	//定义一个变量去转换深度到距离
	float depth_clipping_distance = 1.f;

	// Application still alive? 
	while (cvGetWindowHandle(depth_win) && cvGetWindowHandle(color_win)){ 
	    //堵塞程序直到新的一帧捕获
	    rs2::frameset frameset = pipe.wait_for_frames();
		//取深度图和彩色图
		rs2::frame color_frame = frameset.get_color_frame();//processed.first(align_to);
		rs2::frame depth_frame = frameset.get_depth_frame();
	    //rs2::frame depth_frame_4_show = frameset.get_depth_frame().apply_filter(c);
	    //获取宽高
		const int depth_w=640;
		const int depth_h=480;
		const int color_w=640;
		const int color_h=480;
		//创建OPENCV类型 并传入数据
		Mat depth_image(Size(depth_w, depth_h),CV_16U, (void*)depth_frame.get_data(), Mat::AUTO_STEP);
	    Mat color_image(Size(color_w, color_h),CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
		//实现深度图对齐到彩色图
	    result = Myalign.align_Depth2Color(depth_image, color_image, profile);
		oriX = result.cols / 2;
		oriY = result.rows / 2;
		circle(result, Point2f(oriX, oriY), 2, Scalar(0, 0, 255), 3);
		check=Myalign.check;//物体确认
		dis=Myalign.d;
		miss=Myalign.miss_dis;
		Point a(50, 50);
		Point b(50,100);
		String distance = "d:" + format("%f",Myalign.d);
		String m = "m:" + format("%f",Myalign.miss_dis);
		putText(result, distance, a, FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 255), 1, 8);
		putText(result, m, b, FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 255), 1, 8);
		imshow("result",result);
	    imshow(depth_win,depth_image*15);
	    imshow(color_win, color_image);
	    waitKey(1);
	} 
	
}
int main(int argc, char* argv[]){
	
	std::thread temp1(send);
    temp1.detach();	

    receive();
    //std::thread temp2(contour,result);
    //temp2.detach();
	
    return 0;
}

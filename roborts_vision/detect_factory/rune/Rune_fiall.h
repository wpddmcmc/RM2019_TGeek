#ifndef RUNE_NEW_H
#define RUNE_NEW_H


#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

Mat src,src_1,src_2;
Mat binary_color_img;


std::vector<sdt::vector<Point>> contours_color;

std::vector<cv::Point2f> center_point;

cv::Point2f his_cen_point;
cv::Point2f predict_point;
cv::Point2f his;

float w0;
float L_R;
int color;
int colorwise;

vector<Point2f> RUNE();

Point2f get_center_point(vector<Point> cen_r)
{
    Moments mu = moments(cen_r, false);	
	Point2f mc = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00); 
    return mc;
}

float distance_hanshu(Point2f points,Point2f cen_R)
{
    float distances = sqrt((points.x-cen_R.x)*(points.x-cen_R.x) + (points.y-cen_R.y)*(points.y-cen_R.y));
    return distances;
}

void video_get(string video_dir_, int cols_, int rows_,Mat & src, VideoCapture cap)
{
    int cols = cols_;
    int rows = rows_;
    char filename[128];
    time_t rawtime;
    struct tm *timeinfo;
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    sprintf(filename, "%s/armor_%04d%02d%02d_%02d%02d%02d.avi", video_dir_.c_str(),
            timeinfo->tm_year+1900, timeinfo->tm_mon+1, timeinfo->tm_mday,
            timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
    VideoWriter writer(filename, CV_FOURCC('M', 'J', 'P', 'G'), 60.0, cv::Size(cols, rows));
    while(cap.isOpened())
    {
        if((src.rows ==0 )||(src.cols == 0))
        printf("failed!");
        writer << src;
        exit(0);
    }
}


void gammaProcessImage(Mat& oriMat,double gamma,Mat &outputMat){
   
    //伽马方法也是按照一个公式修改了每个像素值，我们可以通过LUT函数进行编写，它的公式是：
    //O=(I/255)的γ次方×255
    //代码如下
    Mat lookupTable(1,256,CV_8U);
    uchar* p = lookupTable.ptr();
    for (int i =0 ; i < 256; i++) {
        p[i] = saturate_cast<uchar>(pow(i/255.0, gamma) * 255.0);
    }
    LUT(oriMat,lookupTable,outputMat);
}

#endif
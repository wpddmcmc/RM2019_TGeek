#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "Rune_fiall.h"
using namespace cv;
using namespace std;

#define USE_VIDEO
#define DEBUG
VideoCapture cap; 

vector<Point2f> RUNE()
{
    color = 0;//0 for red, 1 for blue;
    int Maxarea = 1500;
    int Minarea = 1000;
#ifdef USE_VIDEO 
    if(color == 0)
        cap.open("12.mp4");
    else
        cap.open("b1.mp4");
    cout<<"successful load image"<<endl;
#else
    cap.open("/dev/video1");
#endif

#ifndef USE_VIDEO
    string video_dir_ = "../video";
    char filename[128];
    time_t rawtime;
    struct tm *timeinfo;
    time(&rawtime);
    timeinfo = localtime(&rawtime);
    sprintf(filename, "%s/armor_%04d%02d%02d_%02d%02d%02d.avi", video_dir_.c_str(),
            timeinfo->tm_year+1900, timeinfo->tm_mon+1, timeinfo->tm_mday,
            timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
    VideoWriter writer(filename, CV_FOURCC('M', 'J', 'P', 'G'), 60.0, cv::Size(640, 480));
#endif
    while(true)
    {
        double start = static_cast<double>(getTickCount());
        cap >> src_1;
#ifndef USE_VIDEO
        //writer << src;
#endif
        if(src_1.empty())
        {
            cout<<"no video"<<endl;
            break;
        }
        //Rect rect_1(250,0,640,480);//200,0
        //src_2 = Mat(src_1,rect_1);
        gammaProcessImage(src_1,1.5,src);//提高对比度
	    vector<Mat> bgr_channel;
	    split(src, bgr_channel);
	    if (color == 0){
            binary_color_img =bgr_channel.at(2)-bgr_channel.at(0);
            threshold(binary_color_img,binary_color_img, 50, 255, CV_THRESH_BINARY);//130
        }
	    else{
            binary_color_img =bgr_channel.at(0)-bgr_channel.at(2);
            threshold(binary_color_img,binary_color_img, 150, 255, CV_THRESH_BINARY);//130
        }
        Mat binary;
        binary_color_img.copyTo(binary);
        GaussianBlur(binary,binary,Size(3,3),0,0);

        floodFill(binary_color_img,Point(0,0),Scalar(255));
        threshold(binary_color_img,binary_color_img,0,255,THRESH_BINARY_INV);
        Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
        morphologyEx(binary_color_img, binary_color_img, MORPH_OPEN, element);
        findContours(binary_color_img,contours_color, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

        vector<vector<Point> > contours_color_l;
        vector<vector<Point> > contours_color_finall;
        findContours(binary,contours_color_l, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
        
        for(int i=0;i<contours_color_l.size();i++)
        {
            double Cont = fabs(contourArea(contours_color_l[i],true));
            if(Cont < 5000 && Cont > 3500){
                contours_color_finall.push_back(contours_color_l[i]);
                break;
            }
        }
        if(contours_color_finall.empty()){
            continue;
        }
        vector<vector<Point> >contours_ploy(contours_color.size());
        vector<RotatedRect> RotatedRect_ploy;
        vector<vector<Point> >contours_r;
        RotatedRect predict_rect;
        Point2f predict_rect_points[4];
        for(int i=0;i<contours_color.size();i++)
        {
            double Cont = fabs(contourArea(contours_color[i],true));
            approxPolyDP(contours_color[i], contours_ploy[i], 5, true);
		    RotatedRect temp1 = minAreaRect(contours_ploy[i]);
            float min,max;
            if(temp1.size.width > temp1.size.height){
                min = temp1.size.height;
                max = temp1.size.width;
            }
            else{
                max = temp1.size.height;
                min = temp1.size.width;
            }
            if(max/min >1.5 && max/min< 2.2 && Cont <Maxarea && Cont >Minarea){
                RotatedRect_ploy.push_back(temp1);
            }
            if(Cont < 70 && Cont >30){
                contours_r.push_back(contours_color[i]);
            }
        }
        Point2f pot_cen,cen_R;
        Point2f pot[4];
        for (int i = 0; i< RotatedRect_ploy.size(); i++)
		{
			Scalar color = Scalar(0,0,255);
			RotatedRect_ploy[i].points(pot);
            if(pointPolygonTest(contours_color_finall[0],RotatedRect_ploy[i].center,false) == 1)
            {
                predict_rect = RotatedRect_ploy[i];
                for(int j=0; j<4; j++)
		        {
                    pot_cen = RotatedRect_ploy[i].center;
			        line(src, pot[j], pot[(j+1)%4], color,2);
		        }
                break;
            }
		}
//圆的方程预测
        if(contours_r.empty())
        {
            cen_R =his_cen_point;
        }
        else        
        {
            cen_R = get_center_point(contours_r[0]);
            his_cen_point = cen_R;
        }
        float L_R = distance_hanshu(pot_cen,cen_R); 
        w0 = asin((pot_cen.x-cen_R.x)/L_R);
        float w1 = 0.0349*3;
        if((pot_cen.x-his.x)>0 &&(pot_cen.y-his.y)<0)//判断顺时针还是逆时针
            colorwise = 1;
        else
            colorwise = 0;
        if(colorwise == 0)//顺时
        {
            if(pot_cen.y < cen_R.y){
                predict_point.x = sin(w0+w1)*L_R+cen_R.x;
                predict_point.y = -cos(w0+w1)*L_R+cen_R.y;
            }
            else{
                predict_point.x = sin(w0-w1)*L_R+cen_R.x;
                predict_point.y = cos(w0-w1)*L_R+cen_R.y;
            }
        }
        else//逆时针
        {
            if(pot_cen.y < cen_R.y){
                predict_point.x = sin(w0-w1)*L_R+cen_R.x;
                predict_point.y = -cos(w0-w1)*L_R+cen_R.y;
            }
            else{
                predict_point.x = sin(w0+w1)*L_R+cen_R.x;
                predict_point.y = cos(w0+w1)*L_R+cen_R.y;
            }
        }
        his = pot_cen;
        for(int i = 0;i<4;i++)
        {
            float cha_x = predict_point.x - pot_cen.x;
            float cha_y = predict_point.y - pot_cen.y;
            predict_rect_points[i].x = pot[i].x + cha_x;
            predict_rect_points[i].y = pot[i].y + cha_y;
        }
        for(int j=0;j<4;j++)
        {
            line(src, predict_rect_points[j], predict_rect_points[(j+1)%4], Scalar(0,255,0),2);
        }
//**********************
        imshow("img",src);
        double time = ((double)getTickCount() - start) / getTickFrequency();
        cout << time << "秒" << endl;

        contours_r.clear();
        contours_color.clear();
        contours_color_l.clear();
        contours_color_finall.clear();
        char key = (char)waitKey(0);
        if (key == 27)
           break;

        vector<Point2f> result;
        for(int i=0;i<4;i++)
        {
            result.push_back(predict_rect_points[i]);
        } 
        return result;
    }      
}


/*
 *@brief: 实现双目测距, 西电Robomaster视觉组
 */
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utility.hpp>
#include "Stereo.hpp"

using namespace std;
using namespace cv;

//#define VIDEO

static void saveXYZ(const char* filename, const Mat& mat);

int no_display=0;
//std::string point_cloud_named;
#ifndef VIDEO
int main() 
{
	Stereo stereo;
	stereo.initCamera(); // 用std::string加载xml
	stereo.initStereoBM();
	stereo.initStereoSGBM();

	Mat frame_L, frame_R, img1r, img2r;
	// 打开摄像头

	frame_L = imread("left/I1_000002.png");
	frame_R = imread("right/I2_000002.png");
	Mat rmap00 = stereo.getrmap(0,0);
	Mat rmap01 = stereo.getrmap(0,1);
	Mat rmap10 = stereo.getrmap(1,0);
	Mat rmap11 = stereo.getrmap(1,1);

		Mat img(VIDEO_HEIGHT, VIDEO_WIDTH * 2, CV_8UC3); // 高度一样，宽度双倍
		
		// 用remap来校准输入的左右图像
		remap(frame_L, img1r, rmap00, rmap01, CV_INTER_AREA);//左校正
		remap(frame_R, img2r, rmap10, rmap11, CV_INTER_AREA);//右校正
		// 其中remap的图像剪裁系数alpha，取值范围是-1、0~1。当取值为 0 时，OpenCV会对校正后的图像进行缩放和平移，
		// 使得remap图像只显示有效像素（即去除不规则的边角区域），适用于机器人避障导航等应用；
		// 当alpha取值为1时，remap图像将显示所有原图像中包含的像素，该取值适用于畸变系数极少的高端摄像头；
		// alpha取值在0-1之间时，OpenCV按对应比例保留原图像的边角区域像素。Alpha取值为-1时，OpenCV自动进行缩放和平移。 
		
		Mat imgPart1 = img(Rect(0, 0, VIDEO_WIDTH, VIDEO_HEIGHT));          // 浅拷贝
		Mat imgPart2 = img(Rect(VIDEO_WIDTH, 0, VIDEO_WIDTH, VIDEO_HEIGHT));// 浅拷贝
		resize(img1r, imgPart1, imgPart1.size(), 0, 0, CV_INTER_AREA);
		resize(img2r, imgPart2, imgPart2.size(), 0, 0, CV_INTER_AREA);

		//画横线
		for (int i = 0; i < img.rows; i += 32)
			line(img, Point(0, i), Point(img.cols, i), Scalar(0, 255, 0), 1, 8);

		//imshow("L", frame_L);
		//imshow("R", frame_R);
		imshow("rectified", img);  // 第一次输出


		// step4.立体匹配 such as StereoBM
		cvtColor(img1r,img1r,CV_RGB2GRAY);
		cvtColor(img2r,img2r,CV_RGB2GRAY);

		Mat disp,disp8;
		int64 t = getTickCount();
		//stereo.bm->compute(img1r, img2r, stereo.disp);
		stereo.sgbm->compute(img1r, img2r, stereo.disp);
		stereo.disp.convertTo(stereo.disp8, CV_8U, 255/(16*5*16.));
		//disp.convertTo(disp8, CV_8U);
		t = getTickCount() - t; 
    	std::cout << "Time elapsed: "<< t*1000/getTickFrequency() << "ms \n";
		
    	// !再加一个判断图像是否为空!

		if( !no_display )
    	{
        	imshow("disparity", stereo.disp8);
   		}
		waitKey(0);
	


	return 0;
}
#else
int main() 
{
	Stereo stereo;
	stereo.initCamera(); // 用std::string加载xml
	stereo.initStereoBM();
	stereo.initStereoSGBM();

	Mat frame_L, frame_R, img1r, img2r;
	// 打开摄像头
	VideoCapture cap_left, cap_right;

	cap_left.open(0);
	cap_right.open(1);

	if (!cap_left.isOpened()) return -1;
	if (!cap_right.isOpened()) return -1;

	cap_left.set(CV_CAP_PROP_FRAME_WIDTH, VIDEO_WIDTH);
	cap_left.set(CV_CAP_PROP_FRAME_HEIGHT, VIDEO_HEIGHT);

	cap_right.set(CV_CAP_PROP_FRAME_WIDTH, VIDEO_WIDTH);
	cap_right.set(CV_CAP_PROP_FRAME_HEIGHT, VIDEO_HEIGHT);

	Mat rmap00 = stereo.getrmap(0,0);
	Mat rmap01 = stereo.getrmap(0,1);
	Mat rmap10 = stereo.getrmap(1,0);
	Mat rmap11 = stereo.getrmap(1,1);

	while (true)
	{
		cap_left >> frame_L;
		cap_right >> frame_R;

		Mat img(VIDEO_HEIGHT, VIDEO_WIDTH * 2, CV_8UC3); // 高度一样，宽度双倍
		
		// 用remap来校准输入的左右图像
		remap(frame_L, img1r, rmap00, rmap01, CV_INTER_AREA);//左校正
		remap(frame_R, img2r, rmap10, rmap11, CV_INTER_AREA);//右校正
		// 其中remap的图像剪裁系数alpha，取值范围是-1、0~1。当取值为 0 时，OpenCV会对校正后的图像进行缩放和平移，
		// 使得remap图像只显示有效像素（即去除不规则的边角区域），适用于机器人避障导航等应用；
		// 当alpha取值为1时，remap图像将显示所有原图像中包含的像素，该取值适用于畸变系数极少的高端摄像头；
		// alpha取值在0-1之间时，OpenCV按对应比例保留原图像的边角区域像素。Alpha取值为-1时，OpenCV自动进行缩放和平移。 
		
		Mat imgPart1 = img(Rect(0, 0, VIDEO_WIDTH, VIDEO_HEIGHT));          // 浅拷贝
		Mat imgPart2 = img(Rect(VIDEO_WIDTH, 0, VIDEO_WIDTH, VIDEO_HEIGHT));// 浅拷贝
		resize(img1r, imgPart1, imgPart1.size(), 0, 0, CV_INTER_AREA);
		resize(img2r, imgPart2, imgPart2.size(), 0, 0, CV_INTER_AREA);

		//画横线
		for (int i = 0; i < img.rows; i += 32)
			line(img, Point(0, i), Point(img.cols, i), Scalar(0, 255, 0), 1, 8);

		//imshow("L", frame_L);
		//imshow("R", frame_R);
		imshow("rectified", img);  // 第一次输出


		// step4.立体匹配 such as StereoBM
		cvtColor(img1r,img1r,CV_RGB2GRAY);
		cvtColor(img2r,img2r,CV_RGB2GRAY);

		Mat disp,disp8;
		int64 t = getTickCount();
		stereo.bm->compute(img1r, img2r, stereo.disp);
		//stereo.sgbm->compute(img1r, img2r, stereo.disp);
		stereo.disp.convertTo(stereo.disp8, CV_8U, 255/(16*5*16.));
		//disp.convertTo(disp8, CV_8U);
		t = getTickCount() - t; 
    	std::cout << "Time elapsed: "<< t*1000/getTickFrequency() << "ms \n";
		
    	// !再加一个判断图像是否为空!

		if( !no_display )
    	{
        	imshow("disparity", stereo.disp8);
   		}
		waitKey(15);
	}


	return 0;
}
#endif

static void saveXYZ(const char* filename, const Mat& mat)
{
    const double max_z = 1.0e4;
    FILE* fp = fopen(filename, "wt");
    for(int y = 0; y < mat.rows; y++)
    {
        for(int x = 0; x < mat.cols; x++)
        {
            Vec3f point = mat.at<Vec3f>(y, x);
            if(fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) continue;
            fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);
        }
    }
    fclose(fp);
}

		/*
    	if(!point_cloud_filename.empty())
   		{
        	printf("storing the point cloud...");
        	fflush(stdout);
        	Mat xyz;
			// 将视差矩阵转换成实际的物理坐标矩阵。
			// 在实际求距离时，[X/W, Y/W, Z/W] 都要乘以16(也就是W÷16)，才能得到正确的三维坐标信息
        	reprojectImageTo3D(stereo.disp, xyz, stereo.Q, true);
        	saveXYZ(point_cloud_filename.c_str(), xyz);
        	printf("\n");
   	 	}
		*/

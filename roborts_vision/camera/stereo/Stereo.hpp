#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/core/cuda.hpp>


using namespace cv;

#define USE_VIDEO

// 设置输入图像的分辨率
#define VIDEO_WIDTH  1344    // 视频宽
#define VIDEO_HEIGHT 391    // 视频高
#define BUFFER_SIZE 1       // 缓存一张图片

class Stereo
{
	int checkstereoMatchParam();

	// step0.加载所有的xml
	Mat cam_matrix_L, distortion_Coeff_L;
	Mat cam_matrix_R, distortion_Coeff_R;
	Mat RotMatrix, Translation;

	// step1.计算旋转矩阵和投影矩阵 [立体矫正]
	Mat R1;           // 左摄像机旋转矩阵
	Mat R2;           // 右摄像机旋转矩阵
	Mat P1;           // 左摄像机投影矩阵
	Mat P2;           // 右摄像机投影矩阵 
	Mat Q;            // 重投影矩阵
	Rect rect_roi1, rect_roi2;
	Size img_size;

	// step2.计算校正查找映射表 [矫正映射]
	Mat rmap[2][2];

public:
	// StereoMatch参数设置只需要执行一步
	enum { STEREO_BM = 0, STEREO_SGBM = 1, STEREO_HH = 2, STEREO_VAR = 3, STEREO_3WAY = 4 };

    int alg;                  /*选择算法*/
    int SADWindowSize;        /*=blocksize*/
	int numberOfDisparities;  /*=max-disparity*/

    float scale;    // 目前 no-use

public:
	Ptr<StereoBM> bm;
	Ptr<StereoSGBM> sgbm;
	Mat disp, disp8; // 定义深度图

public:

	int initCamera();
	
	Mat getrmap(int i,int j)
	{
		if (0 <= i && i <= 1 && 0 <= j && j <= 1)
			return rmap[i][j];
		else{
			std::cout << "Param Error: ramp[][] out of range " << std::endl;
			return rmap[0][0];
		}
	}
	/*
 	*@brief: block matching algorithm, introduced and contributed to OpenCV by K. Konolige. 
		使用的是SAD方法，速度比较快，但效果一般
	
		- MinDisparity 代表了匹配搜苏从哪里开始，设置为0，因为两个摄像头是前向平行放置，相同的物体在左图中一定比在右图中偏右。如果为了追求更大的双目重合区域而将两个摄像头向内偏转的话，这个参数是需要考虑的。
   		- UniquenessRatio 主要可以防止误匹配，此参数对于最后的匹配结果是有很大的影响。立体匹配中，宁愿区域无法匹配，也不要误匹配。如果有误匹配的话，碰到障碍检测这种应用，就会很麻烦。该参数不能为负值，一般5-15左右的值比较合适，int型。 
    	- BlockSize SAD窗口大小，容许范围是[5,255]，一般应该在 5x5..21x21 之间，参数必须为奇数值, int型。 
    	- NumDisparities 视差窗口，即最大视差值与最小视差值之差,窗口大小必须是 16的整数倍，int型。 
		- numberOfDisparities 表示最大搜索视差数
		- uniquenessRatio表示匹配功能函数

    	在BM算法的参数中，对视差生成效果影响较大的主要参数是BlockSize、NumDisparities和UniquenessRatio三个，一般只需对这三个参数进行调整，其余参数按默认设置即可。 
 	*/
	int initStereoBM();
	int initStereoSGBM();	// sgbm算法
	Mat StereoMatch_dist(Mat img1, Mat img2);
};
	


#include "Stereo.hpp"

using namespace cv;


int Stereo:: initCamera() // 前面的四步只需要加载一次
{
    img_size = Size(VIDEO_WIDTH, VIDEO_HEIGHT);
	// step0.加载所有的xml
	FileStorage fs("cam_intrinsic.xml", FileStorage::READ);
	if (!fs.isOpened()) {
		std::cout << "Could not open the configuration file: cam_intrinsic.xml " << std::endl;
		return -1;
	}
	fs["Camera_Matrix_L"] >> cam_matrix_L;
	fs["Camera_Matrix_R"] >> cam_matrix_R;
	fs["Distortion_Coefficients_L"] >> distortion_Coeff_L;
	fs["Distortion_Coefficients_R"] >> distortion_Coeff_R;
	fs["RotMatrix"] >> RotMatrix;
	fs["Translation"] >> Translation;

	// step1.计算旋转矩阵和投影矩阵 [立体矫正]
	stereoRectify(cam_matrix_L, distortion_Coeff_L, cam_matrix_R, distortion_Coeff_R, 
				  img_size, RotMatrix, Translation, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &rect_roi1, &rect_roi2); // Q 之后的参数可选

	// step2.计算校正查找映射表 [矫正映射]
	initUndistortRectifyMap(cam_matrix_L, distortion_Coeff_L, R1, P1, img_size, CV_16SC2, rmap[0][0], rmap[0][1]);  // CV_16SC2可选的
	initUndistortRectifyMap(cam_matrix_R, distortion_Coeff_R, R2, P2, img_size, CV_16SC2, rmap[1][0], rmap[1][1]);
	return 0;
}

int Stereo:: checkstereoMatchParam()
{
	if (alg < 0) {
    	std::cout << "Parameter error: : Unknown stereo algorithm\n";
   	 	return -1;
	}
    if (SADWindowSize < 1 || SADWindowSize % 2 != 1) {
    	std::cout << "Command-line parameter error: The block size must be a positive odd number\n";
    	return -1;
	}
	if (numberOfDisparities < 1 || numberOfDisparities % 16 != 0) {
    	std::cout << "Parameter error: The max disparity must be a positive integer divisible by 16\n";
    	return -1;
	}
	if (scale < 0){
    	std::cout << "Parameter error: The scale factor must be a positive floating-point number\n";
    	return -1;
	}
}
	
int Stereo:: initStereoBM() 
{
    alg = STEREO_SGBM;
	SADWindowSize = 19;		// SADWindowSize：SAD窗口大小，容许范围是[5,255]，一般应该在 5x5 至 21x21 之间，参数必须是奇数，int 型
   	numberOfDisparities =16*5; /**< Range of disparity */

	checkstereoMatchParam();

	// int color_mode = alg == STEREO_BM ? 0 : -1; // 不同算法要求不同,有的必须要求是灰度图像
	numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width/8) + 15) & -16;

	bm = StereoBM::create(16,9); // 该算法有CUDA版本

	// 左右视图的有效像素区域，一般由双目校正阶段的 cvStereoRectify 函数传递，也可以自行设定。
	// 一旦在状态参数中设定了 roi1 和 roi2，OpenCV 会通过cvGetValidDisparityROI 函数计算出视差图的有效区域，在有效区域外的视差值将被清零。
	bm->setROI1(rect_roi1); 
   	bm->setROI2(rect_roi2); 

	// 预处理滤波参数
	// preFilterType：
	// 			主要是用于降低亮度失真（photometric distortions）、消除噪声和增强纹理等,
	// 			有两种可选类型：CV_STEREO_BM_NORMALIZED_RESPONSE（归一化响应） 或者 CV_STEREO_BM_XSOBEL（水平方向Sobel算子，默认类型）, 该参数为 int 型；
    // preFilterSize：
	// 			预处理滤波器窗口大小，容许范围是[5,255]，一般应该在 5x5..21x21 之间，参数必须为奇数值, int 型
	// 			bm.State->preFilterSize=41;//预处理滤波器窗口大小,5-21,odd
	bm->setPreFilterCap(31); 					 // PreFilterCap: 预处理滤波器的截断值，预处理的输出值仅保留[-preFilterCap, preFilterCap]范围内的值，参数范围：1 - 31（文档中是31，但代码中是 63）, int

	// SAD 参数
	bm->setBlockSize(SADWindowSize > 0 ? SADWindowSize : 9); // SAD窗口大小5-21
   	bm->setMinDisparity(0);  					 // minDisparity：最小视差，默认值为 0, 可以是负值, int 型. 因为两个摄像头是前向平行放置，相同的物体在左图中一定比在右图中偏右。如果为了追求更大的双目重合区域而将两个摄像头向内偏转的话，这个参数是需要考虑的。
	bm->setNumDisparities(numberOfDisparities);  // numberOfDisparities：视差窗口，即最大视差值与最小视差值之差, 窗口大小必须是 16 的整数倍，int 型

	// 后处理参数
	bm->setTextureThreshold(10);  				 // textureThreshold：低纹理区域的判断阈值。如果当前SAD窗口内所有邻居像素点的x导数绝对值之和小于指定阈值，则该窗口对应的像素点的视差值为 0
	bm->setUniquenessRatio(15);  				 // uniquenessRatio：视差唯一性百分比， 视差窗口范围内最低代价是次低代价的(1 + uniquenessRatio/100)倍时，最低代价对应的视差值才是该像素点的视差，否则该像素点的视差为 0,范围5-15,int
	bm->setSpeckleWindowSize(100);				 // speckleWindowSize：检查视差连通区域变化度的窗口大小, 值为 0 时取消 speckle 检查，int 型
	bm->setSpeckleRange(32);  					 // speckleRange：视差变化阈值，当窗口内视差变化大于阈值时，该窗口内的视差清零，int 型
   	bm->setDisp12MaxDiff(1);  					 // disp12MaxDiff：左视差图（直接计算得出）和右视差图（通过cvValidateDisparity计算得出）之间的最大容许差异。超过该阈值的视差值将被清零。该参数默认为 -1，即不执行左右视差检查。
       	                                         // 注意在程序调试阶段最好保持该值为 -1，以便查看不同视差窗口生成的视差效果。
}

int Stereo:: initStereoSGBM()	// sgbm算法
{
	// in BM 
	int SADWindowSize = 19;		// SADWindowSize：SAD窗口大小，容许范围是[5,255]，一般应该在 5x5 至 21x21 之间，参数必须是奇数，int 型
	int numberOfDisparities =16*3; /**< Range of disparity */
	// in BM
	checkstereoMatchParam();
	int color_mode = alg == STEREO_BM ? 0 : -1; // 不同算法要求不同,有的必须要求是灰度图像
	numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width/8) + 15) & -16;
	
	sgbm = StereoSGBM::create(0,16,3);

	// SGBM算法的状态参数大部分与BM算法的一致，下面只解释不同的部分：

	// SADWindowSize：SAD窗口大小，容许范围是[1,11]，一般应该在 3x3 至 11x11 之间，参数必须是奇数，int 型
   	// P1, P2：控制视差变化平滑性的参数。P1、P2的值越大，视差越平滑。P1是相邻像素点视差增/减 1 时的惩罚系数；P2是相邻像素点视差变化值大于1时的惩罚系数。P2必须大于P1。OpenCV2.1提供的例程 stereo_match.cpp 给出了 P1 和 P2 比较合适的数值。
   	// fullDP：布尔值，当设置为 TRUE 时，运行双通道动态编程算法（full-scale 2-pass dynamic programming algorithm），会占用O(W*H*numDisparities)个字节，对于高分辨率图像将占用较大的内存空间。一般设置为 FALSE。

 	// 算法默认运行单通道DP算法，只用了5个方向，而fullDP使能时则使用8个方向（可能需要占用大量内存）。
   	// 算法在计算匹配代价函数时，采用块匹配方法而非像素匹配（不过SADWindowSize=1时就等于像素匹配了）。
   	// 匹配代价的计算采用BT算法（"Depth Discontinuities by Pixel-to-Pixel Stereo" by S. Birchfield and C. Tomasi），并没有实现基于互熵信息的匹配代价计算。
   	// 增加了一些BM算法中的预处理和后处理程序。
   	sgbm->setPreFilterCap(63);
   	int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
   	sgbm->setBlockSize(sgbmWinSize);

   	int cn = 1; //img1.channels();

   	sgbm->setP1(8*cn*sgbmWinSize*sgbmWinSize);
   	sgbm->setP2(32*cn*sgbmWinSize*sgbmWinSize);
   	sgbm->setMinDisparity(0);
   	sgbm->setNumDisparities(numberOfDisparities);
   	sgbm->setUniquenessRatio(10);
   	sgbm->setSpeckleWindowSize(100);
   	sgbm->setSpeckleRange(32);
   	sgbm->setDisp12MaxDiff(1);
   	if(alg==STEREO_HH)
   		sgbm->setMode(StereoSGBM::MODE_HH);
   	else if(alg==STEREO_SGBM)
   		sgbm->setMode(StereoSGBM::MODE_SGBM);
   	else if(alg==STEREO_3WAY)
   		sgbm->setMode(StereoSGBM::MODE_SGBM_3WAY);

	//（3）StereoGCState
	// GC算法的状态参数只有两个：numberOfDisparities 和 maxIters ，并且只能通过 cvCreateStereoGCState 
	// 在创建算法状态结构体时一次性确定，不能在循环中更新状态信息。GC算法并不是一种实时算法，但可以得到物体轮廓清晰准确的视差图，适用于静态环境物体的深度重构。
	// 注意GC算法只能在C语言模式下运行，并且不能对视差图进行预先的边界延拓，左右视图和左右视差矩阵的大小必须一致。
}

Mat Stereo::StereoMatch_dist(Mat img1, Mat img2)  // imgr
{
	// 计算视差
	
   	if( alg == STEREO_BM )
	{
   		bm->compute(img1, img2, disp);
		//-- Check its extreme values
       	// cv::minMaxLoc( disp, &minVal, &maxVal );
       	// cout<<"Min disp: Max value"<< minVal<<maxVal; //numberOfDisparities.= (maxVal - minVal)
	}
    else if( alg == STEREO_SGBM || alg == STEREO_HH || alg == STEREO_3WAY )
   		sgbm->compute(img1, img2, disp);

	// 记录算法运行时间
    

    // 对深度进行转换
	// BM算法计算出的视差disp是CV_16S格式，通过disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.))变换才能得到真实的视差值。
    if( alg != STEREO_VAR )
	{
       	disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));
		//-- 4. Display it as a CV_8UC1 image
    	// disp.convertTo(disp8u, CV_8U, 255/(maxVal - minVal));//(numberOfDisparities*16.)
    	// cv::normalize(disp8u, disp8u, 0, 255, CV_MINMAX, CV_8UC1);    // obtain normalized image
	}  
   	else
       	disp.convertTo(disp8, CV_8U);
}
	

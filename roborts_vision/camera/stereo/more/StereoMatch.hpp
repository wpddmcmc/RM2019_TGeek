/*s
*@brief: 双目立体匹配, 西安电子科技大学 Robomasters 视觉组
*/
#pragma once

#include <iostream>   
#include <opencv2/opencv.h>   
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utility.hpp>   
using namespace cv;
using namespace std; 

class StereoMatch
{
public:
    StereoMatch();
    ~StereoMatch();
public:
    int ElasMatch(Mat leftImage, Mat rightImage);
};

int StereoMatch::ElasMatch(Mat leftImage, Mat rightImage)
{
    cv::Mat disp_l,disp_r,disp8u_l,disp8u_r;
    double minVal; double maxVal; //视差图的极值

    // cv::Mat leftImage = cv::imread("../test_images/leftr31.png",0);
    // cv::Mat rightImage = cv::imread("../test_images/rightr31.png",0);

    // 计算视差
    // generate disparity image using LIBELAS
    int bd = 0;
    const int32_t dims[3] = {leftImage.cols,leftImage.rows,leftImage.cols};
    cv::Mat leftdpf = cv::Mat::zeros(cv::Size(leftImage.cols,leftImage.rows), CV_32F);
    cv::Mat rightdpf = cv::Mat::zeros(cv::Size(leftImage.cols,leftImage.rows), CV_32F);
    Elas::parameters param;
    param.postprocess_only_left = false;
    Elas elas(param);
    elas.process(leftImage.data,rightImage.data,leftdpf.ptr<float>(0),rightdpf.ptr<float>(0),dims);

    cv::Mat(leftdpf(cv::Rect(bd,0,leftImage.cols,leftImage.rows))).copyTo(disp_l);
    cv::Mat(rightdpf(cv::Rect(bd,0,rightImage.cols,rightImage.rows))).copyTo(disp_r);

    //-- Check its extreme values
    cv::minMaxLoc( disp_l, &minVal, &maxVal );
    cout<<"Min disp: Max value"<< minVal<<maxVal; //numberOfDisparities.= (maxVal - minVal)

    //-- Display it as a CV_8UC1 image
    disp_l.convertTo(disp8u_l, CV_8U, 255/(maxVal - minVal));//(numberOfDisparities*16.)

    cv::minMaxLoc( disp_r, &minVal, &maxVal );
    cout<<"Min disp: Max value"<< minVal<<maxVal; //numberOfDisparities.= (maxVal - minVal)

    //-- Display it as a CV_8UC1 image
    disp_r.convertTo(disp8u_r, CV_8U, 255/(maxVal - minVal));//(numberOfDisparities*16.)

    cv::normalize(disp8u_l, disp8u_l, 0, 255, CV_MINMAX, CV_8UC1);    // obtain normalized image
    cv::normalize(disp8u_r, disp8u_r, 0, 255, CV_MINMAX, CV_8UC1);    // obtain normalized image

    cv::imshow("Left",leftImage);
    cv::imshow("Right",rightImage);

    cv::imshow("Elas_left",disp8u_l);
    cv::imshow("Elas_right",disp8u_r);
    cv::imwrite("Elas_left.png",disp8u_l);
    cv::imwrite("Elas_right.png",disp8u_r);

    cout<<endl<<"Over"<<endl;
    cv::waitKey(0);

    return 0;
}

int StereoMatch::BMMatching()
{
    cv::Mat disp,disp8u;
    double minVal; double maxVal; //视差图的极值

    cv::Mat leftImage = cv::imread("../test_images/leftr.png",0);
    cv::Mat rightImage = cv::imread("../test_images/rightr.png",0);

    int SADWindowSize = 19;
    int numberOfDisparities =16*3; /**< Range of disparity */
    numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((leftImage.cols/8) + 15) & -16;

    //bm.state->roi1 = remapMat.Calib_Roi_L;//左右视图的有效像素区域，一般由双目校正阶段的 cvStereoRectify 函数传递，也可以自行设定。
    //bm.state->roi2 = remapMat.Calib_Roi_R;//一旦在状态参数中设定了 roi1 和 roi2，OpenCV 会通过cvGetValidDisparityROI 函数计算出视差图的有效区域，在有效区域外的视差值将被清零。
    //bm.State->preFilterSize=41;//预处理滤波器窗口大小,5-21,odd
    bm.state->preFilterCap = 31; //63,1-31//预处理滤波器的截断值，预处理的输出值仅保留[-preFilterCap, preFilterCap]范围内的值,
    bm.state->SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 9; //SAD窗口大小5-21
    bm.state->minDisparity = 0; //64 最小视差，默认值为 0
    bm.state->numberOfDisparities = numberOfDisparities; //128视差窗口，即最大视差值与最小视差值之差, 窗口大小必须是 16 的整数倍
    bm.state->textureThreshold = 10;//低纹理区域的判断阈值。如果当前SAD窗口内所有邻居像素点的x导数绝对值之和小于指定阈值，则该窗口对应的像素点的视差值为 0
    bm.state->uniquenessRatio = 15;//5-15 视差唯一性百分比， 视差窗口范围内最低代价是次低代价的(1 + uniquenessRatio/100)倍时，最低代价对应的视差值才是该像素点的视差，否则该像素点的视差为 0
    bm.state->speckleWindowSize = 100;//检查视差连通区域变化度的窗口大小, 值为 0 时取消 speckle 检查
    bm.state->speckleRange = 32;//视差变化阈值，当窗口内视差变化大于阈值时，该窗口内的视差清零
    bm.state->disp12MaxDiff = 1;//左视差图（直接计算得出）和右视差图（通过cvValidateDisparity计算得出）之间的最大容许差异。超过该阈值的视差值将被清零。该参数默认为 -1，即不执行左右视差检查。
                                                      //注意在程序调试阶段最好保持该值为 -1，以便查看不同视差窗口生成的视差效果。

    // 计算视差
    bm(leftImage, rightImage, disp);

    //-- Check its extreme values
    cv::minMaxLoc( disp, &minVal, &maxVal );
    cout<<"Min disp: Max value"<< minVal<<maxVal; //numberOfDisparities.= (maxVal - minVal)

    //-- 4. Display it as a CV_8UC1 image
    disp.convertTo(disp8u, CV_8U, 255/(maxVal - minVal));//(numberOfDisparities*16.)
    cv::normalize(disp8u, disp8u, 0, 255, CV_MINMAX, CV_8UC1);    // obtain normalized image

    cv::imshow("left",leftImage);
    cv::imshow("right",leftImage);
    cv::imshow("Disp",disp8u);
    cv::imwrite("bm.png",disp8u);
    cv::waitKey(0);

}

int StereoMatch::SGBMMatching()
{
    cv::Mat disp,disp8u;
    double minVal; double maxVal; //视差图的极值

    cv::Mat leftImage = cv::imread("../test_images/leftr.png",0);
    cv::Mat rightImage = cv::imread("../test_images/rightr.png",0);

    int numberOfDisparities =16*2; /**< Range of disparity */
    numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((leftImage.cols/8) + 15) & -16;

    int SADWindowSize = 11;
    sgbm.preFilterCap = 63;
    sgbm.SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 3; //3-11

    int cn = leftImage.channels();

    sgbm.P1 = 8*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;//P1、P2的值越大，视差越平滑。P2>P1，可取（50，800）或者（40，2500）
    sgbm.P2 = 32*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
    sgbm.minDisparity = 0;
    sgbm.numberOfDisparities = numberOfDisparities; //128,256
    sgbm.uniquenessRatio = 10;    //10,0
    sgbm.speckleWindowSize = 100; //200
    sgbm.speckleRange = 32;
    sgbm.disp12MaxDiff = 1;
    sgbm.fullDP = 1;

    // 计算视差
    sgbm(leftImage, rightImage, disp);

    //-- Check its extreme values
    cv::minMaxLoc( disp, &minVal, &maxVal );
    cout<<"Min disp: Max value"<< minVal<<maxVal; //numberOfDisparities.= (maxVal - minVal)

    //-- 4. Display it as a CV_8UC1 image
    disp.convertTo(disp8u, CV_8U, 255/(maxVal - minVal));//(numberOfDisparities*16.)
    cv::normalize(disp8u, disp8u, 0, 255, CV_MINMAX, CV_8UC1);    // obtain normalized image

    cv::imshow("left",leftImage);
    cv::imshow("right",leftImage);
    cv::imshow("Disp",disp8u);
    cv::imwrite("sgbm.png",disp8u);
    cv::waitKey(0);

}

int StereoMatch::VARMatching()
{
    cv::Mat disp,disp8u;
    double minVal; double maxVal; //视差图的极值

    cv::Mat leftImage = cv::imread("../test_images/leftr.png",0);
    cv::Mat rightImage = cv::imread("../test_images/rightr.png",0);

    int numberOfDisparities =16*2; /**< Range of disparity */
    numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((leftImage.cols/8) + 15) & -16;

    var.levels = 3;                                 // ignored with USE_AUTO_PARAMS
    var.pyrScale = 0.5;                             // ignored with USE_AUTO_PARAMS
    var.nIt = 25;
    var.minDisp = -numberOfDisparities;
    var.maxDisp = 0;
    var.poly_n = 3;
    var.poly_sigma = 0.0;
    var.fi = 15.0f;
    var.lambda = 0.03f;
    var.penalization = var.PENALIZATION_TICHONOV;   // ignored with USE_AUTO_PARAMS
    var.cycle = var.CYCLE_V;                        // ignored with USE_AUTO_PARAMS
    var.flags = var.USE_SMART_ID | var.USE_AUTO_PARAMS | var.USE_INITIAL_DISPARITY | var.USE_MEDIAN_FILTERING ;

    // 计算视差
    var(leftImage, rightImage, disp);

    //-- Check its extreme values
    cv::minMaxLoc( disp, &minVal, &maxVal );
    cout<<"Min disp: Max value"<< minVal<<endl<<maxVal; //numberOfDisparities.= (maxVal - minVal)

    //-- 4. Display it as a CV_8UC1 image
    disp.convertTo(disp8u, CV_8U, 255/(maxVal - minVal));//(numberOfDisparities*16.)
    cv::normalize(disp8u, disp8u, 0, 255, CV_MINMAX, CV_8UC1);    // obtain normalized image

    cv::imshow("left",leftImage);
    cv::imshow("right",leftImage);
    cv::imshow("Disp",disp8u);
    cv::imwrite("var.png",disp8u);
    cv::waitKey(0);

}

int StereoMatch::GCMatching()
{
    IplImage * leftImage = cvLoadImage("../test_images/leftr31.png",0);
    IplImage * rightImage = cvLoadImage("../test_images/rightr31.png",0);

    CvStereoGCState* state = cvCreateStereoGCState( 16, 4 );
    IplImage * left_disp_  =cvCreateImage(cvGetSize(leftImage),leftImage->depth,1);
    IplImage * right_disp_ =cvCreateImage(cvGetSize(leftImage),leftImage->depth,1);
    cvFindStereoCorrespondenceGC( leftImage, rightImage, left_disp_, right_disp_, state, 0 );
    cvReleaseStereoGCState( &state );

    cvNamedWindow("Left",1);
    cvNamedWindow("Right",1);
    cvNamedWindow("GC_left",1);
    cvNamedWindow("GC_right",1);

    cvShowImage("Left",leftImage);
    cvShowImage("Right",rightImage);

    cvNormalize(left_disp_,left_disp_,0,255,CV_MINMAX,CV_8UC1);
    cvNormalize(right_disp_,right_disp_,0,255,CV_MINMAX,CV_8UC1);

    cvShowImage("GC_left",left_disp_);
    cvShowImage("GC_right",right_disp_);
    cvSaveImage("GC_left.png",left_disp_);
    cvSaveImage("GC_right.png",right_disp_);


    cout<<endl<<"Over"<<endl;
    cvWaitKey(0);
    cvDestroyAllWindows();
    cvReleaseImage(&leftImage);
    cvReleaseImage(&rightImage);
    return 0;
}

int StereoMatch::GCMatching_Mat()
{
    double minVal; double maxVal; //视差图的极值
    cv::Mat disp8u_l,disp8u_r;

    cv::Mat leftImage = cv::imread("../test_images/leftr31.png",0);
    cv::Mat rightImage = cv::imread("../test_images/rightr31.png",0);

    CvStereoGCState* state = cvCreateStereoGCState( 16, 5 );
    cv::Mat left_disp_  =leftImage.clone();
    cv::Mat right_disp_ =rightImage.clone();

    IplImage temp = (IplImage)leftImage;
    IplImage* leftimg = &temp;
    IplImage temp1 = (IplImage)rightImage;
    IplImage* rightimg = &temp1;
    IplImage temp2 = (IplImage)left_disp_;
    IplImage* leftdisp = &temp2;
    IplImage temp3 = (IplImage)right_disp_;
    IplImage* rightdisp = &temp3;

    cvFindStereoCorrespondenceGC( leftimg, rightimg, leftdisp, rightdisp, state, 0 );
    cvReleaseStereoGCState( &state );

    cv::namedWindow("Left",1);
    cv::namedWindow("Right",1);
    cv::namedWindow("GC_left",1);
    cv::namedWindow("GC_right",1);

    cv::imshow("Left",leftImage);
    cv::imshow("Right",rightImage);

/*    //-- Check its extreme values
    cv::minMaxLoc(right_disp_, &minVal, &maxVal );
    cout<<"Min disp: Max value"<< minVal<<maxVal; //numberOfDisparities.= (maxVal - minVal)

    //-- Display it as a CV_8UC1 image
    right_disp_.convertTo(disp8u_r, CV_8U, 255/(maxVal - minVal));//(numberOfDisparities*16.)

    cv::normalize(left_disp_,left_disp_,0,255,CV_MINMAX,CV_8UC1);
    cv::normalize(disp8u_r,disp8u_r,0,255,CV_MINMAX,CV_8UC1);
*/
    cv::normalize(left_disp_,left_disp_,0,255,CV_MINMAX,CV_8UC1);
    cv::normalize(right_disp_,right_disp_,0,255,CV_MINMAX,CV_8UC1);

    cv::imshow("GC_left",left_disp_);
    cv::imshow("GC_right",right_disp_);
    cv::imwrite("GC_left.png",left_disp_);
    cv::imwrite("GC_right.png",right_disp_);

    cout<<endl<<"Over"<<endl;
    cv::waitKey(0);

    return 0;
}

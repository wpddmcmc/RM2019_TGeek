
#include "task.h"

const int  red_color_diff = 50;
const int blue_color_diff = 50;

void task2(cv::Mat src, std::vector<int>& key)
{
    // step1
	std::vector<cv::Mat> bgr_channel;
	cv::split(src, bgr_channel);
	
    // step2
    cv::Mat color_light_RED;
    cv::Mat color_light_BLUE;

	cv::subtract(bgr_channel[2], bgr_channel[1], color_light_RED);
	cv::subtract(bgr_channel[0], bgr_channel[1], color_light_BLUE);
    
    // step3
    cv::Mat binary_color_img_RED;      // 颜色二值化
    cv::Mat binary_color_img_BLUE;     // 颜色二值化

    cv::threshold(color_light_RED, binary_color_img_RED, red_color_diff, 255, CV_THRESH_BINARY);
    cv::threshold(color_light_BLUE, binary_color_img_BLUE, blue_color_diff, 255, CV_THRESH_BINARY);

    std::vector<std::vector<cv::Point>> contours_color_RED;
	cv::findContours(binary_color_img_RED, contours_color_RED, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    std::vector<std::vector<cv::Point>> contours_color_BLUE;
	cv::findContours(binary_color_img_BLUE, contours_color_BLUE, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    
    // step4
    // Get Moments for all Contours and the mass centers
	// std::vector<cv::Moments> mu(contours.size());
	// std::vector<cv::Point2f> mc(contours.size());

	// for (int i = 0; i < contours.size(); i++)
	// {
	// 	mu[i] = cv::moments(contours[i], false);
	// 	mc[i] = cv::Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
	// }

    // std::vector<cv::Point> pointsseq;				    // used to save the approximated sides of each contour
    // cv::approxPolyDP(contours[i], pointsseq, arcLength(contours[i], true)*0.02, true);
    // if (pointsseq.size() < 10 ) // 条件1满足，则
    // {
    //     // hierarchy[k][2]； // 没有父轮廓，也没有子轮廓
    //     // 
    // }
    cv::waitKey(1);
}

#pragma once
#include "opencv2/opencv.hpp"
using namespace cv;

namespace roborts_detection{


class RuneDetector
{
  cv::Mat img_src_;
  cv::Mat src_;
  cv::Mat binary_;
  float w0;
  float L_R;
  int color;
  int colorwise;
  cv::Point2f his_cen_point;
  cv::Point2f predict_point;
  cv::Point2f his;

  std::vector<std::vector<Point>> contours_color;

  std::vector<cv::Point2f> center_point;

  
  cv::Point2f get_center_point(vector<cv::Point> cen_r)
  {
      Moments mu = moments(cen_r, false);	
      return cv::Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00); 
  }

  float distance_hanshu(cv::Point2f points,cv::Point2f cen_R)
  {
      float distances = sqrt((points.x-cen_R.x)*(points.x-cen_R.x) + (points.y-cen_R.y)*(points.y-cen_R.y));
      return distances;
  }

public:
  void Detect();
  void gammaProcessImage(Mat& oriMat,double gamma,Mat &outputMat);
};

}

}
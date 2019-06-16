
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <iostream>
#include <vector>
#include "opencv2/opencv.hpp"

#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag36h11.h"

namespace roborts_detection {

class TagDetection 
{
  AprilTags::TagDetector* m_tagDetector;
  AprilTags::TagCodes m_tagCodes;

  ros::NodeHandle nh_;
  image_transport::Subscriber sub_;
  cv::Mat src_img_;
  cv::Mat gray_img_;

public:
  TagDetection(): m_tagCodes(AprilTags::tagCodes36h11)
  {
    m_tagDetector = new AprilTags::TagDetector(m_tagCodes);

    image_transport::ImageTransport it(nh_);
    sub_ = it.subscribe("/gimbal_camera/image_raw", 20, boost::bind(&TagDetection::ReceiveImg, this, _1));
  }

  void ReceiveImg(const sensor_msgs::ImageConstPtr &msg) 
  {
    src_img_ = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
    cv::cvtColor(src_img_, gray_img_, CV_BGR2GRAY);

    vector<AprilTags::TagDetection> detections = m_tagDetector->extractTags(gray_img_);

    for (int i=0; i<detections.size(); i++) detections[i].draw(src_img_);
    
    cv::imshow("apriltags_TagDetection", src_img_); 
    cv::waitKey(1);
  }

};

} // namespace roborts_detection

int main(int argc, char* argv[]) 
{
  ros::init(argc, argv, "apriltags_detection");

  roborts_detection::TagDetection detector;

  ros::spin();

  return 0;
}


#include "opencv2/opencv.hpp"

color = 0;//0 for red, 1 for blue;
int Maxarea = 1500;
int Minarea = 1000;


namespace roborts_detection{

  void RuneDetector::Detect()
  {
    double start = static_cast<double>(getTickCount());
    if(src_1.empty())
    {
      cout<<"no video"<<endl;
      break;
    }
    // Rect rect_1(250,0,640,480);//200,0
    // src_2 = Mat(src_1,rect_1);
    gammaProcessImage(img_src_, 1.5, src_); // 提高对比度
    std::vector<cv::Mat> bgr_channel;
    cv::Mat binary_color_img;
	split(src_, bgr_channel);
	if (color == 0){
      binary_color_img = bgr_channel.at(2) - bgr_channel.at(0);
      cv:: threshold(binary_color_img,binary_color_img, 50, 255, CV_THRESH_BINARY);//130
    }
	else{
      binary_color_img = bgr_channel.at(0) - bgr_channel.at(2);
      cv::threshold(binary_color_img,binary_color_img, 150, 255, CV_THRESH_BINARY);//130
    }
    binary_color_img.copyTo(binary_);
    GaussianBlur(binary_, binary_, Size(3,3), 0, 0);
    floodFill(binary_color_img, Point(0,0), Scalar(255));
    threshold(binary_color_img, binary_color_img, 0, 255, THRESH_BINARY_INV);
    cv::Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
    cv::morphologyEx(binary_color_img, binary_color_img, MORPH_OPEN, element);
    cv::findContours(binary_color_img, contours_color, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    std::vector<std::vector<cv::Point>> contours_color_l;
    std::vector<std::vector<cv::Point>> contours_color_finall;
    findContours(binary_, contours_color_l, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    for (int i=0;i<contours_color_l.size();i++)
    {
      double Cont = fabs(contourArea(contours_color_l[i],true));
      if(Cont < 5000 && Cont > 3500){
        contours_color_finall.push_back(contours_color_l[i]);
        break;
      }
    }
    if (contours_color_finall.empty()){
      continue;
    }
    std::vector<std::vector<Point>> contours_ploy(contours_color.size());
    std::vector<cv::RotatedRect> RotatedRect_ploy;
    std::vector<std::vector<cv::Point>> contours_r;
    cv::RotatedRect predict_rect;
    cv::Point2f predict_rect_points[4];
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
      if (max/min >1.5 && max/min< 2.2 && Cont <Maxarea && Cont >Minarea){
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
  void RuneDetector::gammaProcessImage(Mat& oriMat,double gamma,Mat &outputMat)
  { 
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

}
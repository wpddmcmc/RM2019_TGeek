// SAD
#include <iostream>   
#include <opencv2/opencv.h>   
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utility.hpp>   
using namespace cv;
using namespace std;  
int GetHammingWeight(unsigned int value);  
int _SADmain(){  
    /*Half of the window size for the census transform*/  
    int hWin = 11;  
    int compareLength = (2*hWin+1)*(2*hWin+1);  

    cout<<"hWin: "<<hWin<<";  "<<"compare length:  "<<compareLength<<endl;    
    cout<<"SAD test"<<endl;  
    // char stopKey;   
    IplImage * leftImage = cvLoadImage("left.bmp",0);  
    IplImage * rightImage = cvLoadImage("right.bmp",0);  

    int imageWidth = leftImage->width;  
    int imageHeight =leftImage->height;  

    IplImage * SADImage = cvCreateImage(cvGetSize(leftImage),leftImage->depth,1);  
    IplImage * MatchLevelImage = cvCreateImage(cvGetSize(leftImage),leftImage->depth,1);  

    int minDBounds = 0;  
    int maxDBounds = 31;  

    cvNamedWindow("Left",1);  
    cvNamedWindow("Right",1);  
    cvNamedWindow("Census",1);  
    cvNamedWindow("MatchLevel",1);  

    cvShowImage("Left",leftImage);  
    cvShowImage("Right",rightImage);  



    /*Census Transform */  
    int i,j ,m,n,k;  
    unsigned char centerPixel = 0;   
    unsigned char neighborPixel = 0;  
    int bitCount = 0;  
    unsigned int bigger = 0;  



    int sum = 0;  
    unsigned int *matchLevel = new unsigned int[maxDBounds - minDBounds  + 1];  
    int tempMin = 0;  
    int tempIndex = 0;  

    unsigned char* dst;  
    unsigned char* leftSrc  = NULL;  
    unsigned char* rightSrc = NULL;  

    unsigned char leftPixel = 0;  
    unsigned char rightPixel =0;  
    unsigned char subPixel = 0;  


    for(i = 0 ; i < leftImage->height;i++){  
        for(j = 0; j< leftImage->width;j++){  

            for (k = minDBounds;k <= maxDBounds;k++)  
            {  
                sum = 0;  
                for (m = i-hWin; m <= i + hWin;m++)  
                {  
                    for (n = j - hWin; n <= j + hWin;n++)  
                    {  
                          if (m < 0 || m >= imageHeight || n <0 || n >= imageWidth )  
                          {  
                              subPixel  = 0;  
                          }else if (n + k >= imageWidth)  
                          {  
                              subPixel = 0;  
                          }else  
                          {  
                              leftSrc = (unsigned char*)leftImage->imageData   
                                          + m*leftImage->widthStep + n + k;   
                              rightSrc = (unsigned char*)rightImage->imageData   
                                           + m*rightImage->widthStep + n;  

                              leftPixel = *leftSrc;  
                              rightPixel = *rightSrc;  
                              if (leftPixel > rightPixel)  
                              {  
                                   subPixel = leftPixel - rightPixel;  
                              }else   
                              {  
                                   subPixel = rightPixel -leftPixel;  
                              }  

                          }  

                          sum += subPixel;  
                    }  
                }  
                matchLevel[k] = sum;  
                //cout<<sum<<endl;   
            }  

            /*寻找最佳匹配点*/  
           // matchLevel[0] = 1000000;   

            tempMin = 0;  
            tempIndex = 0;  
            for ( m = 1;m < maxDBounds - minDBounds + 1;m++)  
            {  
                //cout<<matchLevel[m]<<endl;   
                if (matchLevel[m] < matchLevel[tempIndex])  
                {  
                    tempMin = matchLevel[m];  
                    tempIndex = m;  
                }  
            }  
            dst = (unsigned char *)SADImage->imageData + i*SADImage->widthStep + j;  
            //cout<<"index: "<<tempIndex<<"  ";   

            *dst = tempIndex*8;  

            dst = (unsigned char *)MatchLevelImage->imageData + i*MatchLevelImage->widthStep + j;  
            *dst = tempMin;  
            //cout<<"min:  "<<tempMin<<"  ";   
            //cout<< tempIndex<<"  " <<tempMin<<endl;   
        }  
        //cvWaitKey(0);   
    }  

    cvShowImage("Census",SADImage);  
    cvShowImage("MatchLevel",MatchLevelImage);  
    cvSaveImage("depth.jpg",SADImage);  
    cvSaveImage("matchLevel.jpg",MatchLevelImage);  

    cvWaitKey(0);  
    cvDestroyAllWindows();  
    cvReleaseImage(&leftImage);  
    cvReleaseImage(&rightImage);  
    return 0;  
}  

// SSD
using namespace std;  
int GetHammingWeight(unsigned int value);  
int _SSDmain(){  
    /*Half of the window size for the census transform*/  
    int hWin = 11;  
    int compareLength = (2*hWin+1)*(2*hWin+1);  

    cout<<"hWin: "<<hWin<<";  "<<"compare length:  "<<compareLength<<endl;    
    cout<<"SAD test"<<endl;  
    // char stopKey;   
    IplImage * leftImage = cvLoadImage("l2.jpg",0);  
    IplImage * rightImage = cvLoadImage("r2.jpg",0);  

    int imageWidth = leftImage->width;  
    int imageHeight =leftImage->height;  

    IplImage * SADImage = cvCreateImage(cvGetSize(leftImage),leftImage->depth,1);  
    IplImage * MatchLevelImage = cvCreateImage(cvGetSize(leftImage),leftImage->depth,1);  

    int minDBounds = 0;  
    int maxDBounds = 31;  

    cvNamedWindow("Left",1);  
    cvNamedWindow("Right",1);  
    cvNamedWindow("Census",1);  
    cvNamedWindow("MatchLevel",1);  

    cvShowImage("Left",leftImage);  
    cvShowImage("Right",rightImage);  



    /*Census Transform */  
    int i,j ,m,n,k;  
    unsigned char centerPixel = 0;   
    unsigned char neighborPixel = 0;  
    int bitCount = 0;  
    unsigned int bigger = 0;  



    int sum = 0;  
    unsigned int *matchLevel = new unsigned int[maxDBounds - minDBounds  + 1];  
    int tempMin = 0;  
    int tempIndex = 0;  

    unsigned char* dst;  
    unsigned char* leftSrc  = NULL;  
    unsigned char* rightSrc = NULL;  

    unsigned char leftPixel = 0;  
    unsigned char rightPixel =0;  
    unsigned char subPixel = 0;  


    for(i = 0 ; i < leftImage->height;i++){  
        for(j = 0; j< leftImage->width;j++){  

            for (k = minDBounds;k <= maxDBounds;k++)  
            {  
                sum = 0;  
                for (m = i-hWin; m <= i + hWin;m++)  
                {  
                    for (n = j - hWin; n <= j + hWin;n++)  
                    {  
                          if (m < 0 || m >= imageHeight || n <0 || n >= imageWidth )  
                          {  
                              subPixel  = 0;  
                          }else if (n + k >= imageWidth)  
                          {  
                              subPixel = 0;  
                          }else  
                          {  
                              leftSrc = (unsigned char*)leftImage->imageData   
                                          + m*leftImage->widthStep + n + k;   
                              rightSrc = (unsigned char*)rightImage->imageData   
                                           + m*rightImage->widthStep + n;  

                              leftPixel = *leftSrc;  
                              rightPixel = *rightSrc;  
                              if (leftPixel > rightPixel)  
                              {  
                                   subPixel = leftPixel - rightPixel;  
                              }else   
                              {  
                                   subPixel = rightPixel -leftPixel;  
                              }  

                          }  

                          sum += subPixel*subPixel;  
                    }  
                }  
                matchLevel[k] = sum;  
                //cout<<sum<<endl;   
            }  

            /*寻找最佳匹配点*/  
           // matchLevel[0] = 1000000;   

            tempMin = 0;  
            tempIndex = 0;  
            for ( m = 1;m < maxDBounds - minDBounds + 1;m++)  
            {  
                //cout<<matchLevel[m]<<endl;   
                if (matchLevel[m] < matchLevel[tempIndex])  
                {  
                    tempMin = matchLevel[m];  
                    tempIndex = m;  
                }  
            }  
            dst = (unsigned char *)SADImage->imageData + i*SADImage->widthStep + j;  
            //cout<<"index: "<<tempIndex<<"  ";   

            *dst = tempIndex*8;  

            dst = (unsigned char *)MatchLevelImage->imageData + i*MatchLevelImage->widthStep + j;  
            *dst = tempMin;  
            //cout<<"min:  "<<tempMin<<"  ";   
            //cout<< tempIndex<<"  " <<tempMin<<endl;   
        }  
        //cvWaitKey(0);   
    }  

    cvShowImage("Census",SADImage);  
    cvShowImage("MatchLevel",MatchLevelImage);  
    cvSaveImage("depth.jpg",SADImage);  
    cvSaveImage("matchLevel.jpg",MatchLevelImage);  

    cvWaitKey(0);  
    cvDestroyAllWindows();  
    cvReleaseImage(&leftImage);  
    cvReleaseImage(&rightImage);  
    return 0;  
}  

// ZSSD算法：
int GetHammingWeight(unsigned int value);  
int _ZSSDmain(){  
    /*Half of the window size for the census transform*/  
    int hWin = 11;  
    int compareLength = (2*hWin+1)*(2*hWin+1);  

    cout<<"hWin: "<<hWin<<";  "<<"compare length:  "<<compareLength<<endl;    
    cout<<"ZSSD test"<<endl;  
    // char stopKey;   
  /*  IplImage * leftImage = cvLoadImage("l2.jpg",0); 
    IplImage * rightImage = cvLoadImage("r2.jpg",0);*/  

    IplImage * leftImage = cvLoadImage("left.bmp",0);  
    IplImage * rightImage = cvLoadImage("right.bmp",0);  

    int imageWidth = leftImage->width;  
    int imageHeight =leftImage->height;  

    IplImage * SADImage = cvCreateImage(cvGetSize(leftImage),leftImage->depth,1);  
    IplImage * MatchLevelImage = cvCreateImage(cvGetSize(leftImage),leftImage->depth,1);  

    int minDBounds = 0;  
    int maxDBounds = 31;  

    cvNamedWindow("Left",1);  
    cvNamedWindow("Right",1);  
    cvNamedWindow("Census",1);  
    cvNamedWindow("MatchLevel",1);  

    cvShowImage("Left",leftImage);  
    cvShowImage("Right",rightImage);  



    /*Census Transform */  
    int i,j ,m,n,k;  
    unsigned char centerPixel = 0;   
    unsigned char neighborPixel = 0;  
    int bitCount = 0;  
    unsigned int bigger = 0;  



    int sumLeft = 0;  
    int sumRight = 0;  
    int sum =0;  

    int zSumLeft  = 0;  
    int zSumRight = 0;  

    unsigned int *matchLevel = new unsigned int[maxDBounds - minDBounds  + 1];  
    int tempMin = 0;  
    int tempIndex = 0;  

    unsigned char* dst;  
    unsigned char* leftSrc  = NULL;  
    unsigned char* rightSrc = NULL;  

    unsigned char leftPixel = 0;  
    unsigned char rightPixel =0;  
    unsigned char subPixel = 0;  
    unsigned char meanLeftPixel  = 0;  
    unsigned char meanRightPixel = 0;  


    for(i = 0 ; i < leftImage->height;i++){  
        for(j = 0; j< leftImage->width;j++){  

            /*均值计算 */  
            for (k = minDBounds;k <= maxDBounds;k++)  
            {  
                sumLeft  = 0;  
                sumRight = 0;  
                for (m = i-hWin; m <= i + hWin;m++)  
                {  
                    for (n = j - hWin; n <= j + hWin;n++)  
                    {  
                          if (m < 0 || m >= imageHeight || n <0 || n >= imageWidth )  
                          {  
                              sumLeft += 0;  
                          }else {  
                              leftSrc = (unsigned char*)leftImage->imageData   
                                  + m*leftImage->widthStep + n + k;   
                              leftPixel = *leftSrc;  
                              sumLeft += leftPixel;  
                          }  


                          if (m < 0 || m >= imageHeight || n + k <0 || n +k >= imageWidth)  
                          {  
                               sumRight += 0;  
                          }else  
                          {   
                              rightSrc = (unsigned char*)rightImage->imageData   
                                           + m*rightImage->widthStep + n;  
                              rightPixel = *rightSrc;  
                              sumRight += rightPixel;     
                          }  

                    }  
                }  

                meanLeftPixel  = sumLeft/compareLength;  
                meanRightPixel = sumRight/compareLength;  
                 /*ZSSD*/  
               sum = 0;  
                 for (m = i-hWin; m <= i + hWin;m++)  
                {  
                    for (n = j - hWin; n <= j + hWin;n++)  
                    {  
                          if (m < 0 || m >= imageHeight || n <0 || n >= imageWidth )  
                          {  
                              //zSumLeft += 0;   
                              leftPixel = 0;  
                          }else {  
                              leftSrc = (unsigned char*)leftImage->imageData   
                                  + m*leftImage->widthStep + n + k;   
                              leftPixel = *leftSrc;  
                              //zSumLeft += (leftPixel - meanLeftPixel)*(leftPixel -meanLeftPixel);   
                          }  


                          if (m < 0 || m >= imageHeight || n + k <0 || n +k >= imageWidth)  
                          {  
                               //zSumRight += 0;   
                              rightPixel = 0;  
                          }else  
                          {   
                              rightSrc = (unsigned char*)rightImage->imageData   
                                           + m*rightImage->widthStep + n;  
                              rightPixel = *rightSrc;  
                              // zSumRight += (rightPixel - meanRightPixel)*(rightPixel - meanRightPixel);      
                          }  

                          sum += ((rightPixel - meanRightPixel)-(leftPixel -meanLeftPixel))  
                                 *((rightPixel - meanRightPixel)-(leftPixel -meanLeftPixel));  
                    }  
                }  


                matchLevel[k] = sum;  
                //cout<<sum<<endl;   
            }  

            /*寻找最佳匹配点*/  
           // matchLevel[0] = 1000000;   

            tempMin = 0;  
            tempIndex = 0;  
            for ( m = 1;m < maxDBounds - minDBounds + 1;m++)  
            {  
                //cout<<matchLevel[m]<<endl;   
                if (matchLevel[m] < matchLevel[tempIndex])  
                {  
                    tempMin = matchLevel[m];  
                    tempIndex = m;  
                }  
            }  
            dst = (unsigned char *)SADImage->imageData + i*SADImage->widthStep + j;  
            //cout<<"index: "<<tempIndex<<"  ";   

            *dst = tempIndex*8;  
            dst = (unsigned char *)MatchLevelImage->imageData + i*MatchLevelImage->widthStep + j;  
            *dst = tempMin;  
            //cout<<"min:  "<<tempMin<<"  ";   
            //cout<< tempIndex<<"  " <<tempMin<<endl;   
        }  
       // cvWaitKey(0);   
    }  

    cvShowImage("Census",SADImage);  
    cvShowImage("MatchLevel",MatchLevelImage);  
    cvSaveImage("depth.jpg",SADImage);  
    cvSaveImage("matchLevel.jpg",MatchLevelImage);  


    cout<<endl<<"Over"<<endl;  
    cvWaitKey(0);  
    cvDestroyAllWindows();  
    cvReleaseImage(&leftImage);  
    cvReleaseImage(&rightImage);  
    return 0;  
}  

// census 算法：
int GetHammingWeight(unsigned int value);  
int _CENSUSmain(){  
    /*Half of the window size for the census transform*/  
    int hWin = 11;  
    int bitlength = 0;  
    if ((2*hWin+1)*(2*hWin+1)%32 == 0)  
    {  
        bitlength = (2*hWin+1)*(2*hWin+1)/32;  
    }else {  
        bitlength = (2*hWin+1)*(2*hWin+1)/32  + 1;   
    }    
    cout<<"hWin: "<<hWin<<";  "<<"bit length:  "<<bitlength<<endl;    
    cout<<"Census test"<<endl;  
   // char stopKey;   
    IplImage * leftImage = cvLoadImage("left.bmp",0);  
    IplImage * rightImage = cvLoadImage("right.bmp",0);  

    int imageWidth = leftImage->width;  
    int imageHeight =leftImage->height;  

    IplImage * CensusImage = cvCreateImage(cvGetSize(leftImage),leftImage->depth,1);  
    IplImage * MatchLevelImage = cvCreateImage(cvGetSize(leftImage),leftImage->depth,1);  

     int minDBounds = 0;  
     int maxDBounds = 31;  
    // int leftCensus[imageHeight][imageWidth][bitlength] = {0};     
     unsigned  int *leftCensus = new unsigned int[imageHeight*imageWidth*bitlength];  
     unsigned  int *rightCensus = new unsigned int[imageHeight*imageWidth*bitlength];   
     for (int i = 0;i < imageHeight*imageWidth*bitlength;i++)  
     {  
         leftCensus[i] = 0;  
         rightCensus[i] = 0;  
     }  
     int pointCnt = 0;  




    cvNamedWindow("Left",1);  
    cvNamedWindow("Right",1);  
    cvNamedWindow("Census",1);  
    cvNamedWindow("MatchLevel",1);  

    cvShowImage("Left",leftImage);  
    cvShowImage("Right",rightImage);  



    /*Census Transform */  
   int i,j ,m,n,k,l;  
   unsigned char centerPixel = 0;   
   unsigned char neighborPixel = 0;  
   int bitCount = 0;  
   unsigned int bigger = 0;  
    for(i = 0 ; i < leftImage->height;i++){  
        for(j = 0; j< leftImage->width;j++){  
             centerPixel = *((unsigned char *)leftImage->imageData + i*leftImage->widthStep + j);   
             bitCount = 0;  

             for (m = i - hWin; m <= i + hWin;m++)  
             {  
                 for (n = j - hWin; n<= j+hWin;n++)  
                 {  
                     bitCount++;  
                     if (m < 0 || m >= leftImage->height || n < 0 || n >= leftImage->width)  
                     {  
                         neighborPixel = 0;  
                     }else{  
                          neighborPixel = *((unsigned char *)leftImage->imageData + m*leftImage->widthStep + n);   
                     }  
                     bigger = (neighborPixel > centerPixel)?1:0;  
                     leftCensus[(i*imageWidth + j)*bitlength + bitCount/32] |= (bigger<<(bitCount%32));  
                 }  
             }  

        }  
    }  

    for(i = 0 ; i < rightImage->height;i++){  
        for(j = 0; j< rightImage->width;j++){  
            centerPixel = *((unsigned char *)rightImage->imageData + i*rightImage->widthStep + j);   
            bitCount = 0;  

            for (m = i - hWin; m <= i + hWin;m++)  
            {  
                for (n = j - hWin; n<= j+hWin;n++)  
                {  
                    bitCount++;  
                    if (m < 0 || m >= rightImage->height || n < 0 || n >= rightImage->width)  
                    {  
                        neighborPixel = 0;  
                    }else{  
                        neighborPixel = *((unsigned char *)rightImage->imageData + m*rightImage->widthStep + n);   
                    }  
                    bigger = (neighborPixel > centerPixel)?1:0;  
                    rightCensus[(i*imageWidth + j)*bitlength + bitCount/32] |= (bigger<<(bitCount%32));  
                }  
            }  

        }  
    }  
    int sum = 0;  
    unsigned int *matchLevel = new unsigned int[maxDBounds - minDBounds  + 1];  
    int tempMin = 0;  
    int tempIndex = 0;  
    unsigned  char *dst;  
    unsigned char pixle = 0;  
    for(i = 0 ; i < rightImage->height;i++){  
        for(j = 0; j< rightImage->width;j++){  


            for (k = minDBounds;k <= maxDBounds;k++)  
            {  
                sum = 0;  
                for (l = 0;l< bitlength;l++)  
                {     
                    if (((i*imageWidth+j+k)*bitlength + l) < imageHeight*imageWidth*bitlength)  
                    {  
                        sum += GetHammingWeight(rightCensus[(i*imageWidth+j)*bitlength + l]   
                        ^ leftCensus[(i*imageWidth+j+k)*bitlength + l]);  
                    }else {  
                        //sum += 0;   
                       // cout<<".";   
                    }  

                }  
                matchLevel[k] = sum;  
            }  

            /*寻找最佳匹配点*/  
            tempMin = 0;  
            tempIndex = 0;  
            for ( m = 1;m < maxDBounds - minDBounds + 1;m++)  
            {  
                 if (matchLevel[m] < matchLevel[tempIndex])  
                 {  
                     tempMin = matchLevel[m];  
                     tempIndex = m;  
                 }  
            }  

            if (tempMin > (2*hWin+1)*(2*hWin+1)*0.2)  
            {  
                tempMin = 0;  
                pointCnt++;  
            }else{  
                tempMin = 255;  
            }  
            dst = (unsigned char *)CensusImage->imageData + i*CensusImage->widthStep + j;  
            *dst = tempIndex*8;  

            dst = (unsigned char *)MatchLevelImage->imageData + i*MatchLevelImage->widthStep + j;  
            *dst = tempMin;  

            //cout<< tempIndex<<"  " <<tempMin<<endl;;   

        }  
    }  
    cout<<"pointCnt:  "<<pointCnt<<endl;  
    cvShowImage("Census",CensusImage);  
    cvShowImage("MatchLevel",MatchLevelImage);  
    cvSaveImage("depth.jpg",CensusImage);  
    cvSaveImage("matchLevel.jpg",MatchLevelImage);  

    cvWaitKey(0);  
    cvDestroyAllWindows();  
    cvReleaseImage(&leftImage);  
    cvReleaseImage(&rightImage);  
    return 0;  
}  


int GetHammingWeight(unsigned int value)  
{  
    if(value == 0) return 0;  

    int a = value;  
    int b = value -1;  
    int c = 0;  

    int count = 1;  
    while(c = a & b)  
    {  
        count++;  
        a = c;  
        b = c-1;  
    }  
    return count;  
}  

// NCC算法：
#include<iostream>   
#include<cv.h>   
#include<highgui.h>   
#include <cmath>   
using namespace std;  

int _NCCmain(){  
    /*Half of the window size for the census transform*/  
    int hWin = 11;  
    int compareLength = (2*hWin+1)*(2*hWin+1);  

    cout<<"hWin: "<<hWin<<";  "<<"compare length:  "<<compareLength<<endl;    
    cout<<"NCC test"<<endl;  
  /*  IplImage * leftImage = cvLoadImage("l2.jpg",0); 
    IplImage * rightImage = cvLoadImage("r2.jpg",0);*/  

    IplImage * leftImage = cvLoadImage("left.bmp",0);                             
    IplImage * rightImage = cvLoadImage("right.bmp",0);  

    int imageWidth = leftImage->width;  
    int imageHeight =leftImage->height;  

    IplImage * NCCImage = cvCreateImage(cvGetSize(leftImage),leftImage->depth,1);  
    IplImage * MatchLevelImage = cvCreateImage(cvGetSize(leftImage),leftImage->depth,1);  

    int minDBounds = 0;  
    int maxDBounds = 31;  

    cvNamedWindow("Left",1);  
    cvNamedWindow("Right",1);  
    cvNamedWindow("Census",1);  
    cvNamedWindow("MatchLevel",1);  

    cvShowImage("Left",leftImage);  
    cvShowImage("Right",rightImage);  



    /*Census Transform */  
    int i,j ,m,n,k;  
    unsigned char centerPixel = 0;   
    unsigned char neighborPixel = 0;  
    int bitCount = 0;  
    unsigned int bigger = 0;  

    unsigned int sum =0;  
    unsigned int leftSquareSum = 0;  
    unsigned int rightSquareSum = 0;   

    double *matchLevel = new double[maxDBounds - minDBounds  + 1];  
    double tempMax = 0;  
    int tempIndex = 0;  

    unsigned char* dst;  
    unsigned char* leftSrc  = NULL;  
    unsigned char* rightSrc = NULL;  

    unsigned char leftPixel = 0;  
    unsigned char rightPixel =0;  
    unsigned char subPixel = 0;  
    unsigned char meanLeftPixel  = 0;  
    unsigned char meanRightPixel = 0;  


    for(i = 0 ; i < leftImage->height;i++){  
        for(j = 0; j< leftImage->width;j++){  

            /*均值计算 */  
            for (k = minDBounds;k <= maxDBounds;k++)  
            {  
                sum = 0;  
                leftSquareSum  = 0;  
                rightSquareSum = 0;  

                 for (m = i-hWin; m <= i + hWin;m++)  
                {  
                    for (n = j - hWin; n <= j + hWin;n++)  
                    {  
                          if (m < 0 || m >= imageHeight || n <0 || n >= imageWidth )  
                          {  

                              leftPixel = 0;  
                          }else {  
                              leftSrc = (unsigned char*)leftImage->imageData   
                                  + m*leftImage->widthStep + n + k;   
                              leftPixel = *leftSrc;  

                          }  


                          if (m < 0 || m >= imageHeight || n + k <0 || n +k >= imageWidth)  
                          {  

                              rightPixel = 0;  
                          }else  
                          {   
                              rightSrc = (unsigned char*)rightImage->imageData   
                                           + m*rightImage->widthStep + n;  
                              rightPixel = *rightSrc;  

                          }  

                          sum +=  leftPixel*rightPixel;  
                          leftSquareSum  += leftPixel*leftPixel;  
                          rightSquareSum += rightPixel*rightPixel;  

                    }  
                }  
                matchLevel[k] = (double)sum/(sqrt(double(leftSquareSum))*sqrt((double)rightSquareSum));  

            }  


            tempMax = 0;  
            tempIndex = 0;  
            for ( m = 1;m < maxDBounds - minDBounds + 1;m++)  
            {  

                if (matchLevel[m] > matchLevel[tempIndex])  
                {  
                    tempMax = matchLevel[m];  
                    tempIndex = m;  
                }  
            }  
            dst = (unsigned char *)NCCImage->imageData + i*NCCImage->widthStep + j;  

            *dst = tempIndex*8;  
            dst = (unsigned char *)MatchLevelImage->imageData + i*MatchLevelImage->widthStep + j;  
            *dst = (unsigned char)(tempMax*255);  

        }  

    }  

    cvShowImage("Census",NCCImage);  
    cvShowImage("MatchLevel",MatchLevelImage);  
    cvSaveImage("depth.jpg",NCCImage);  
    cvSaveImage("matchLevel.jpg",MatchLevelImage);  


    cout<<endl<<"Over"<<endl;  
    cvWaitKey(0);  
    cvDestroyAllWindows();  
    cvReleaseImage(&leftImage);  
    cvReleaseImage(&rightImage);  
    return 0;  
}  

// DP算法：
#include <cstdio>   
#include <cstring>   
#include <iostream>   
#include<cv.h>   
#include<highgui.h>   
#include <cmath>   

using namespace std;  
const int Width =  1024;  
const int Height = 1024;  
 int Ddynamic[Width][Width];  

int  _DPmain()  
{    
    /*Half of the window size for the census transform*/  
    int hWin = 11;  
    int compareLength = (2*hWin+1)*(2*hWin+1);  

    cout<<"hWin: "<<hWin<<";  "<<"compare length:  "<<compareLength<<endl;    
    cout<<"belief propagation test"<<endl;  

    IplImage * leftImage = cvLoadImage("l2.jpg",0);  
    IplImage * rightImage = cvLoadImage("r2.jpg",0);  

   // IplImage * leftImage = cvLoadImage("left.bmp",0);                              
   // IplImage * rightImage = cvLoadImage("right.bmp",0);   

    int imageWidth = leftImage->width;  
    int imageHeight =leftImage->height;  



    IplImage * DPImage = cvCreateImage(cvGetSize(leftImage),leftImage->depth,1);  
    //IplImage * MatchLevelImage = cvCreateImage(cvGetSize(leftImage),leftImage->depth,1);   

    unsigned char * pPixel = NULL;  
    unsigned char  pixel;  
    for (int i = 0; i< imageHeight;i++)  
    {  
        for (int j =0; j < imageWidth;j++ )  
        {  
            pPixel = (unsigned char *)DPImage->imageData + i*DPImage->widthStep + j;  
            *pPixel = 0;  
        }  
    }  

    int minDBounds = 0;  
    int maxDBounds = 31;  

    cvNamedWindow("Left",1);  
    cvNamedWindow("Right",1);  
    cvNamedWindow("Depth",1);  
    cvNamedWindow("MatchLevel",1);  

    cvShowImage("Left",leftImage);  
    cvShowImage("Right",rightImage);  

    int minD = 0;  
    int maxD = 31;  
    //假设图像是经过矫正的，那么每次都只是需要搜搜同一行的内容   
    int max12Diff = 10;  

    for (int i = 0;i < imageWidth;i++)  
    {  
        Ddynamic[0][i] = 0;  
        Ddynamic[i][0] = 0;  
    }  

    unsigned char * pLeftPixel  = NULL;  
    unsigned char * pRightPixel = NULL;  
    unsigned char leftPixel = 0;  
    unsigned char rightPixel =0;  
    int m,n,l;  

    for (int i = 0 ; i < imageHeight;i++)  
    {  
        for (int j = 0; j<imageWidth;j++)  
        {  
            for (int k = j + minD; k <= j + maxD;k++)  
            {  
                if (k <0 || k >= imageWidth)  
                {  

                }else {  
                    pLeftPixel = (unsigned char*)leftImage->imageData + i*leftImage->widthStep + k;  
                    pRightPixel= (unsigned char*)rightImage->imageData+i*rightImage->widthStep + j;  
                    leftPixel  = *pLeftPixel;  
                    rightPixel = *pRightPixel;  

                    if (abs(leftPixel - rightPixel) <= max12Diff)  
                    {  
                        Ddynamic[j + 1][k + 1] = Ddynamic[j][k] +1;   
                    }else if (Ddynamic[j][k+1] > Ddynamic[j+1][k])  
                    {  
                        Ddynamic[j + 1][k + 1] = Ddynamic[j][k+1];  
                    }else{  
                        Ddynamic[j+1][k+1] = Ddynamic[j+1][k];  
                    }  

                    //cout<<Ddynamic[j +1][k+1]<<"  ";   
                }  

            }  
             //cout<<"\n";   
        }  
        //逆向搜索，找出最佳路径   
         m = imageWidth;  
         n = imageWidth;  
         l = Ddynamic[imageWidth][imageWidth];  
        while( l>0 )  
        {  
            if (Ddynamic[m][n] == Ddynamic[m-1][n])    
                m--;  
            else if (Ddynamic[m][n] == Ddynamic[m][n-1])    
                n--;  
            else  
            {   
                //s[--l]=a[i-1];   
                pPixel = (unsigned char *)DPImage->imageData + i*DPImage->widthStep + m;  
                *pPixel = (n-m)*8;  
                l--;  
                m--;   
                n--;  
            }  
        }  

       //cvWaitKey(0);   

    }  

    cvShowImage("Depth",DPImage);  
    cvSaveImage("depth.jpg",DPImage);  
    cvWaitKey(0);  
    return 0;  
}  

// DP_5算法：
//引入概率公式   
//   
#include <cstdio>   
#include <cstring>   
#include <iostream>   
#include<cv.h>   
#include<highgui.h>   
#include <cmath>   

using namespace std;  
const int Width =  1024;  
const int Height = 1024;  
 int Ddynamic[Width][Width];  

 //使用钟形曲线作为匹配概率，差值越小则匹配的概率越大，最终的要求是使匹配的概率最大，概率曲线使用matlab生成   
 int Probability[256] = {  
    255, 255, 254, 252, 250, 247, 244, 240, 235, 230, 225, 219, 213, 206, 200, 192, 185, 178, 170, 162,   
    155, 147, 139, 132, 124, 117, 110, 103, 96, 89, 83, 77, 71, 65, 60, 55, 50, 46, 42, 38, 35, 31, 28,   
    25, 23, 20, 18, 16, 14, 13, 11, 10, 9, 8, 7, 6, 5, 4, 4, 3, 3, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 0, 0,   
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0  
 };  

int  _DP5main()  
{    

    IplImage * leftImage = cvLoadImage("l2.jpg",0);  
    IplImage * rightImage = cvLoadImage("r2.jpg",0);  

    //IplImage * leftImage = cvLoadImage("left.bmp",0);                              
    //IplImage * rightImage = cvLoadImage("right.bmp",0);   

    int imageWidth = leftImage->width;  
    int imageHeight =leftImage->height;  

    IplImage * DPImage = cvCreateImage(cvGetSize(leftImage),leftImage->depth,1);  
    IplImage * effectiveImage = cvCreateImage(cvGetSize(leftImage),leftImage->depth,1);  
    IplImage * FilterImage = cvCreateImage(cvGetSize(leftImage),leftImage->depth,1);  

    unsigned char * pPixel = NULL;  
    unsigned char  pixel;  
    unsigned char * pPixel2 = NULL;  
    unsigned char  pixel2;  
    for (int i = 0; i< imageHeight;i++)  
    {  
        for (int j =0; j < imageWidth;j++ )  
        {  
            pPixel = (unsigned char *)DPImage->imageData + i*DPImage->widthStep + j;  
            *pPixel = 0;  
            pPixel = (unsigned char *)effectiveImage->imageData + i*effectiveImage->widthStep + j;  
            *pPixel = 0;  
        }  
    }  



    cvNamedWindow("Left",1);  
    cvNamedWindow("Right",1);  
    cvNamedWindow("Depth",1);  
    cvNamedWindow("effectiveImage",1);  

    cvShowImage("Left",leftImage);  
    cvShowImage("Right",rightImage);  

    int minD = 0;  
    int maxD = 31;  
    //假设图像是经过矫正的，那么每次都只是需要搜搜同一行的内容   
    int max12Diff = 5;  

    for (int i = 0;i < imageWidth;i++)  
    {  
        Ddynamic[0][i] = 0;  
        Ddynamic[i][0] = 0;  
    }  

    unsigned char * pLeftPixel  = NULL;  
    unsigned char * pRightPixel = NULL;  
    unsigned char leftPixel = 0;  
    unsigned char rightPixel =0;  
    int m,n,l;  

    int t1 = clock();  
    for (int i = 0 ; i < imageHeight;i++)  
    {  
        for (int j = 0; j<imageWidth;j++)  
        {  
            for (int k = j + minD; k <= j + maxD;k++)  
            {  
                if (k <0 || k >= imageWidth)  
                {  

                }else {  
                    pLeftPixel = (unsigned char*)leftImage->imageData + i*leftImage->widthStep + k;  
                    pRightPixel= (unsigned char*)rightImage->imageData+i*rightImage->widthStep + j;  
                    leftPixel  = *pLeftPixel;  
                    rightPixel = *pRightPixel;  
                    //之前概率最大的点加上当前的概率   

                    Ddynamic[j + 1][k + 1] = max(Ddynamic[j][k],max(Ddynamic[j][k+1],Ddynamic[j+1][k]))  
                                             + Probability[abs(leftPixel - rightPixel)];  
                    /* if (abs(leftPixel - rightPixel) <= max12Diff) 
                    { 
                    Ddynamic[j + 1][k + 1] = Ddynamic[j][k] +1;  
                    }else if (Ddynamic[j][k+1] > Ddynamic[j+1][k]) 
                    { 
                    Ddynamic[j + 1][k + 1] = Ddynamic[j][k+1]; 
                    }else{ 
                    Ddynamic[j+1][k+1] = Ddynamic[j+1][k]; 
                    }*/  

                    //cout<<Ddynamic[j +1][k+1]<<"  ";   
                }  

            }  
             //cout<<"\n";   
        }  
        //逆向搜索，找出最佳路径   
         m = imageWidth;  
         n = imageWidth;  
         l = Ddynamic[imageWidth][imageWidth];  
        while( m >= 1 && n >= 1)  
        {  
            pPixel = (unsigned char *)DPImage->imageData + i*DPImage->widthStep + m;  
            *pPixel = (n-m)*8;  
            //标记有效匹配点   
            pPixel = (unsigned char *)effectiveImage->imageData + i*effectiveImage->widthStep + m;  
            *pPixel = 255;  
            if (Ddynamic[m-1][n] >= Ddynamic[m][n -1] && Ddynamic[m-1][n] >= Ddynamic[m-1][n -1])    
                m--;  
            else if (Ddynamic[m][n-1] >= Ddynamic[m-1][n] && Ddynamic[m][n -1] >= Ddynamic[m-1][n -1])    
                n--;  
            else  
            {   
                //s[--l]=a[i-1];          
               // l -= Ddynamic[m][n];   
                m--;   
                n--;  
            }  
        }  

       //cvWaitKey(0);   
    }  

    //refine the depth image  7*7中值滤波   
    //统计未能匹配点的个数   
    int count = 0;  
    for (int i = 0 ;i< imageHeight;i++)  
    {  
        for (int j= 0; j< imageWidth;j++)  
        {  
            pPixel = (unsigned char *)effectiveImage->imageData + i*effectiveImage->widthStep + j;  
            pixel = *pPixel;  
            if (pixel == 0)  
            {  
                count++;  
            }  
        }  
    }  
    int t2 = clock();  
    cout<<"dt: "<<t2-t1<<endl;  
    cout<<"Count:  "<<count<<"  "<<(double)count/(imageWidth*imageHeight)<<endl;  
    cvShowImage("Depth",DPImage);  
    cvShowImage("effectiveImage",effectiveImage);  
   // cvWaitKey(0);   


    FilterImage = cvCloneImage(DPImage);  

    //7*7中值滤波   
    int halfMedianWindowSize = 3;  
    int medianWindowSize = 2*halfMedianWindowSize + 1;  
    int medianArray[100] = {0};  
    count = 0;  
    int temp = 0;  
    int medianVal = 0;  

    for (int i = halfMedianWindowSize + 1 ;i< imageHeight - halfMedianWindowSize;i++)  
    {  
        for (int j = halfMedianWindowSize; j< imageWidth - halfMedianWindowSize;j++)  
        {  
            pPixel = (unsigned char *)effectiveImage->imageData + i*effectiveImage->widthStep + j;  
            pixel = *pPixel;  
            if (pixel == 0)  
            {  
                count = 0;  
                for (int m = i - halfMedianWindowSize ; m <= i + halfMedianWindowSize ;m++)  
                {  
                    for (int n = j - halfMedianWindowSize; n <= j + halfMedianWindowSize ;n++)  
                    {  
                        pPixel2 = (unsigned char *)DPImage->imageData + m*DPImage->widthStep + n;  
                        pixel2 = *pPixel2;  
                        if (pixel2 != 0)  
                        {  
                             medianArray[count] = pixel2;  
                             count++;  
                        }  

                    }  
                    //排序   
                    for (int k = 0; k< count;k++)  
                    {  
                        for (int l = k + 1; l< count;l++)  
                        {  
                            if (medianArray[l] < medianArray[l-1] )  
                            {  
                                temp = medianArray[l];  
                                medianArray[l] = medianArray[l-1];  
                                medianArray[l-1] = temp;  
                            }  
                        }  
                    }  
                    medianVal = medianArray[count/2];  
                    pPixel = (unsigned char *)FilterImage->imageData + i*DPImage->widthStep + j;  
                    *pPixel = medianVal;  
                }  

            }  
        }  
    }  
    cvShowImage("Depth",DPImage);  
    cvShowImage("effectiveImage",effectiveImage);  
    cvShowImage("Filter",FilterImage);  
    cvSaveImage("depth.jpg",DPImage);  
    cvSaveImage("effective.jpg",effectiveImage);  

    cvWaitKey(0);  
    return 0;  
}  
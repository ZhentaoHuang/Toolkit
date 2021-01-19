#ifndef IMGPROC_H
#define IMGPROC_H




#include <limits.h> /* PATH_MAX */
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <fstream>
#include<iostream>
#include <random>
#include <algorithm>
#include <math.h>

//#include <cv.h>
#include <opencv2/opencv.hpp>


#include <vector>
#include <ctime>
//#include "timer.h"
//#include "easylogging.h"



//using namespace Eigen;
//string to_string(T value);
namespace speedbot2d{

  typedef enum {
   AUTO_THRESHOLD_HUANG,      
   AUTO_THRESHOLD_INTERMODES, 
   AUTO_THRESHOLD_ISODATA,    
   AUTO_THRESHOLD_MEAN,       
   AUTO_THRESHOLD_OTSU,       
   AUTO_THRESHOLD_TRIANGLE    
 } AutoThresholdMethod;

cv::Mat AdjustBrightness(cv::Mat in_image, float alpha, float beta);
cv::Mat GammaCorrection(const cv::Mat in_image, float gamma);
cv::Mat ReadImg();
int WriteImg(cv::Mat in_image);
cv::Mat StretchContrast(const cv::Mat in_image);
cv::Mat GetHistogram(const cv::Mat& image);
int ComputeThresholdHuang(cv::Mat hist);
cv::Mat AutoThresholding(const cv::Mat in_image, AutoThresholdMethod method);
int EdgeDetectionLaplacian(const cv::Mat in_image, cv::Mat &out_image);
}



#endif

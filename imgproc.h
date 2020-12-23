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

cv::Mat AdjustBrightness(cv::Mat in_image, float alpha, float beta);
cv::Mat GammaCorrection(const cv::Mat in_image, float gamma);
cv::Mat ReadImg();
int WriteImg(cv::Mat in_image);
cv::Mat StretchContrast(const cv::Mat in_image);
cv::Mat GetHistogram(const cv::Mat& image);
int ComputeThresholdHuang(cv::Mat hist);

}



#endif

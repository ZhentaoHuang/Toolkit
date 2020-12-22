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

//#include <cv.h>
#include <opencv2/opencv.hpp>


#include <vector>
#include <ctime>
//#include "timer.h"
//#include "easylogging.h"


using namespace std;
//using namespace Eigen;
//string to_string(T value);
cv::Mat adjustBrightness(cv::Mat in_image, float alpha, float beta);
cv::Mat gammaCorrection(const cv::Mat in_image, float gamma);
cv::Mat readImg();





#endif

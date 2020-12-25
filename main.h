#ifndef MAIN_H
#define MAIN_H




#include <limits.h> /* PATH_MAX */
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <fstream>
#include<iostream>
#include <random>
#include <algorithm>

#include <opencv2/opencv.hpp>


#include <vector>
#include <ctime>


std::vector<int>ActBoxRange_all;
std::vector<std::vector<int>> Ranges_all;
int maduo_placed_num_all = 0;


//cv::Mat in_image, out_image;
cv::Mat M1;
std::map<std::string, cv::Mat> map_cvmat;
int AdjustBrightness_count = 0;
int GammaCorrection_count = 0;
int ReadImg_count = 0;
int WriteImg_count = 0;

#endif

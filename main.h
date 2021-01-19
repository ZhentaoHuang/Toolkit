#ifndef MAIN_H
#define MAIN_H

#include "threedproc.h"
#include "httplib.h"
#include "imgproc.h"
#include "json.hpp"
#include <map>
#include "CPlanning_box_davit.cpp"
#include "CPlanning_box_davit.h"

using json = nlohmann::json;




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
std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> map_plyptr;
std::map<std::string, GAPacking::boxinfo>map_boxinfo;

int xx = 113;
int yy = 95;
int zz = 100;
GAPacking::CPlanning_Box PlanningBox(xx,yy,zz);

int AdjustBrightness_count = 0;
int GammaCorrection_count = 0;
int ReadImg_count = 0;
int WriteImg_count = 0;
int ReadPLY_count = 0;
int DownSampling_count = 0;
int StatisticalOutlierRemoval_count = 0;
int PassFilter_count = 0;
int ChangeDetector_count = 0;
int PlacementPlanning_count = 0;

#endif

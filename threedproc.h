#ifndef THREEDPROC_H
#define THREEDPROC_H




#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/sac_segmentation.h>



#include <pcl/console/parse.h>
#include <limits.h> /* PATH_MAX */
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <fstream>
#include<iostream>
#include <random>
#include <algorithm>

#include <cv.h>


#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/correspondence.h>
#include <pcl/common/geometry.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/point_types.h>
//#include<Eigen/Eigen>
//#include<Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/octree/octree.h>
#include <vector>
#include <ctime>
//#include "timer.h"
//#include "easylogging.h"

typedef pcl::PointXYZ PointT;


//过滤
#define LeafSize 0.005f*1000 //下采样参数  down_sample
#define FilterSize 100 //滤波考虑最邻点数量  s_filter
#define FilterThres 0.5 //滤波阈值倍数  s_filter


#define kdtree_number  80// 判断kdtree的数量
#define kdtree_search_radius 3  //判断kdtree搜索半径

//get_big_matrix
#define height_matrix_row 93
#define height_matrix_col 112
#define height_matrix_z 110

//欧式聚类对象
#define ECDist 5// 设置近邻搜索的搜索半径为20mm
#define ECClusMin 16*100 //设置一个聚类需要的最少的点数目为100
#define ECClusMax 16*10000 //设置一个聚类需要的最大点数目为25000



namespace speedbot3d{

    int ReadPLY(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string filename);
    
    int DownSampling(   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input, 
                        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output, 
                        float leaf_size, 
                        bool save=false, 
                        std::string filename="Down_Sampling");

    int StatisticalOutlierRemoval(	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input, 
								pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output, 
								int mean_k, 
								float threshold, 
								bool save=false, 
								std::string filename="StatisticalOutlierRemoval");

    int PassFilter(	pcl::PointCloud<PointT>::Ptr cloud_input, 
					pcl::PointCloud<PointT>::Ptr cloud_output, 
					double x_min,
					double x_max,
					double y_min,
					double y_max,
					double z_min,
					double z_max, 
					bool negative=false,
					bool save=false,
					std::string filename="PassFilter");

		int ChangeDetector(	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ori, 
					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_new, 
					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_change, 
					float resolution, 
					bool save=false, 
					std::string filename="ChangeDetector");
    
}




#endif

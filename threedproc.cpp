/*
 * Author:zhentaohuang
 * Email: zhentaohuang222@gmail.com
 */
//#include "timer.h"
#include "threedproc.h"

//#define PI 3.1415926
//using namespace speedbot;

namespace speedbot3d{


int ReadPLY (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string filename) {
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PLYReader plyReader;
    if (plyReader.read("../data/"+ filename +".ply", *cloud) == -1) {
		PCL_ERROR("Couldn't read file\n");
		return -1;
	}

	return 0;

}




/**
 * @brief 对于点云进行下采样处理，提升运行速率
 * 
 * @param cloud_input 输入点云
 * @param cloud_output 输出点云
 * @param leaf_size 下采样参数
 * @param save 是否将输出保存在本地，默认为否
 * @param filename 保存的名字，默认为函数名
 * @return int 
 */
int DownSampling(	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input, 
					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output, 
					float leaf_size, 
					bool save, 
					std::string filename)
{
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud (cloud_input);
	sor.setLeafSize (leaf_size, leaf_size, leaf_size);
	sor.filter (*cloud_output);

	
	if(save)
	{
		pcl::io::savePLYFileBinary("../data/" + filename + ".ply", *cloud_output);    
	}

	return 0;
}



/**
 * @brief 对点云进行基于统计滤波器的去噪处理
 * 
 * @param cloud_input 输入点云
 * @param cloud_output 输出点云
 * @param mean_k 考虑的周围点的数量
 * @param threshold 滤波器阈值
 * @param save 是否将输出保存在本地，默认为否
 * @param filename 保存的名字，默认为函数名
 * @return int 
 */
int StatisticalOutlierRemoval(	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input, 
								pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output, 
								int mean_k, 
								float threshold, 
								bool save, 
								std::string filename)
{
	
	pcl::StatisticalOutlierRemoval <pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_input);
	sor.setMeanK(mean_k);
	sor.setStddevMulThresh(threshold);
	sor.filter(*cloud_output);

	if(save)
	{
		pcl::io::savePLYFileBinary("../data/" + filename + ".ply", *cloud_output);    
	}


	return 0;
}

/**
 * @brief 直通滤波器，传入xyz的范围进行滤波
 * 
 * @param cloud_input 输入点云
 * @param cloud_output 输出点云
 * @param x_min x轴最小值
 * @param x_max x轴最大值
 * @param y_min y轴最小值
 * @param y_max y轴最大值
 * @param z_min z轴最小值
 * @param z_max z轴最大值
 * @param negative 是保留范围内的点云还是范围外的点云，默认为负值保留范围内
 * @param save 是否将输出保存在本地，默认为否
 * @param filename 保存的名字，默认为函数名
 * @return int 
 */
int PassFilter(	pcl::PointCloud<PointT>::Ptr cloud_input, 
					pcl::PointCloud<PointT>::Ptr cloud_output, 
					double x_min,
					double x_max,
					double y_min,
					double y_max,
					double z_min,
					double z_max, 
					bool negative,
					bool save,
					std::string filename)
{//TODO: x_max > x_min ?
	pcl::PassThrough<PointT> pass;

	pass.setInputCloud(cloud_input);            //设置输入点云
	pass.setFilterFieldName("x");         //设置过滤时所需要点云类型的Z字段
	pass.setFilterLimits(x_min, x_max);       //设置在过滤字段的范围
	pass.setNegative(negative);
	pass.filter(*cloud_output);

	pass.setInputCloud(cloud_output);
	pass.setFilterFieldName("y");         //设置过滤时所需要点云类型的Z字段
	pass.setFilterLimits(y_min, y_max);      //设置在过滤字段的范围
	pass.filter(*cloud_output);

	pass.setInputCloud(cloud_output);
	pass.setFilterFieldName("z");         //设置过滤时所需要点云类型的Z字段
	pass.setFilterLimits(z_min, z_max);       //设置在过滤字段的范围
	pass.filter(*cloud_output);    

	if(save)
	{
		pcl::io::savePLYFileBinary("../data/" + filename + ".ply", *cloud_output);    
	}

	return 0;
}



/**
 * @brief 比较传入的cloud_new较cloud_ori新增的点云并存入cloud_change
 * 
 * @param cloud_ori 
 * @param cloud_new 
 * @param cloud_change 
 * @param resolution 
 * @param save 
 * @param filename 
 * @return int 
 */
int ChangeDetector(	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ori, 
					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_new, 
					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_change, 
					float resolution, 
					bool save, 
					std::string filename)
{
	//实例化基于octree的点云检测类
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree(resolution);//创建检测类的对象

    //add cloud_point from cloud to octree
    octree.setInputCloud(cloud_ori);//输入点云cloud1
    octree.addPointsFromInputCloud();//从输入点云cloud1构建八叉树
 
    
    octree.switchBuffers();//交换八叉树缓存，但是cloud_ori对应的八叉树结构仍在内存中
  
    //add points from cloud1 to octree
    octree.setInputCloud(cloud_new);//makeShared()返回一个指针
    octree.addPointsFromInputCloud();//从输入点云cloud2构建八叉树
 
    std::vector<int> newPointIdxVector;//存储新加点的索引的向量
    octree.getPointIndicesFromNewVoxels(newPointIdxVector);//获取新增点的索引
    //将新增点的放到cloud_result所指向的内存中
    for(size_t i = 0;i < newPointIdxVector.size();++i)
    {
		pcl::PointXYZ tmppoint;
        tmppoint.x = cloud_new->points[newPointIdxVector[i]].x;
        tmppoint.y = cloud_new->points[newPointIdxVector[i]].y;
        tmppoint.z = cloud_new->points[newPointIdxVector[i]].z;

		cloud_change->points.push_back(tmppoint);
    }
	if(save)
	{
		pcl::io::savePLYFileBinary("../data/" +filename+ ".ply", *cloud_change);
	}

	return 0;
}

/**
 * @brief 将cloud_a和cloud_b合并为cloud_result
 * 
 * @param cloud_a 
 * @param cloud_b 
 * @param cloud_result 
 * @param save 
 * @param filename 
 */
void CombinePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_a, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_b, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_result, bool save, std::string filename)
{
	*cloud_result = *cloud_a + *cloud_b;
	if(save)
	{
		//保存文件
		pcl::io::savePLYFileBinary("../data/" +filename+ ".ply", *cloud_result);
   // std::cout<<filename<<".ply saved"<< cloud_result->size()<< "points"<<endl;
	}
	
}










}
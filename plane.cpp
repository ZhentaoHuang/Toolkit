/*
 * Author:zhentaohuang
 * Email: zhentaohuang222@gmail.com
 */
//#include "timer.h"
#include "plane.h"
#include "httplib.h"
#define PI 3.1415926
//using namespace speedbot;


//过滤离群点
int InitialFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	
	pcl::StatisticalOutlierRemoval <pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(FilterSize);
	sor.setStddevMulThresh(FilterThres);
	sor.filter(*cloud);

	return 0;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr cvMatToPcl(cv::Mat &mat) 
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(
			new pcl::PointCloud<pcl::PointXYZ>);
	
	std::cout << "begin parsing file" << std::endl;
	for (int ki = 0; ki < mat.rows; ki++) {
		for (int kj = 0; kj < mat.cols; kj++) {
			pcl::PointXYZ pointXYZ;

			pointXYZ.x = mat.at<cv::Point3f>(ki, kj).x;
			pointXYZ.y = mat.at<cv::Point3f>(ki, kj).y;
			pointXYZ.z = mat.at<cv::Point3f>(ki, kj).z;

			if(pointXYZ.z <= 0)
				continue;
			cloud->points.push_back(pointXYZ);
		}

	}
	return cloud;
}


//根据传入的参数 滤除笼车，保留箱子的点云
void PassFilter(pcl::PointCloud<PointT>::Ptr cloud_input, pcl::PointCloud<PointT>::Ptr cloud_output, double xyzMinMax[], bool negative)
{
	pcl::PassThrough<PointT> pass;

		pass.setInputCloud(cloud_input);            //设置输入点云
		pass.setFilterFieldName("x");         //设置过滤时所需要点云类型的Z字段
		pass.setFilterLimits(xyzMinMax[0], xyzMinMax[1]);       //设置在过滤字段的范围
		pass.setNegative(negative);
    pass.filter(*cloud_output);

		pass.setInputCloud(cloud_output);
		pass.setFilterFieldName("y");         //设置过滤时所需要点云类型的Z字段
		pass.setFilterLimits(xyzMinMax[2], xyzMinMax[3]);      //设置在过滤字段的范围
		//pass.setNegative(negative);
		pass.filter(*cloud_output);
	
		pass.setInputCloud(cloud_output);
		pass.setFilterFieldName("z");         //设置过滤时所需要点云类型的Z字段
		pass.setFilterLimits(xyzMinMax[4], xyzMinMax[5]);       //设置在过滤字段的范围
		//pass.setNegative(negative);
		pass.filter(*cloud_output);    

	
}

void DownSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output, bool save, string filename)
{
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud (cloud_input);
	sor.setLeafSize (LeafSize, LeafSize, LeafSize);
	sor.filter (*cloud_output);
	cout<<"re_cloud downsample finish"<<endl;
	
	if(save)
	{
		pcl::io::savePLYFileBinary("../data/" + filename + ".ply", *cloud_output);    
	}

}

bool  kdtree_radius(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointXYZ searchPoint,float radius){
	pcl::KdTreeFLANN<pcl::PointXYZ>kdtree;//创建kd_tree对象 
	kdtree.setInputCloud (cloud);//设置搜索空间 
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) >0 )
	{
		// for (size_t i=0; i<pointIdxRadiusSearch.size (); ++i)
		if(pointIdxRadiusSearch.size ()>kdtree_number){
			return true;
		}else{
		//	cout<<"size:  "<<pointIdxRadiusSearch.size ()<<endl;
			return false;
		}

	}
	else{
		return false;
	}
}





void GetHeightMatrix(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1,MatrixXf &big_matrix, double minx, double miny){
	// big_matrix(big_matrix_row,big_matrix_col);
	for(int i=0;i< big_matrix.rows();i++){
		for(int j=0;j< big_matrix.cols();j++){
			big_matrix(i,j)=-1;
		}
	}

	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PLYReader plyReader;
    //plyReader.read("../data/testcam2robot.ply", *cloud1);
	

	for(int i=0;i<cloud1->points.size();i++){
		cloud1->points[i].x=cloud1->points[i].x/10;
		cloud1->points[i].y=cloud1->points[i].y/10;
		cloud1->points[i].z=cloud1->points[i].z/10;

	}
	//pcl::io::savePLYFileBinary("../data/mm_robot.ply", *cloud);	
	pcl::io::savePLYFileBinary("../data/cm_robot.ply", *cloud1);	

	double xyz[6];
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
	for(int i=0;i< big_matrix.rows();i++){
		for(int j=0;j< big_matrix.cols();j++){
			
			xyz[0]= i + minx;xyz[1]= i + 1 + minx;xyz[2]= j + miny;xyz[3]= j + 1 + miny;xyz[4]=0;xyz[5]=height_matrix_z;
			PassFilter(cloud1, cloud2, xyz, false);
			if(cloud2->points.size()>0){
				
				pcl::PointXYZ minPt, maxPt;
				pcl::getMinMax3D (*cloud2, minPt, maxPt);	

				//if(kdtree_radius(cloud1,maxPt,kdtree_search_radius)){
				big_matrix(i,j)=maxPt.z;
				//}
				

			}
			
		}
	}
}




void GetDepthcloud(MatrixXf big_matrix, pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_depth, bool savezero, bool savemone, double minx, double miny){
		for(int i=0;i< big_matrix.rows();i++){
			for(int j=0;j< big_matrix.cols();j++){
				if((big_matrix(i,j) == 0 && !savezero) ||(big_matrix(i,j)==-1 && !savemone))
				{
					continue;
				}

				for(double k=0;k<=1;k=k+0.5){
					for(double q=0;q<=1;q=q+0.5){
							pcl::PointXYZ pointXYZ;
							pointXYZ.x = k+i+minx;
							pointXYZ.y = q+j+miny;
							pointXYZ.z =big_matrix(i,j) ;
							cloud_depth->points.push_back(pointXYZ);
						


					}
				}
				
			}
	}
}

void eucl_clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, vector <vector<int>> &Ranges,  bool save)
{
	int plane_count = 0;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	//pcl::PLYWriter writer;
	tree->setInputCloud (cloud_filtered);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;   //欧式聚类对象
	ec.setClusterTolerance (ECDist);                     // 设置近邻搜索的搜索半径为2cm
	ec.setMinClusterSize (ECClusMin);                 //设置一个聚类需要的最少的点数目为100
	ec.setMaxClusterSize (ECClusMax);               //设置一个聚类需要的最大点数目为25000
	ec.setSearchMethod (tree);                    //设置点云的搜索机制
	ec.setInputCloud (cloud_filtered);
	ec.extract (cluster_indices);           //从点云中提取聚类，并将点云索引保存在cluster_indices中
	//迭代访问点云索引cluster_indices,直到分割处所有聚类
	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{ //迭代容器中的点云的索引，并且分开保存索引的点云
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
		{
			//设置保存点云的属性问题
			cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
		}
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		//jisuan
		vector<int>tmpx;
		vector<int>tmpy;
		vector<int>tmpz;
		tmpx.clear();
		tmpy.clear();
		tmpz.clear();



		for(auto& point : *cloud_cluster)
		{
			tmpx.push_back(point.x);
			tmpy.push_back(point.y);
			tmpz.push_back(point.z);
		}

		sort(tmpx.begin(), tmpx.end());
		sort(tmpy.begin(), tmpy.end());
		sort(tmpz.begin(), tmpz.end());
		vector<int>tmpxyz;
		tmpxyz.push_back(tmpx[0]);
		tmpxyz.push_back(tmpx[tmpx.size()-1]);
		tmpxyz.push_back(tmpy[0]);
		tmpxyz.push_back(tmpy[tmpy.size()-1]);
		tmpxyz.push_back(tmpz[0]);
		tmpxyz.push_back(tmpz[tmpz.size()-1]);

		Ranges.push_back(tmpxyz);
		


		if(save)
		{
			//pcl::io::savePLYFileBinary("../data/plane/"+ to_string(plane_count) +".ply", *output);
			std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
			std::stringstream ss;
			ss << "../data/plane/" << plane_count << ".ply";
			//writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false);
			pcl::io::savePLYFileBinary(ss.str(), *cloud_cluster);
		}
		plane_count ++;



		
		
		j++;
	}
}


void MedChange(MatrixXf &change_matrix, int row, int col)
{
	vector<double> tmp;
  
	for(int i = 0; i < 5; i++)
	{
		for(int j = 0; j < 5; j++)
		{
			int tmprow = row + i;
			int tmpcol = row + j;
			if(tmprow >= 0 && tmprow < change_matrix.rows() && tmpcol >=0 && tmpcol < change_matrix.cols() && change_matrix(tmprow,tmpcol)>0)
			{
				tmp.push_back(change_matrix(tmprow, tmpcol));		
			}
				

		}
	}
	if(tmp.size() == 0)
	{
		//cout<<"size 0 !!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
	}
	else
	{
		sort(tmp.begin(), tmp.end());

		if(tmp.size() % 2 == 1)
		{
			change_matrix(row, col) = tmp[tmp.size()/2];
		}
		else
		{
			change_matrix(row, col) = tmp[tmp.size()/2];// + tmp[tmp.size()/2 -1])/2;
		}
	}
	

}


void MedianFilter(MatrixXf &change_matrix)
{
	//中值滤波
	for(int i = 0; i < change_matrix.rows(); i++)
	{
		for(int j = 0; j < change_matrix.cols(); j++)
		{
			MedChange(change_matrix, i, j);


		}

	}
}


void TransToRobot(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, string filepath ,bool save, string filename)
{
	if(cloud->points.size() ==0)
	{
		cout<<"wrong file!"<<endl;
		return;
	}

	
	std::ifstream in(filepath);
	double xyzMinMax[6];
	double R[7];double robotT[3];

	in>>R[0];in>>R[1];in>>R[2];
	in>>R[3];in>>R[4];in>>R[5];
	in>>R[6];in>>robotT[0]; in>>robotT[1]; in>>robotT[2];
	in>>xyzMinMax[0];
	in>>xyzMinMax[1];
	in>>xyzMinMax[2];
	in>>xyzMinMax[3];
	in>>xyzMinMax[4];
	in>>xyzMinMax[5];
	Eigen::Quaterniond q;  
    q.x() = R[3];  
    q.y() = R[4];  
    q.z() = R[5];  
    q.w() = R[6];  
  
    Eigen::Matrix3d RotationMat = q.normalized().toRotationMatrix();  
    cout << "Quaternion2RotationMatrix result is:" <<endl;  
    cout << "R = " << endl << RotationMat << endl<< endl; 

	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 RT;
	RT<<RotationMat(0,0), RotationMat(0,1), RotationMat(0,2), R[0]*1000 - robotT[0],
	RotationMat(1,0), RotationMat(1,1), RotationMat(1,2), R[1]*1000 - robotT[1],
	RotationMat(2,0), RotationMat(2,1), RotationMat(2,2), R[2]*1000 - robotT[2],
	0, 0, 0, 1;
	cout << "RT = " << endl << RT << endl<< endl; 

	pcl::transformPointCloud(*cloud, *cloud, RT);

	PassFilter(cloud, cloud, xyzMinMax, false);

	InitialFilter(cloud);

	//LOG(INFO)<<"坐标转换耗时: "<< time_cal.Stop();         //hk_1022: 记录耗时
	if(save)
	{
	    pcl::io::savePLYFileBinary("../data/" + filename + ".ply", *cloud);	
		cout<<"save ok"<<endl;
	}
}

//比较传入的cloud_new较cloud_ori新增的点云并存入cloud_change
void ChangeDetector(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ori, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_new, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_change, float resolution, bool save, string filename)
{
	//实例化基于octree的点云检测类
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree(resolution);//创建检测类的对象

    //add cloud_point from cloud to octree
    octree.setInputCloud(cloud_ori);//输入点云cloud1
    octree.addPointsFromInputCloud();//从输入点云cloud1构建八叉树
 
    //Switch octree buffers: This resets octree but keeps previous tree structure in memory.
    octree.switchBuffers();//交换八叉树缓存，但是cloud1对应的八叉树结构仍在内存中
  
    //add points from cloud1 to octree
    octree.setInputCloud(cloud_new);//makeShared()返回一个指针
    octree.addPointsFromInputCloud();//从输入点云cloud2构建八叉树
 
    std::vector<int> newPointIdxVector;//存储新加点的索引的向量
    //Get vector of point indices from octree voxels which did not exist in previous buffer
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
}

void RemoveOutlier(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool save, string filename)
{
	//对变化的点云进行一系列去噪处理
	pcl::StatisticalOutlierRemoval <pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(50);
	sor.setStddevMulThresh(1);
	sor.filter(*cloud);
 	//std::cout<<"result.ply 1 saved"<< cloud->size()<< "points"<<endl;
	pcl::RadiusOutlierRemoval<pcl::PointXYZ> pcFilter;
	pcFilter.setInputCloud(cloud);
	pcFilter.setRadiusSearch(150);
	pcFilter.setMinNeighborsInRadius(300);
	pcFilter.filter(*cloud);
  	//std::cout<<"result.ply 2 saved"<< cloud->size()<< "points"<<endl;
	//pcl::RadiusOutlierRemoval<pcl::PointXYZ> pcFilter;
	pcFilter.setInputCloud(cloud);
	pcFilter.setRadiusSearch(20);
	pcFilter.setMinNeighborsInRadius(10);
	pcFilter.filter(*cloud);

	if(save)
	{
		pcl::io::savePLYFileBinary("../data/" +filename+ ".ply", *cloud);
	}
}

void ExtractBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inter, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outer, vector<int> box)
{
	vector<int > indexs;
	int xyzMinMax[6];

	xyzMinMax[0] = 10*(((box[0] - 3) < 0) ?  0 : box[0]-3); //xmin
	xyzMinMax[1] = 10*((box[0] + box[3] + 2) > height_matrix_row ? height_matrix_row : box[0] + box[3] + 2); //xmax
	xyzMinMax[2] = 10*(((box[1] - 3) < 0) ? 0 : box[1]-1); //ymin
	xyzMinMax[3] = 10*((box[1] + box[4] + 2) > height_matrix_col ? height_matrix_col : box[1] + box[4] + 2); //ymax
	xyzMinMax[4] = 10*(box[5] -10); //zmin
	xyzMinMax[5] = 10*(box[5] + 3); //zmax

	int j = 0;
	for (auto i : *cloud)
	{
		if (i.x > xyzMinMax[0] && i.x < xyzMinMax[1] && i.y > xyzMinMax[2] && i.y < xyzMinMax[3] && i.z > xyzMinMax[4] && i.z <xyzMinMax[5])
		{
			cloud_inter->points.push_back(i);
			indexs.push_back(j);
		}
		j++;
	}
	//打印滤波后将法向量存储在normal1的信息，以及相应的索引
	//std::cout << *cloud_inter << std::endl;
	std::cout << indexs.size() << std::endl;

	//索引
	boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(indexs);
	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	// Extract the inliers
	extract.setInputCloud(cloud);
	extract.setIndices(index_ptr);
	extract.setNegative(true);//如果设为true,可以提取指定index之外的点云
	extract.filter(*cloud_outer);
}

void CombinePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_a, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_b, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_result, bool save, string filename)
{
	*cloud_result = *cloud_a + *cloud_b;
	if(save)
	{
		//保存文件
		pcl::io::savePLYFileBinary("../data/" +filename+ ".ply", *cloud_result);
    std::cout<<filename<<".ply saved"<< cloud_result->size()<< "points"<<endl;
	}
	
}


void CheckMatrix(const MatrixXf &cloud_matrix, const MatrixXf &cloud1_matrix, MatrixXf &change_matrix, int row, int col)
{
	int count = 0;
	for(int i = 0; i < row; i++)
	{
		for(int j = 0; j < col; j++)
		{
			if(abs(cloud_matrix(i,j) - cloud1_matrix(i,j)) < 1)
			{
				count ++;
				change_matrix(i,j) = -1;
			}
		}
	}
	cout<<"删除了"<<count<<endl;

}


void UpdateRanges(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, vector <vector<int>> &Ranges)
{

	pcl::PointXYZ minPt, maxPt;
	pcl::getMinMax3D (*cloud, minPt, maxPt);
	vector<int> tmprange;
	tmprange.push_back(minPt.x);
	tmprange.push_back(maxPt.x);
	tmprange.push_back(minPt.y);
	tmprange.push_back(maxPt.y);
	tmprange.push_back(minPt.z);
	tmprange.push_back(maxPt.z);


	Ranges.push_back(tmprange);
}



void PalletDetect(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_last, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_current, vector <vector<int>> &Ranges, string filepath,MatrixXf &big_matrix1, vector<int>box, bool save, int cap_count)
{
	//Timer time_cal;
	//time_cal.Start();
	//pcl::io::savePLYFileBinary("../data/" + to_string(cap_count) + ".ply", *cloud);

	if(cloud_last->points.size() ==0)
	{
		cout<<"wrong file!"<<endl;
		return;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::copyPointCloud(*cloud_last, *cloud);		//复制输入点云，防止更改原始数据
	pcl::copyPointCloud(*cloud_current, *cloud1);

	DownSampling(cloud, cloud, save, "downsample");	//点云下采样
	DownSampling(cloud1, cloud1, save, "downsample1");

	TransToRobot(cloud, filepath, save, "testcam2robot");	//从相机坐标系转换到笼车坐标系
	TransToRobot(cloud1, filepath, save, "testcam2robot1");	

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_box(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_box1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rest(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rest1(new pcl::PointCloud<pcl::PointXYZ>);
	

	if(box.size() != 6)
	{
		cout<<"wrong box range!"<<endl;
		return;
	}
	

	ExtractBox(cloud, cloud_box, cloud_rest, box);		//将输入点云按照摆放箱子的位置进行分割，即箱子为一部分，其余为一部分，方便之后使用不同分辨率构建八叉树
	ExtractBox(cloud1, cloud_box1, cloud_rest1, box);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_result_box(new pcl::PointCloud<pcl::PointXYZ>);	//对箱子部分检测变化并去噪音
	ChangeDetector(cloud_box, cloud_box1, cloud_result_box, 20.0f, save, "changedetect_ori_box");
	RemoveOutlier(cloud_result_box, save, "changedetect_box");
	

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_result_rest(new pcl::PointCloud<pcl::PointXYZ>);	//对箱子以外剩余部分检测变化并去噪音
	ChangeDetector(cloud_rest, cloud_rest1, cloud_result_rest, 80.0f, save, "changedetect_rest_ori");
	RemoveOutlier(cloud_result_rest, save, "changedetect_rest");

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_result(new pcl::PointCloud<pcl::PointXYZ>);		//取两部分的变化的并集
	CombinePointCloud(cloud_result_box, cloud_result_rest, cloud_result, save, "changedetect");
	
		
	if(cloud_result->size() == 0)
	{
		cout<<"no change!"<<endl;
		return;
	}
	else
	{
		
		pcl::PointXYZ minPt, maxPt;
  		pcl::getMinMax3D (*cloud_result, minPt, maxPt);


		//todo： 矩阵坐标转换更新 qsort
		double tmpMinMax[4];
		tmpMinMax[0] = minPt.x;
		tmpMinMax[1] = maxPt.x;
		tmpMinMax[2] = minPt.y;
		tmpMinMax[3] = maxPt.y;
		
		cout<<"Ranges_change: "<<tmpMinMax[0]<<"\t"<<tmpMinMax[1]<<"\t"<<tmpMinMax[2]<<"\t"<<tmpMinMax[3]<<endl;

		int row = ceil(tmpMinMax[1]/10) - floor(tmpMinMax[0]/10);
		int col = ceil(tmpMinMax[3]/10) - floor(tmpMinMax[2]/10);


		//定义变化区域以及前后两帧的高度矩阵		
		MatrixXf change_matrix(row,col);
		MatrixXf cloud_matrix(row, col);
		MatrixXf cloud1_matrix(row, col);

		//获取高度矩阵
		GetHeightMatrix(cloud_result,change_matrix, tmpMinMax[0]/10, tmpMinMax[2]/10);
		GetHeightMatrix(cloud, cloud_matrix, tmpMinMax[0]/10, tmpMinMax[2]/10);
		GetHeightMatrix(cloud1, cloud1_matrix, tmpMinMax[0]/10, tmpMinMax[2]/10);

		//判断，如果单位内变化小于1cm则不考虑
		CheckMatrix(cloud_matrix, cloud1_matrix, change_matrix, row, col);
	
		//对变化区域的高度矩阵进行中值滤波处理
		MedianFilter(change_matrix);
	
	


		//获取变化矩阵的实际点云
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_change(new pcl::PointCloud<pcl::PointXYZ>);
		GetDepthcloud(change_matrix,cloud_change,true, false, tmpMinMax[0]/10, tmpMinMax[2]/10);

	
		pcl::io::savePLYFileBinary("../data/change_mat.ply", *cloud_change);	

		//更新Ranges即输出
		UpdateRanges(cloud_change, Ranges);
		

	}
	
	//LOG(INFO)<<"聚类耗时: "<< time_cal.Stop();         //hk_1022: 记录耗时

	for(int i = 0; i < Ranges.size(); i++)
	{
		cout<<"Ranges: "<<Ranges[i][0]<<"\t"<<Ranges[i][1]<<"\t"<<Ranges[i][2]<<"\t"<<Ranges[i][3]<<"\t"<<Ranges[i][4]<<"\t"<<Ranges[i][5]<<endl;
	}

}

/*
 * Author: Zhentao Huang
 * Email: zhentaohuang222@gmail.com
 */
#include "main.h"

#include "httplib.h"
#include "imgproc.h"
#include "threedproc.h"
//#include "CPlanning_box_davit.cpp"
//#include "CPlanning_box_davit.h"
#include "json.hpp"
#include <map>

using json = nlohmann::json;

#include <iostream>
#include <memory>
#include <thread>
#include <Open3D/Open3D.h>

#include <fstream>
#include <sstream>


//#include "plane.h"

int main(int agrc, char* argv[]) {
	
	
	

	httplib::Server svr;
	
	svr.Get("/run/", [](const httplib::Request& req, httplib::Response& res) {
        json result;

        if (req.has_param("action")) {
					std::string str_action = req.get_param_value("action");
					json action = json::parse(str_action);

					std::string function = action["function"];
					
					transform(function.begin(), function.end(), function.begin(), ::tolower);
					std::cout<<function<<std::endl;

					//AdjustBrightness 调整图片的亮度和对比度
					if (function == "adjustbrightness") {		// 调用的是函数 aaa
            
            auto params = action["params"];									// 取出参数（其实就是个数组，参数是按顺序的，分别是参数一、参数二、参数三。。。）

						std::map<std::string, cv::Mat>::iterator iter;	// 从map中寻找传来的参数所对应的数据
						iter = map_cvmat.find(params[0]);
						
						cv::Mat in;
						if(iter != map_cvmat.end()) {							//如果在map中找到了数据

							in = iter->second;		
							std::cout<<"Running AdjustBrightness with "<<iter->first<<std::endl;

							auto attributes = action["attributes"];	// 取出固定属性，这是一个包含了 key-value 的数组
							std::string s =  attributes["alpha"];		
							float alpha= atof(s.c_str());
							s =  attributes["beta"];
							float beta= atof(s.c_str());

							cv::Mat out = speedbot2d::AdjustBrightness(in, alpha, beta);	//调用函数

							map_cvmat.insert(std::map<std::string, cv::Mat>::value_type("AdjustBrightness_out"+std::to_string(AdjustBrightness_count), out));


							//cv::imwrite("../data/output.png", out);
							std::cout<<"Result: AdjustBrightness_out"+std::to_string(AdjustBrightness_count) <<std::endl;
							
							// 封装执行结果
							result["code"] = 200;   // code-200, 代表执行成功
							result["data"] = { "AdjustBrightness_out"+std::to_string(AdjustBrightness_count) };  
							
							AdjustBrightness_count ++;

						} else {
							std::cout<<"Wrong Params!!!"<<std::endl;
							result["code"] = 300;		// code 300 代表没有找到输入的参数对应数据
							result["msg"] = "Wrong Params";
						}

					//GammaCorrection 图片Gamma矫正
					} else if (function == "GammaCorrection") {

						auto params = action["params"];									// 取出参数（其实就是个数组，参数是按顺序的，分别是参数一、参数二、参数三。。。）

						std::map<std::string, cv::Mat>::iterator iter;	// 从map中寻找传来的参数所对应的数据
						iter = map_cvmat.find(params[0]);
						
						cv::Mat in;
						if(iter != map_cvmat.end()) {

							in = iter->second;		
							std::cout<<"Running GammaCorrection with "<<iter->first<<std::endl;

							auto attributes = action["attributes"];				// 取出固定属性，这是一个包含了 key-value 的数组
							std::string s = attributes["gamma"];
							float gamma= atof(s.c_str());

							cv::Mat out = speedbot2d::GammaCorrection(in, gamma);

							map_cvmat.insert(std::map<std::string, cv::Mat>::value_type("GammaCorrection_out"+std::to_string(GammaCorrection_count), out));

							std::cout<<"Result: GammaCorrection_out"+std::to_string(GammaCorrection_count) <<std::endl;

							
							
							// 封装执行结果
							result["code"] = 200;   // code-200, 代表执行成功
							result["data"] = { "GammaCorrection_out"+std::to_string(GammaCorrection_count) };  

							GammaCorrection_count ++;

						} else {
							std::cout<<"Wrong Params!!!"<<std::endl;
							result["code"] = 300;		// code 300 代表没有找到输入的参数对应数据
							result["msg"] = "Wrong Params";
						}

					//ReadImg
        	} else if (function == "readimg") {

						auto params = action["params"];

						cv::Mat ReadImg_in = speedbot2d::ReadImg();
						//test
						//speedbot2d::EdgeDetectionLaplacian(ReadImg_in, ReadImg_in);
						//
						map_cvmat.insert(std::map<std::string, cv::Mat>::value_type("ReadImg_out"+std::to_string(ReadImg_count), ReadImg_in));

						std::cout<<"Result: ReadImg_out"+std::to_string(ReadImg_count) <<std::endl;
						
						// 封装执行结果
						result["code"] = 200;   // code-200, 代表执行成功
						result["data"] = { "ReadImg_out"+std::to_string(ReadImg_count) };  

						ReadImg_count ++;

					//WriteImg
					} else if (function == "WriteImg") {

						auto params = action["params"];
						std::map<std::string, cv::Mat>::iterator iter;	// 从map中寻找传来的参数所对应的数据
						iter = map_cvmat.find(params[0]);
						cv::Mat in;
						
						if(iter != map_cvmat.end()) {

							in = iter->second;		
							std::cout<<"Running WriteImg with "<<iter->first<<std::endl;
							speedbot2d::WriteImg(in);

						} else {
							std::cout<<"Wrong Params!!!"<<std::endl;
							result["code"] = 300;		// code 300 代表没有找到输入的参数对应数据
							result["msg"] = "Wrong Params";
						}

					} else if (function == "ReadPLY") {

						auto params = action["params"];
						std::map<std::string, pcl::PointCloud<pcl::PointXYZ>>::iterator iter;	// 从map中寻找传来的参数所对应的数据
						
						pcl::PointCloud<pcl::PointXYZ>::Ptr ReadPLY_in(new pcl::PointCloud<pcl::PointXYZ>);	

						//auto attributes = action["attributes"];				// 取出固定属性，这是一个包含了 key-value 的数组
						std::string filename = "test";
						
						speedbot3d::ReadPLY(ReadPLY_in, filename);
						//test
						//speedbot2d::EdgeDetectionLaplacian(ReadImg_in, ReadImg_in);
						//
						map_plyptr.insert(std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr>::value_type("ReadPLY_out"+std::to_string(ReadPLY_count), ReadPLY_in));

						std::cout<<"Result: ReadPLY_out"+std::to_string(ReadPLY_count) <<std::endl;
						
						// 封装执行结果
						result["code"] = 200;   // code-200, 代表执行成功
						result["data"] = { "ReadPLY_out"+std::to_string(ReadPLY_count) };  

						ReadPLY_count ++;
					
					
					
					} else if (function == "DownSampling") {

						auto params = action["params"];
						std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator iter;	// 从map中寻找传来的参数所对应的数据

						iter = map_plyptr.find(params[0]);
						pcl::PointCloud<pcl::PointXYZ>::Ptr DownSampling_in(new pcl::PointCloud<pcl::PointXYZ>);	

						auto attributes = action["attributes"];				// 取出固定属性，这是一个包含了 key-value 的数组
						float leaf_size = attributes["leaf_size"];
						
						if (iter != map_plyptr.end()) {

							DownSampling_in = iter->second;		
							std::cout<<"Running DownSampling with "<<iter->first<<std::endl;

							pcl::PointCloud<pcl::PointXYZ>::Ptr DownSampling_out(new pcl::PointCloud<pcl::PointXYZ>);
							if(speedbot3d::DownSampling(iter->second, DownSampling_out, leaf_size) == 0) {

								map_plyptr.insert(std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr>::value_type("DownSampling_out"+std::to_string(DownSampling_count), DownSampling_out));

								std::cout<<"Result: DownSampling_out"+std::to_string(DownSampling_count) <<std::endl;
								// 封装执行结果
								result["code"] = 200;   // code-200, 代表执行成功
								result["data"] = { "DownSampling_out"+std::to_string(DownSampling_count) };  

								DownSampling_count ++;
							}
							
						} else {
							std::cout<<"Wrong Params!!!"<<std::endl;	
							result["code"] = 300;		// code 300 代表没有找到输入的参数对应数据
							result["msg"] = "Wrong Params";
						}
					
					} else if (function == "StatisticalOutlierRemoval") {

						auto params = action["params"];
						std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator iter;	// 从map中寻找传来的参数所对应的数据

						iter = map_plyptr.find(params[0]);
						pcl::PointCloud<pcl::PointXYZ>::Ptr StatisticalOutlierRemoval_in(new pcl::PointCloud<pcl::PointXYZ>);	

						auto attributes = action["attributes"];				// 取出固定属性，这是一个包含了 key-value 的数组
						int mean_k = attributes["mean_k"];
						float threshold = attributes["threshold"];


						if (iter != map_plyptr.end()) {

							StatisticalOutlierRemoval_in = iter->second;		
							std::cout<<"Running StatisticalOutlierRemoval with "<<iter->first<<std::endl;

							pcl::PointCloud<pcl::PointXYZ>::Ptr StatisticalOutlierRemoval_out(new pcl::PointCloud<pcl::PointXYZ>);
							if(speedbot3d::StatisticalOutlierRemoval(iter->second, StatisticalOutlierRemoval_out, mean_k, threshold) == 0) {

								map_plyptr.insert(std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr>::value_type("StatisticalOutlierRemoval_out"+std::to_string(StatisticalOutlierRemoval_count), StatisticalOutlierRemoval_out));

								std::cout<<"Result: StatisticalOutlierRemoval_out"+std::to_string(StatisticalOutlierRemoval_count) <<std::endl;
								// 封装执行结果
								result["code"] = 200;   // code-200, 代表执行成功
								result["data"] = { "StatisticalOutlierRemoval_out"+std::to_string(StatisticalOutlierRemoval_count) };  

								StatisticalOutlierRemoval_count ++;
							}

						} else {
							std::cout<<"Wrong Params!!!"<<std::endl;	
							result["code"] = 300;		// code 300 代表没有找到输入的参数对应数据
							result["msg"] = "Wrong Params";
						}			


					} else if (function == "PassFilter") {

						auto params = action["params"];
						std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator iter;	// 从map中寻找传来的参数所对应的数据

						iter = map_plyptr.find(params[0]);
						pcl::PointCloud<pcl::PointXYZ>::Ptr PassFilter_in(new pcl::PointCloud<pcl::PointXYZ>);	

						auto attributes = action["attributes"];				// 取出固定属性，这是一个包含了 key-value 的数组
						float x_min = attributes["x_min"];
						float x_max = attributes["x_max"];
						float y_min = attributes["y_min"];
						float y_max = attributes["y_max"];
						float z_min = attributes["z_min"];
						float z_max = attributes["z_max"];

						if (iter != map_plyptr.end()) {

							PassFilter_in = iter->second;		
							std::cout<<"Running PassFilter with "<<iter->first<<std::endl;

							pcl::PointCloud<pcl::PointXYZ>::Ptr PassFilter_out(new pcl::PointCloud<pcl::PointXYZ>);
							if(speedbot3d::PassFilter(PassFilter_in, PassFilter_out, x_min, x_max, y_min, y_max, z_min, z_max) == 0) {

								map_plyptr.insert(std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr>::value_type("PassFilter_out"+std::to_string(PassFilter_count), PassFilter_out));

								std::cout<<"Result: PassFilter_out"+std::to_string(PassFilter_count) <<std::endl;
								// 封装执行结果
								result["code"] = 200;   // code-200, 代表执行成功
								result["data"] = { "PassFilter_out"+std::to_string(PassFilter_count) };  

								PassFilter_count ++;
							}

						} else {
							std::cout<<"Wrong Params!!!"<<std::endl;	
							result["code"] = 300;		// code 300 代表没有找到输入的参数对应数据
							result["msg"] = "Wrong Params";
						}	


					} else if (function == "ChangeDetector") {

						auto params = action["params"];
						std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator iter;	// 从map中寻找传来的参数所对应的数据

						iter = map_plyptr.find(params[0]);
						pcl::PointCloud<pcl::PointXYZ>::Ptr ChangeDetector_in1(new pcl::PointCloud<pcl::PointXYZ>);	
						pcl::PointCloud<pcl::PointXYZ>::Ptr ChangeDetector_in2(new pcl::PointCloud<pcl::PointXYZ>);	

						auto attributes = action["attributes"];				// 取出固定属性，这是一个包含了 key-value 的数组
						float resolution = attributes["resolution"];

						if (iter != map_plyptr.end()) {

							ChangeDetector_in1 = iter->second;		
							std::cout<<"Running ChangeDetector with "<<iter->first<<std::endl;

							iter = map_plyptr.find(params[1]);
							if (iter != map_plyptr.end()) {

								ChangeDetector_in2 = iter->second;		
								std::cout<<"Running ChangeDetector with "<<iter->first<<std::endl;

								pcl::PointCloud<pcl::PointXYZ>::Ptr ChangeDetector_out(new pcl::PointCloud<pcl::PointXYZ>);
								if(speedbot3d::ChangeDetector(ChangeDetector_in1, ChangeDetector_in2, ChangeDetector_out, resolution) == 0) {

									map_plyptr.insert(std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr>::value_type("ChangeDetector_out"+std::to_string(ChangeDetector_count), ChangeDetector_out));

									std::cout<<"Result: ChangeDetector_out"+std::to_string(ChangeDetector_count) <<std::endl;
									// 封装执行结果
									result["code"] = 200;   // code-200, 代表执行成功
									result["data"] = { "ChangeDetector_out"+std::to_string(ChangeDetector_count) };  

									ChangeDetector_count ++;
								}

							} else {
								std::cout<<"Wrong Params!!!"<<std::endl;	
								result["code"] = 300;		// code 300 代表没有找到输入的参数对应数据
								result["msg"] = "Wrong Params";
							}

						} else {
							std::cout<<"Wrong Params!!!"<<std::endl;	
							result["code"] = 300;		// code 300 代表没有找到输入的参数对应数据
							result["msg"] = "Wrong Params";
						}	
						


					} else if (function == "GAPacking::CPlanning_Box PlanningBox") {
						
						
					} else if (function == "PlanningBox.Reset()") {

						PlanningBox.Reset();
						
						
					} else if (function == "PlanningBox.PlacementPlanning(place_box)") {
						
						auto params = action["params"];
						std::map<std::string,GAPacking::boxinfo>::iterator iter;	// 从map中寻找传来的参数所对应的数据

						

						auto attributes = action["attributes"];				// 取出固定属性，这是一个包含了 key-value 的数组
						int dim1 = attributes["dim1"];
						int dim2 = attributes["dim2"];
						int dim3 = attributes["dim3"];

						GAPacking::boxinfo place_box;

						place_box.dim1 = dim1;
						place_box.dim2 = dim2;
						place_box.dim3 = dim3;
						place_box.vol = place_box.dim1 * place_box.dim2 * place_box.dim3;

						bool result_planning = PlanningBox.PlacementPlanning(place_box);
					

						iter = map_boxinfo.find(params[0]);
						

						if (iter != map_boxinfo.end()) {

							iter->second = place_box;		
							//std::cout<<"Running PassFilter with "<<iter->first<<std::endl;

			

								// map_boxinfo.insert(std::map<std::string, GAPacking::boxinfo>::Ptr>::value_type("PassFilter_out"+std::to_string(PlacementPlanning_count), place_box));

								std::cout<<"Result: PlacementPlanning_count_out"+std::to_string(PlacementPlanning_count) <<std::endl;
								// 封装执行结果
								result["code"] = 200;   // code-200, 代表执行成功
								result["data"] = { "PlacementPlanning_count_out"+std::to_string(PlacementPlanning_count) };  

								PlacementPlanning_count ++;
							

						} else {
							map_boxinfo.insert(std::map<std::string, GAPacking::boxinfo>::value_type(iter->first, place_box));
						}	

					} else if (action["function"] == "PlanningBox.SaveState()") {

						PlanningBox.SaveState();
						
						
					} else if (action["function"] == "PlanningBox.UpdateState(place_box)") {


					
					
					} else {	//没找到输入的函数

						std::cout<<"Can not Find Function!!!"<<std::endl;
						result["code"] = 250;		// code 250 代表没有找到输入的函数
						result["msg"] = "Can not Find Function";

					}

        }	else {
					// 没有获取到参数
					result["code"] = 500;
					result["msg"] = "param is empty";
        }

        // 最终输出 json 格式的执行结果
        res.set_content(result.dump(), "text/plain");
	
  });

	
	
	svr.Get("/show/", [](const httplib::Request& req, httplib::Response& res) {
		if (req.has_param("id")) {
			std::string str_action = req.get_param_value("id");
			//json action = json::parse(str_action);
			//std::cout<str_action<<std::endl;
			//if (action["function"] == "ShowImg") {
				std::string id = str_action;
				std::cout<<id<<std::endl;
				//auto params = action["params"];
				std::map<std::string, cv::Mat>::iterator iter;	// 从map中寻找传来的参数所对应的数据
			
				iter = map_cvmat.find(id);

				cv::Mat in;
				if(iter != map_cvmat.end()) {//image
					in = iter->second;		

					std::vector<uchar> data_encode;	//将图片进行编码以传输
					imencode(".png", in, data_encode);
					std::string str_encode(data_encode.begin(), data_encode.end());
					std::cout<<"ShowImg Finish"<<std::endl;
					res.set_content(str_encode, "image/png");

				} else {//pointcloud

					std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator iter_cloud;	// 从map中寻找传来的参数所对应的数据
					iter_cloud = map_plyptr.find(id);
					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);

					if(iter_cloud != map_plyptr.end()) {
						cloud_in = iter_cloud->second;

						pcl::io::savePLYFileBinary("../data/save_tmp.ply", *cloud_in);
						std::ifstream ifile("../data/save_tmp.ply");
						std::ostringstream buf;
						char ch;
						while(buf&&ifile.get(ch))
						buf.put(ch);

						res.set_content(buf.str(), "cloud/ply");

					}
					
					std::cout<<"wrong param!!!"<<std::endl;
					//return -1;
				}
			
		} 
	});

	
	svr.listen("0.0.0.0", 8083);
	
  return 0;
}



// int main()
// {
// 	// int xx, yy, zz; //托盘的三个维度
// 	// //托盘尺寸
// 	// xx = 113;
// 	// yy = 95;
// 	// zz = 100;

// 	// ///////////////////////////////////////////////////////////
// 	// vector<GAPacking::boxinfo> Box_size;

// 	// GAPacking::boxinfo place_box;


		
// 	// place_box.dim1 = 32;
// 	// place_box.dim2 = 56;
// 	// place_box.dim3 = 11;
// 	// place_box.vol = 32 * 56 * 11;


// 	// //初始化
// 	// GAPacking::CPlanning_Box PlanningBox(xx, yy, zz);
// 	// PlanningBox.Reset();
// 	// bool result_planning = PlanningBox.PlacementPlanning(place_box);
// 	// PlanningBox.SavePlacement(place_box);
// 	// int k = PlanningBox.UpdateState(place_box);
// 	// PlanningBox.SaveState();

// 	cv::Mat ReadImg_in = cv::imread("../data/autothres.png", cv::IMREAD_UNCHANGED);
// 	cv::Mat out_image = cv::Mat::zeros(ReadImg_in.size(), ReadImg_in.type());  //创建一个和原图像大小相同，类型相同，像素值为0的图像

// 		for (int i = 0; i < ReadImg_in.rows; i++) {		//对每个像素点的操作
// 				for (int j = 0; j < ReadImg_in.cols; j++) {	// x = 255 * (x - min) / range;
					
				
// 						out_image.at<uchar>(i,j) = cv::saturate_cast<uchar>(ReadImg_in.at<uchar>(i,j)/0.04 );

							
						
						
// 				}
// 			}

// 	std::cout<<ReadImg_in.rows<<ReadImg_in.cols<<std::endl;
// 	cv::imwrite("../data/output.png", out_image);


// }
	


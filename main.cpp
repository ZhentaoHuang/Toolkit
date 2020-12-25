/*
 * Author: Zhentao Huang
 * Email: zhentaohuang222@gmail.com
 */
#include "main.h"

#include "httplib.h"
#include "imgproc.h"
#include "json.hpp"
#include <map>
using json = nlohmann::json;

//#include "plane.h"

int main(int agrc, char* argv[]) {
	
	
	

	httplib::Server svr;
	
	svr.Get("/run/", [](const httplib::Request& req, httplib::Response& res) {
        json result;

        if (req.has_param("action")) {
					std::string str_action = req.get_param_value("action");
					json action = json::parse(str_action);

					if (action["function"] == "AdjustBrightness") {	// 调用的是函数 aaa
            
            auto params = action["params"];	// 取出参数（其实就是个数组，参数是按顺序的，分别是参数一、参数二、参数三。。。）

						std::map<std::string, cv::Mat>::iterator iter;	// 从map中寻找传来的参数所对应的数据
						iter = map_cvmat.find(params[0]);
						
						cv::Mat in;
						if(iter != map_cvmat.end()) {
							
							in = iter->second;		
						} else {
							std::cout<<"wrong param!!!"<<std::endl;
							//return -1;
						}

						auto attributes = action["attributes"];	// 取出固定属性，这是一个包含了 key-value 的数组
						std::string s =  attributes["alpha"];
						std::cout<<s<<std::endl;
						float alpha= atof(s.c_str());

						s =  attributes["beta"];
						std::cout<<s<<std::endl;
						float beta= atof(s.c_str());


						cv::Mat out = speedbot2d::AdjustBrightness(in, alpha, beta);
						
						map_cvmat.insert(std::map<std::string, cv::Mat>::value_type("GammaCorrection_out"+std::to_string(GammaCorrection_count), out));


						std::cout<<"AdjustBrightness_out"+std::to_string(AdjustBrightness_count) <<std::endl;
						
						// 封装执行结果
						result["code"] = 200;   // code-200, 代表执行成功

						// 加入函数执行的结果数据，分别是数据代号1、数据代号2、数据代号3...
						result["data"] = { "AdjustBrightness_out"+std::to_string(AdjustBrightness_count) };  
						//}
						AdjustBrightness_count ++;


					} else if (action["function"] == "GammaCorrection") {

						auto params = action["params"];	// 取出参数（其实就是个数组，参数是按顺序的，分别是参数一、参数二、参数三。。。）

						std::map<std::string, cv::Mat>::iterator iter;	// 从map中寻找传来的参数所对应的数据
						iter = map_cvmat.find(params[0]);
						
						cv::Mat in;
						if(iter != map_cvmat.end()) {
							in = iter->second;		
						} else {
							std::cout<<"wrong param!!!"<<std::endl;
							//return -1;
						}


						auto attributes = action["attributes"];	// 取出固定属性，这是一个包含了 key-value 的数组
	
						std::string s = attributes["gamma"];

						std::cout<<s<<std::endl;
						float gamma= atof(s.c_str());

						cv::Mat out = speedbot2d::GammaCorrection(in, gamma);

						
						map_cvmat.insert(std::map<std::string, cv::Mat>::value_type("GammaCorrection_out"+std::to_string(GammaCorrection_count), out));

						std::cout<<"GammaCorrection_out"+std::to_string(GammaCorrection_count) <<std::endl;
						
						// 封装执行结果
						result["code"] = 200;   // code-200, 代表执行成功

						// 加入函数执行的结果数据，分别是数据代号1、数据代号2、数据代号3...
						result["data"] = { "GammaCorrection_out"+std::to_string(GammaCorrection_count) };  

						GammaCorrection_count ++;


        	} else if (action["function"] == "ReadImg") {

						auto params = action["params"];

						cv::Mat ReadImg_in = speedbot2d::ReadImg();
						////////////////////////
				
						//in = speedbot2d::AutoThresholding(in, speedbot2d::AUTO_THRESHOLD_HUANG);
				
						////////////////////////
						// std::map<std::string, cv::Mat>::iterator iter;	//将处理完的图片存入传来的参数对应的map结构中
						// iter = map_cvmat.find(params[0]);	
						// if(iter != map_cvmat.end()) {
						// 	std::cout<<iter->first<<std::endl;
						// 	iter->second = in;		
						// } else {
						map_cvmat.insert(std::map<std::string, cv::Mat>::value_type("ReadImg_out"+std::to_string(ReadImg_count), ReadImg_in));

						std::cout<<"ReadImg_out"+std::to_string(ReadImg_count) <<std::endl;
						
						// 封装执行结果
						result["code"] = 200;   // code-200, 代表执行成功

						// 加入函数执行的结果数据，分别是数据代号1、数据代号2、数据代号3...
						result["data"] = { "ReadImg_out"+std::to_string(ReadImg_count) };  
						//}
						ReadImg_count ++;

						// std::vector<uchar> data_encode;	//将图片进行编码以传输
						// imencode(".png", in, data_encode);
						// std::string str_encode(data_encode.begin(), data_encode.end());
						// std::cout<<"ReadImg Finish"<<std::endl;
						// res.set_content(str_encode, "image/png");

					} else if (action["function"] == "WriteImg") {

						auto params = action["params"];
						std::map<std::string, cv::Mat>::iterator iter;	// 从map中寻找传来的参数所对应的数据
						iter = map_cvmat.find(params[0]);
						cv::Mat in = iter->second;

						speedbot2d::WriteImg(in);

					}// else if (action["function"] == "ShowImg") {

					// 	auto params = action["params"];
					// 	std::map<std::string, cv::Mat>::iterator iter;	// 从map中寻找传来的参数所对应的数据
					// 	iter = map_cvmat.find(params[0]);
					// 	cv::Mat in = iter->second;

					// 	//speedbot2d::WriteImg(in);

					// 	std::vector<uchar> data_encode;	//将图片进行编码以传输
					// 	imencode(".png", in, data_encode);
					// 	std::string str_encode(data_encode.begin(), data_encode.end());
					// 	std::cout<<"ShowImg Finish"<<std::endl;
					// 	res.set_content(str_encode, "image/png");

					// }

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
				if(iter != map_cvmat.end()) {
							in = iter->second;		

							std::vector<uchar> data_encode;	//将图片进行编码以传输
							imencode(".png", in, data_encode);
							std::string str_encode(data_encode.begin(), data_encode.end());
							std::cout<<"ShowImg Finish"<<std::endl;
							res.set_content(str_encode, "image/png");

						} else {
							std::cout<<"wrong param!!!"<<std::endl;
							//return -1;
						}
			
		}
	});

	
	svr.listen("0.0.0.0", 8083);
	
  return 0;
}
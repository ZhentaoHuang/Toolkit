/*
 * Author: Zhentao Huang
 * Email: zhentaohuang222@gmail.com
 */

#include "imgproc.h"

//using namespace cv;
using namespace std;



//调整图片的亮度和对比度
//输入：图片，alpha，beta
//输出：调整后的图片
cv::Mat adjustBrightness(const cv::Mat in_image, float alpha=15, float beta=0)
{
	cv::Mat out_image = cv::Mat::zeros(in_image.size(), in_image.type());  //创建一个和原图像大小相同，类型相同，像素值为0的图像。

	
	//对每个像素点的操作
	for (int i = 0; i < in_image.rows; i++)
	{
		for (int j = 0; j < in_image.cols; j++)
		{
			if (in_image.channels() == 3)     //如果是RGB图像
			{ 
				out_image.at<cv::Vec3b>(i, j)[0] = cv::saturate_cast<uchar>(alpha*(in_image.at<cv::Vec3b>(i, j)[0]) + beta);
				out_image.at<cv::Vec3b>(i, j)[1] = cv::saturate_cast<uchar>(alpha*(in_image.at<cv::Vec3b>(i, j)[1]) + beta);
				out_image.at<cv::Vec3b>(i, j)[2] = cv::saturate_cast<uchar>(alpha*(in_image.at<cv::Vec3b>(i, j)[2]) + beta);
			}
			else if (in_image.channels() == 1)  //如果是灰度图像
			{
				out_image.at<uchar>(i,j) = cv::saturate_cast<uchar>(alpha*(in_image.at<uchar>(i, j)) + beta);
				
			}
		}
	}
	
    return out_image;
}



//图片Gamma矫正
//输入：图片，gamma
//输出：调整后的图片
cv::Mat gammaCorrection(const cv::Mat in_image, float gamma=2.5)
{
	cv::Mat out_image = cv::Mat::zeros(in_image.size(), in_image.type());
	float inverse_gamma = 1 / gamma;

	for(int i = 0; i < in_image.rows; i++)
	{
		for(int j = 0; j < in_image.cols; j++)
		{
			if(in_image.channels() == 3)	//如果是RGB图像
			{
				out_image.at<cv::Vec3b>(i,j)[0] = cv::saturate_cast<uchar>(pow(in_image.at<cv::Vec3b>(i,j)[0] / 255.0, inverse_gamma) * 255.0);
				out_image.at<cv::Vec3b>(i,j)[1] = cv::saturate_cast<uchar>(pow(in_image.at<cv::Vec3b>(i,j)[1] / 255.0, inverse_gamma) * 255.0);
				out_image.at<cv::Vec3b>(i,j)[2] = cv::saturate_cast<uchar>(pow(in_image.at<cv::Vec3b>(i,j)[2] / 255.0, inverse_gamma) * 255.0);
			}
			else if(in_image.channels() == 1)	//如果是灰度图像
			{
				out_image.at<uchar>(i,j) = cv::saturate_cast<uchar>(pow(in_image.at<uchar>(i,j) / 255.0, inverse_gamma) * 255.0);
			}
		}
	}

	return out_image;
}

//供本地测试用
cv::Mat readImg()
{
	cv::Mat out_image = cv::imread("../test.png", cv::IMREAD_UNCHANGED);

	return out_image;
}
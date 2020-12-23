/*
 * Author: Zhentao Huang
 * Email: zhentaohuang222@gmail.com
 */

#include "imgproc.h"



namespace speedbot2d{


/**
 * @brief 调整图片的亮度和对比度
 * 
 * @param in_image 输入的需要调整的图片
 * @param alpha 对比度参数，越大对比度越大
 * @param beta 亮度参数，越大亮度越大
 * @return cv::Mat 输出调整后的图片
 */
cv::Mat AdjustBrightness(const cv::Mat in_image, float alpha=15, float beta=0) 
{
	cv::Mat out_image = cv::Mat::zeros(in_image.size(), in_image.type());  //创建一个和原图像大小相同，类型相同，像素值为0的图像
	//对每个像素点的操作
	for (int i = 0; i < in_image.rows; i++) {
		for (int j = 0; j < in_image.cols; j++) {
			if (in_image.channels() == 3) {    //如果是RGB图像 
				out_image.at<cv::Vec3b>(i, j)[0] = cv::saturate_cast<uchar>(alpha*(in_image.at<cv::Vec3b>(i, j)[0]) + beta);
				out_image.at<cv::Vec3b>(i, j)[1] = cv::saturate_cast<uchar>(alpha*(in_image.at<cv::Vec3b>(i, j)[1]) + beta);
				out_image.at<cv::Vec3b>(i, j)[2] = cv::saturate_cast<uchar>(alpha*(in_image.at<cv::Vec3b>(i, j)[2]) + beta);
			} else if (in_image.channels() == 1) { //如果是灰度图像
			
				out_image.at<uchar>(i,j) = cv::saturate_cast<uchar>(alpha*(in_image.at<uchar>(i, j)) + beta);
				
			}
		}
	}
	
    return out_image;
}




/**
 * @brief 图片Gamma矫正
 * 
 * @param in_image 需要矫正的图片
 * @param gamma 矫正的gamma参数
 * @return cv::Mat 矫正后的图片
 */
cv::Mat GammaCorrection(const cv::Mat in_image, float gamma=2.5)
{
	cv::Mat out_image = cv::Mat::zeros(in_image.size(), in_image.type());
	float inverse_gamma = 1 / gamma;

	for(int i = 0; i < in_image.rows; i++) {
		for(int j = 0; j < in_image.cols; j++) {
			if(in_image.channels() == 3) {	//如果是RGB图像
				out_image.at<cv::Vec3b>(i,j)[0] = cv::saturate_cast<uchar>(pow(in_image.at<cv::Vec3b>(i,j)[0] / 255.0, inverse_gamma) * 255.0);
				out_image.at<cv::Vec3b>(i,j)[1] = cv::saturate_cast<uchar>(pow(in_image.at<cv::Vec3b>(i,j)[1] / 255.0, inverse_gamma) * 255.0);
				out_image.at<cv::Vec3b>(i,j)[2] = cv::saturate_cast<uchar>(pow(in_image.at<cv::Vec3b>(i,j)[2] / 255.0, inverse_gamma) * 255.0);
			} else if(in_image.channels() == 1)	{	//如果是灰度图像

				out_image.at<uchar>(i,j) = cv::saturate_cast<uchar>(pow(in_image.at<uchar>(i,j) / 255.0, inverse_gamma) * 255.0);
			}
		}
	}

	return out_image;
}



/**
 * @brief 供本地测试用
 * 
 * @return cv::Mat 图片
 */
cv::Mat ReadImg()
{
	cv::Mat out_image = cv::imread("../data/Crayfish_low_contrast.jpeg", cv::IMREAD_UNCHANGED);

	return out_image;
}

/**
 * @brief 供本地测试用
 * 
 * @param in_image 
 * @return int 
 */
int WriteImg(cv::Mat in_image)
{
	cv::imwrite("../data/output.png", in_image);

	return 0;
}


/**
 * @brief 根据图片的色彩进行对比度拉伸，不需要输入参数
 * 
 * @param in_image 输入需要调整的图片
 * @return cv::Mat 输出调整后的图片
 */
cv::Mat StretchContrast(const cv::Mat in_image) 
{
	cv::Mat out_image = cv::Mat::zeros(in_image.size(), in_image.type());  //创建一个和原图像大小相同，类型相同，像素值为0的图像

	if(in_image.channels() == 3){			//如果是RGB图像

		cv::Mat image_rgb[3];
		cv::split(in_image, image_rgb);	//将三通道图像分为单通道分别处理
	
		double min_value_r, max_value_r, min_value_g, max_value_g, min_value_b, max_value_b;    // 最大值，最小值
		cv::minMaxLoc(image_rgb[0], &min_value_b, &max_value_b);	//求各通道的最大值和最小值
		cv::minMaxLoc(image_rgb[1], &min_value_g, &max_value_g);
		cv::minMaxLoc(image_rgb[2], &min_value_r, &max_value_r);

		if(max_value_b == min_value_b || max_value_g == min_value_g || max_value_r == min_value_r) {

			return in_image;

		} else {

			for (int i = 0; i < in_image.rows; i++) {		//对每个像素点的操作
				for (int j = 0; j < in_image.cols; j++) {	// x = 255 * (x - min) / range;
							
						out_image.at<cv::Vec3b>(i, j)[0] = cv::saturate_cast<uchar>(255 * (in_image.at<cv::Vec3b>(i, j)[0] - min_value_b) / (max_value_b - min_value_b));
						out_image.at<cv::Vec3b>(i, j)[1] = cv::saturate_cast<uchar>(255 * (in_image.at<cv::Vec3b>(i, j)[1] - min_value_g) / (max_value_g - min_value_g));
						out_image.at<cv::Vec3b>(i, j)[2] = cv::saturate_cast<uchar>(255 * (in_image.at<cv::Vec3b>(i, j)[2] - min_value_r) / (max_value_r - min_value_r));
					
				}
			}
		}
	
	} else if (in_image.channels() == 1) { //如果是灰度图像
	
		double min_value, max_value;	//最大值，最小值
		cv::minMaxLoc(in_image, &min_value, &max_value);	//求各通道的最大值和最小值

		if(max_value == min_value) { //如果range 为 0

			return in_image;

		} else {

			for (int i = 0; i < in_image.rows; i++) {		//对每个像素点的操作
				for (int j = 0; j < in_image.cols; j++) {	// x = 255 * (x - min) / range;
						out_image.at<uchar>(i,j) = cv::saturate_cast<uchar>(255 * (in_image.at<uchar>(i,j) - min_value) / (max_value - min_value));
				}
			}
		}
	}
	
	return out_image;

}


/**
 * @brief 获取输入图片的颜色直方图作为特征之一方便后续处理
 * 
 * @param in_image 输入图片
 * @return cv::Mat 输出颜色直方图
 */
cv::Mat GetHistogram(const cv::Mat& in_image)
{

	int histSize[1] = {256};
	float hranges[2] = {0.0, 256.0};
	const float* ranges[1] = {hranges};
	int channels[1] = {0};

	cv::Mat hist;
	cv::calcHist(&in_image,
		1,				//仅为一个图像的直方图
		channels,	//使用的通道
		cv::Mat(),//不使用掩码
		hist,			//作为结果的直方图
		1,				//这时一维的直方图
		histSize,	//箱子数量
		ranges		//像素值的范围
	);
	return hist;
}

/**
 * @brief 计算用于图像二值化的阈值 -- Huang
 * 
 * @param hist 输入的颜色直方图
 * @return int 阈值
 */
int ComputeThresholdHuang(cv::Mat hist)
{
	// Code ported from the AutoThreshold ImageJ plugin:
	// Implements Huang's fuzzy thresholding method
	// Uses Shannon's entropy function (one can also use Yager's entropy
	// function) Huang L.-K. and Wang M.-J.J. (1995) "Image Thresholding by
	// Minimizing the Measures of Fuzziness" Pattern Recognition, 28(1): 41-51
	// Reimplemented (to handle 16-bit efficiently) by Johannes Schindelin Jan
	// 31, 2011
 
   // 找出直方图中最大最小值
   size_t first, last;
   for (first = 0; first < 256 && hist.at<float>(first) == 0; first++) {
     // do nothing
   }
 
   for (last = 256 - 1; last > first && hist.at<float>(last) == 0; last--) {
     // do nothing
   }
 
   if (first == last) {
     return 0;
   }

	// Calculate the cumulative density and the weighted cumulative density
	std::vector<double> S(last + 1);	//实践发现5000x3000以上的图片float类型很有可能溢出，故改成double类型
	std::vector<double> W(last + 1);

	S[0] = hist.at<float>(0);
	W[0] = 0.0f;
	for (size_t i = std::max((size_t)1, first); i <= last; i++) {
		S[i] = S[i - 1] + hist.at<float>((unsigned char)i);
		W[i] = W[i - 1] + i * hist.at<float>((unsigned char)i);
	}

	// Precalculate the summands of the entropy given the absolute difference x
	// - mu (integral)
	float C = (float)(last - first);
	std::vector<float> Smu(last + 1 - first);

	for (size_t i = 1; i < Smu.size(); i++) {
		float mu = 1 / (1 + i / C);
		Smu[i] = -mu * std::log(mu) - (1 - mu) * std::log(1 - mu);
	}

	// Calculate the threshold
	int bestThreshold = 0;
	float bestEntropy = std::numeric_limits<float>::max();

	for (size_t threshold = first; threshold <= last; threshold++) {
		float entropy = 0;
		int mu = round(W[threshold] / S[threshold]);
		for (size_t i = first; i <= threshold; i++) {
		
			entropy += Smu[(size_t)std::abs((int)i - mu)] * hist.at<float>((unsigned char)i);
		}

		mu = round((W[last] - W[threshold]) / (S[last] - S[threshold]));
		for (size_t i = threshold + 1; i <= last; i++) {
			std::cout<<(size_t)std::abs((int)i - mu)<<" "<<(W[last] - W[threshold]) / (S[last] - S[threshold])<<std::endl;
			entropy += Smu[(size_t)std::abs((int)i - mu)] * hist.at<float>((unsigned char)i);
		}

		if (bestEntropy > entropy) {
			bestEntropy = entropy;
			bestThreshold = (int)threshold;
		}
	}

	return bestThreshold;


}


	

}
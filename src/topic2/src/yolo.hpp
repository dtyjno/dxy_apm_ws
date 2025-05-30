#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include "global.hpp"
#include <opencv2/dnn/dnn.hpp>
#include <algorithm>

#ifndef YOLOV5
#define YOLOV5 true // true:Yolov5, false:yolov7
#endif

#ifndef YOLO_P6
#define YOLO_P6 false // 是否使用P6模型
#endif

struct Output
{
	int id;						// 结果类别id
	float confidence; // 结果置信度
	cv::Rect box;			// 矩形#include <ctime>框
};

class Yolo
{
public:
	Yolo()
	{
	}
	~Yolo() {}
	bool readModel(cv::dnn::Net &net, std::string &netPath, bool isCuda);
	bool Detect(cv::Mat &SrcImg, cv::dnn::Net &net, std::vector<Output> &output, int model_flag);
	cv::Mat drawPred(cv::Mat img, std::vector<Output> result, std::vector<cv::Scalar> color, int model_flag);
	void target(cv::Mat src, std::vector<Output> result, int model_flag);
	void findMin(std::vector<Output> result, int size, int model_flag);
	void getBox(std::vector<Output> result);

private:
	float sigmoid_x(float x)
	{
		return static_cast<float>(1.f / (1.f + exp(-x)));
	}
#if (defined YOLO_P6 && YOLO_P6 == true)

#if (defined YOLOV5 && YOLOV5 == false)
	const float netAnchors[4][6] = {{19, 27, 44, 40, 38, 94}, {96, 68, 86, 152, 180, 137}, {140, 301, 303, 264, 238, 542}, {436, 615, 739, 380, 925, 792}}; // yolov7-P6 anchors
#else
	const float netAnchors[4][6] = {{19, 27, 44, 40, 38, 94}, {96, 68, 86, 152, 180, 137}, {140, 301, 303, 264, 238, 542}, {436, 615, 739, 380, 925, 792}}; // yolov5-P6 anchors
#endif
	const int netWidth = 1280;	// ONNX图片输入宽度
	const int netHeight = 1280; // ONNX图片输入高度
	const int strideSize = 4;		// stride size
#else
#if (defined YOLOV5 && YOLOV5 == false)
	const float netAnchors[3][6] = {{12, 16, 19, 36, 40, 28}, {36, 75, 76, 55, 72, 146}, {142, 110, 192, 243, 459, 401}}; // yolov7-P5 anchors
#else
	const float netAnchors[3][6] = {{10, 13, 16, 30, 33, 23}, {30, 61, 62, 45, 59, 119}, {116, 90, 156, 198, 373, 326}}; // yolov5-P5 anchors
#endif
	const int netWidth = 640;	 // ONNX图片输入宽度
	const int netHeight = 640; // ONNX图片输入高度
	const int strideSize = 3;	 // stride size
#endif // YOLO_P6

	const float netStride[4] = {8, 16.0, 32, 64};

	float boxThreshold = 0.25;
	float classThreshold = 0.25;
	float nmsThreshold = 0.45;
	// float nmsScoreThreshold = boxThreshold * classThreshold;
	float nmsScoreThreshold = 0.65; // 置信度
	std::vector<std::vector<std::string>> className = {{"target"}, {"H"}};
};

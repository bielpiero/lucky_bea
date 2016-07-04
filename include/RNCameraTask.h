#ifndef RN_CAMERA_TASK_H
#define RN_CAMERA_TASK_H

#include "RNRecurrentTask.h"
#include "RNLandmark.h"
#include <opencv2/opencv.hpp>

class RNCameraTask : public RNRecurrentTask{
public:
	RNCameraTask(const char* name = "Camera Task", const char* description = "");
	virtual void task();
private:
	cv::VideoCapture capture;
};

#endif
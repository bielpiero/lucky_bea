#ifndef RN_CAMERA_TASK_H
#define RN_CAMERA_TASK_H

#include "RNRecurrentTask.h"
#include "GeneralController.h"
#include <opencv2/opencv.hpp>

#define LEFT_EYE_CAMERA 0
#define RIGHT_EYE_CAMERA 1
#define EYES_FOLDER "eyes/"

class RNCameraTask : public RNRecurrentTask{
public:
	RNCameraTask(const GeneralController* gn, const char* name = "Camera Task", const char* description = "");
	~RNCameraTask();
	virtual void task();
	virtual void onKilled();
private:
	
	cv::VideoCapture leftEye;
	cv::VideoCapture rightEye;
};

#endif
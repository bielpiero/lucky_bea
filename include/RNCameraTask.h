#ifndef RN_CAMERA_TASK_H
#define RN_CAMERA_TASK_H

#include "RNRecurrentTask.h"
#include <opencv2/opencv.hpp>

class RNCameraTask : public RNRecurrentTask{
	public:
		RNCameraTask(const char* name = "Camera Task");
		virtual void task();
};

#endif
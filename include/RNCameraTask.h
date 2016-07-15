#ifndef RN_CAMERA_TASK_H
#define RN_CAMERA_TASK_H

#include "RNRecurrentTask.h"
#include "RNLandmark.h"
#include <opencv2/opencv.hpp>

class RNCameraTask : public RNRecurrentTask{
public:
	RNCameraTask(const char* name = "Camera Task", const char* description = "");
	~RNCameraTask();
	virtual void task();
	virtual void onKilled();
private:

	int init();
	void clearLandmarks();
	int minMax(float* histogram);
	cv::Point calcPoint(cv::Point2f center, double R, double angle);
	void createRect(cv::Rect& rect, int x, int y, int width, int height);
private:
	static const int X1;
	static const int X2;
	static const int Y1;
	static const int Y2;
	static const int WIDTH1;
	static const int WIDTH2;
	static const int HEIGHT;
	
	static const double PI_DEGREES;
	cv::VideoCapture capture;

	std::vector<RNLandmark*>* landmarks;

	int videodevice;
	bool started;
};

#endif
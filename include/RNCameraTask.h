#ifndef RN_CAMERA_TASK_H
#define RN_CAMERA_TASK_H

#include "RNRecurrentTask.h"
#include "RNLandmark.h"
#include <opencv2/opencv.hpp>

#define CURVE_SIZE 4
#define CELL_MARKER_SIZE 7

class RNCameraTask : public RNRecurrentTask{
public:
	RNCameraTask(const char* name = "Camera Task", const char* description = "");
	~RNCameraTask();
	virtual void task();
	virtual void onKilled();
private:
	void rgbToGrayscale(const cv::Mat& input, cv::Mat& output);
	int getFrameFromCamera(cv::Mat &frame);
	void thresholding(const cv::Mat& inputGrayscale, cv::Mat& output);
	void clearNoisyDots(const cv::Mat input, cv::Mat& output);
	void findContours(const cv::Mat& input, std::vector<std::vector<cv::Point> > &contours, int minContourPointsAllowed);
	void findCandidates(const std::vector<std::vector<cv::Point> > &contours, std::vector<std::vector<cv::Point2f> >& markerPoints);
	void recognizeMarkers(const cv::Mat& inputGrayscale, std::vector<std::vector<cv::Point2f> >& markerPoints);
	void poseEstimation(std::vector<std::vector<cv::Point2f> >& markerPoints);
private:
	void clearLandmarks();
	float perimeter(const std::vector<cv::Point2f> &a);
	int markerDecoder(const cv::Mat& inputGrayscale, int& nRrotations);
	int hammingDistance(cv::Mat bits);
	cv::Mat rotate(cv::Mat input);
private:
	static const std::string cameraUrl;
	float minContourLengthAllowed;
	float maxContourLengthAllowed;
	std::vector<cv::Point2f> markerCorners2d;
	std::vector<std::vector<cv::Point> > contours;
	cv::Size markerSize;
	cv::Mat canonicalMarkerImage;
	cv::Mat camMatrix;
	cv::Mat distCoeff;
	static const double PI_DEGREES;
	std::vector<RNLandmark*>* landmarks;
};

#endif
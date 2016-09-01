#ifndef RN_CAMERA_TASK_H
#define RN_CAMERA_TASK_H

#include "RNRecurrentTask.h"
#include "RNLandmark.h"
#include <opencv2/opencv.hpp>
#include <curl/curl.h>

#define CURVE_SIZE 4
#define CELL_MARKER_SIZE 7
#define IMAGE_OFFSET_X 15
#define IMAGE_OFFSET_Y 5

class RNMarker{
private:
	int markerId;
	std::vector<cv::Point2f> markerPoints;
	int contourIdx;
	double area;
	double angleInRadians;
	cv::RotatedRect rect;
public:
	RNMarker(){ markerId = -1; }

	void setMarkerId(int id) { this->markerId = id; }
	void addPoint(cv::Point2f point) { markerPoints.push_back(point); }
	void setPoint(cv::Point2f point, int index) { this->markerPoints.at(index) = point; }
	void setMarkerPoints(std::vector<cv::Point2f> markerPoints) { this->markerPoints = markerPoints; }
	void setContourIdx(unsigned int contourIdx) { this->contourIdx = contourIdx; }
	void setArea(double area) { this->area = area; }
	void setThRad(double angle) { this->angleInRadians = angle; }
	void setRotatedRect(cv::RotatedRect rect) { this->rect = rect; }

	double getArea() { return this->area; }
	double getThRad() { return this->angleInRadians; }
	int getContourIdx() { return this->contourIdx; }
	cv::RotatedRect getRotatedRect() { return this->rect;}
	std::vector<cv::Point2f> getMarkerPoints() { return this->markerPoints; }
	cv::Point2f getPoint(int index){ return this->markerPoints.at(index); }
};

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
	void findCandidates(const std::vector<std::vector<cv::Point> > &contours, std::vector<RNMarker>& markerPoints);
	void recognizeMarkers(const cv::Mat& inputGrayscale, std::vector<RNMarker>& markerPoints);
	void poseEstimation(const cv::Point& center, std::vector<RNMarker>& markerPoints);
private:
	void clearLandmarks();
	float perimeter(const std::vector<cv::Point2f> &a);
	int markerDecoder(const cv::Mat& inputGrayscale, int& nRrotations);
	int hammingDistance(cv::Mat bits);
	cv::Mat rotate(cv::Mat input);
	static size_t write_data(char *ptr, size_t size, size_t nmemb, void *userdata);
private:
	static const std::string cameraUrl;
	float minContourLengthAllowed;
	float maxContourLengthAllowed;
	std::vector<cv::Point2f> markerCorners2d;
	std::vector<cv::Point3f> markerCorners3d;
	std::vector<std::vector<cv::Point> > contours;
	cv::Size markerSize;
	cv::Mat canonicalMarkerImage;
	cv::Mat camMatrix;
	cv::Mat distCoeff;

	Matrix rotation;
	Matrix traslation;

	static const double PI_DEGREES;
	std::vector<RNLandmark*>* landmarks;
	cv::VideoCapture capture;
};

#endif
#ifndef RN_OMNICAMERA_TASK_H
#define RN_OMNICAMERA_TASK_H

#include "RNRecurrentTask.h"
#include "RNLandmark.h"
#include <opencv2/opencv.hpp>
#include <curl/curl.h>

#define CURVE_SIZE 4
#define CELL_MARKER_SIZE 7
#define IMAGE_OFFSET_X 15
#define IMAGE_OFFSET_Y 5
#define RECTIFIED_IMAGE_WIDTH 1812
#define RECTIFIED_IMAGE_HEIGHT 679

class RNMarker{
private:
	int markerId;
	std::vector<cv::Point2f> markerPoints;
	int contourIdx;
	double area;
	double angleInRadians;
	cv::RotatedRect rect;
public:
	RNMarker(){ markerId = RN_NONE; }

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

class RNOmnicameraTask : public RNRecurrentTask{
public:
	RNOmnicameraTask(const char* name = "Omnicamera Task", const char* description = "");
	~RNOmnicameraTask();
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
	void poseEstimation(std::vector<RNMarker>& markerPoints);
	void markerIdNumber(const cv::Mat &bits, int &mapId, int &sectorId, int &markerId);
	int getBrightnessAverageFromHistogram(const cv::Mat& input);

	void initUndistortRectifyMap(cv::InputArray K, cv::InputArray D, cv::InputArray xi, cv::InputArray R, cv::InputArray P, const cv::Size& size,
        int m1type, cv::OutputArray map1, cv::OutputArray map2, int flags);
	void undistortImage(cv::InputArray distorted, cv::OutputArray undistorted, cv::InputArray K, cv::InputArray D, cv::InputArray xi, int flags,
        cv::InputArray knew = cv::noArray(), const cv::Size& newSize = cv::Size(), cv::InputArray R = cv::Mat::eye(3, 3, CV_64F));
private:
	void clearLandmarks();
	void drawRectangle(cv::Mat &img, RNMarker marker);
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
	cv::Mat xi;

	Matrix rotation;
	Matrix traslation;

	static const double PI_DEGREES;
	std::vector<RNLandmark*>* landmarks;
	cv::VideoCapture capture;

	enum{
        RECTIFY_PERSPECTIVE = 1,
        RECTIFY_CYLINDRICAL,
        RECTIFY_LONGLATI,
        RECTIFY_STEREOGRAPHIC
    };
};

#endif
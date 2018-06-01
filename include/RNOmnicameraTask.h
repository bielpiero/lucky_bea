#ifndef RN_OMNICAMERA_TASK_H
#define RN_OMNICAMERA_TASK_H

#include "RNRecurrentTask.h"
#include "RNLandmarkList.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/affine.hpp>
#include <curl/curl.h>
#include "GeneralController.h"

#define CURVE_SIZE 4
#define CELL_MARKER_SIZE_ROWS 8 
#define CELL_MARKER_SIZE_COLUMNS 7
#define IMAGE_OFFSET_X 15
#define IMAGE_OFFSET_Y 10
#define BLOCK_SIZE_FOR_ADAPTIVE_THRESHOLD 75

class RNMarker{
private:
	int mapId;
	int sectorId;
	int markerId;

	std::vector<cv::Point2f> markerPoints;
	int contourIdx;
	double area;
	double weight;
	double angleRad;
	double distance;
	double opticalTheta;
	cv::RotatedRect rect;

public:
	RNMarker(){ 
		mapId = RN_NONE;
		sectorId = RN_NONE;
		markerId = RN_NONE;
	}

	void addPoint(cv::Point2f point) { markerPoints.push_back(point); }
	void setPoint(cv::Point2f point, int index) { this->markerPoints.at(index) = point; }
	void setMarkerPoints(std::vector<cv::Point2f> markerPoints) { this->markerPoints = markerPoints; }
	void setContourIdx(unsigned int contourIdx) { this->contourIdx = contourIdx; }
	void setArea(double area) { this->area = area; }
	void setWeight(double weight) { this->weight = weight; }
	void setDistance(double distance) { this->distance = distance; }
	void setOpticalTheta(double angle) { this->opticalTheta = angle; }
	void setThRad(double angle) { this->angleRad = angle; }
	void setRotatedRect(cv::RotatedRect rect) { this->rect = rect; }

	double getArea() { return this->area; }
	double getWeight() { return this->weight; }
	double getDistance() { return this->distance; }
	double getThRad() { return this->angleRad; }
	double getOpticalTheta() { return this->opticalTheta; }
	int getContourIdx() { return this->contourIdx; }
	cv::RotatedRect getRotatedRect() { return this->rect;}
	std::vector<cv::Point2f> getMarkerPoints() { return this->markerPoints; }
	cv::Point2f getPoint(int index){ return this->markerPoints.at(index); }

	void setMapId(int id){
		this->mapId = id;
	}
	void setSectorId(int id){
		this->sectorId = id;
	}
	void setMarkerId(int id){
		this->markerId = id;
	}
	int getMapId(){
		return mapId;
	}
	int getMarkerId(){
		return markerId;
	}
	int getSectorId(){
		return sectorId;
	}
};

class RNOmnicameraTask : public RNRecurrentTask{
public:
	RNOmnicameraTask(const GeneralController* gn, const char* name = "Omnicamera Task", const char* description = "");
	~RNOmnicameraTask();
	virtual void task();
	virtual void onKilled();
private:
	void rgbToGrayscale(const cv::Mat& input, cv::Mat& output);
	int getFrameFromCamera(cv::Mat &frame);
	void thresholding(const cv::Mat& inputGrayscale, cv::Mat& output);
	void clearNoisyDots(const cv::Mat input, cv::Mat& output);
	void findContours(int minContourPointsAllowed);
	void findCandidates();
	void recognizeMarkers();
	void poseEstimation();
	void markerIdNumber(const Matrix &bits, int &mapId, int &sectorId, int &markerId);
	int getBrightnessAverageFromHistogram(const cv::Mat& input);

private:
	void drawRectangle(cv::Mat &img, RNMarker marker);
	float perimeter(const std::vector<cv::Point2f> &a);
	int markerDecoder(const cv::Mat& inputGrayscale, int& nRrotations, RNMarker &marker, int rows, int cols);
	int hammingDistance(Matrix bits);
	Matrix rotate(Matrix input);
	static size_t write_data(char *ptr, size_t size, size_t nmemb, void *userdata);
private:
	CURL* curl;
	CURLcode res;
	std::vector<char> data;
	std::ostringstream cameraStream;
	GeneralController* gn;

	bool enableVideoProcessing;
	
	static const std::string cameraUrl;

	double minContourLengthAllowed;
	double maxContourLengthAllowed;
	std::vector<cv::Point2f> markerCorners2d;
	std::vector<cv::Point3f> markerCorners3d;
	std::vector<std::vector<cv::Point> > contours;
	
	std::vector<RNMarker> possibleMarkersPoints;
	cv::Size markerSize;
	cv::Mat canonicalMarkerImage;
	cv::Mat camMatrix;
	cv::Mat xi;
	cv::Mat distCoeff;
	cv::Mat tiki;
	cv::Mat edges;
	cv::Mat tikiGray, tikiThreshold;

	std::vector<RNMarker> tikiMarkers;

	static const double PI_DEGREES;
	static const double MARKER_HEIGHT;
	RNLandmarkList* landmarks;
	cv::VideoCapture capture;
};

#endif
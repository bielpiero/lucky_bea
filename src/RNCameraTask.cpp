#include "RNCameraTask.h"

const int RNCameraTask::X1 = 39;
const int RNCameraTask::X2 = 364;
const int RNCameraTask::Y1 = 30;
const int RNCameraTask::Y2 = 359;
const int RNCameraTask::WIDTH1 = 235;
const int RNCameraTask::WIDTH2 = 237;
const int RNCameraTask::HEIGHT = 192;

RNCameraTask::RNCameraTask(const char* name, const char* description) : RNRecurrentTask(name, description){
	this->videodevice = 0;
	this->started = false;
}

RNCameraTask::~RNCameraTask(){
	
}

int RNCameraTask::init(){
	int result = 0;
	if(not started){
		capture = cv::VideoCapture(videodevice);
		if(not capture.isOpened()){
			RNUtils::printLn("Could not open device %d. Please check that the camera is connected to the robot.", videodevice);
			result = -1;
		} else {
			started = true;
		}
	}
	return result;
}

void RNCameraTask::task(){
	if(init() == 0){
		cv::Mat imagen, img, img2;
		capture >> imagen;

		cv::Rect box1, box2, box3, box4;

		createRect(box1, X1, Y1, WIDTH1, HEIGHT);
		createRect(box2, X2, Y1, WIDTH2, HEIGHT);
		createRect(box3, X1, Y2, WIDTH1, HEIGHT);
		createRect(box4, X2, Y2, WIDTH2, HEIGHT);

		cv::Mat cuad1(imagen, box1);
		cv::Mat cuad2(imagen, box2);
		cv::Mat cuad3(imagen, box3);
		cv::Mat cuad4(imagen, box4);

		cv::Mat pan1, pan2, panoramica;

		cv::hconcat(cuad1, cuad2, pan1);
		cv::hconcat(pan1, cuad3, pan2);
		cv::hconcat(pan2, cuad4, panoramica);
		cv::imwrite("Imagen.jpg", panoramica);

	} else {
		kill();
	}
}

void RNCameraTask::createRect(cv::Rect& rect, int x, int y, int width, int height){
	rect.x = x;
	rect.y = y;
	rect.width = width;
	rect.height = height;
}

void RNCameraTask::onKilled(){
	started = false;
	if(capture.isOpened()){
		capture.release();
	}
}
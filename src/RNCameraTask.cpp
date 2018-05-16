#include "RNCameraTask.h"


RNCameraTask::RNCameraTask(const GeneralController* gn, const char* name, const char* description) : RNRecurrentTask(gn, name, description){
	leftEye = cv::VideoCapture(LEFT_EYE_CAMERA);
	rightEye = cv::VideoCapture(RIGHT_EYE_CAMERA);
	//leftEye.set(cv::CAP_PROP_FOURCC, cv::FOURCC('M', 'J', 'P', 'G'));
	leftEye.set(cv::CAP_PROP_FRAME_WIDTH ,1280);
	leftEye.set(cv::CAP_PROP_FRAME_HEIGHT ,720);

	std::cout << "left: " << static_cast<int>(leftEye.get(cv::CAP_PROP_FOURCC)) << std::endl;

	//rightEye.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
	//rightEye.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
	//rightEye.set(CV_CAP_PROP_FRAME_HEIGHT, 720);

	//std::cout << "right: " << static_cast<int>(rightEye.get(CV_CAP_PROP_FOURCC)) << std::endl;

	if(leftEye.isOpened()){
		RNUtils::printLn("Streaming from camera device: %d", LEFT_EYE_CAMERA);
	} else {
		leftEye.release();
		RNUtils::printLn("Could not open device: %d", LEFT_EYE_CAMERA);
	}

	/*if(rightEye.isOpened()){
		RNUtils::printLn("Streaming from camera device: %d", RIGHT_EYE_CAMERA);
	} else {
		rightEye.release();
		RNUtils::printLn("Could not open device: %d", RIGHT_EYE_CAMERA);
	}*/
}

RNCameraTask::~RNCameraTask(){
	rightEye.release();
	leftEye.release();

	if(std::remove(std::string(RNUtils::getApplicationPath() + EYES_FOLDER + "left.jpeg").c_str()) != 0){
		RNUtils::printLn("left.jpeg: No se pudo borrá esta vaina");
	}
	if(std::remove(std::string(RNUtils::getApplicationPath() + EYES_FOLDER + "right.jpeg").c_str()) != 0){
		RNUtils::printLn("right.jpeg: No se pudo borrá esta vaina");
	}
}

void RNCameraTask::task(){
	cv::Mat leftImage, rightImage;
	if(leftEye.isOpened()){
		//leftEye >> leftImage;
		//cv::resize(leftImage, leftImage, cv::Size(1024, 768));
		cv::imwrite(RNUtils::getApplicationPath() + EYES_FOLDER + "left.jpeg", leftImage);
	}
	/*if(rightEye.isOpened()){
		//rightEye >> rightImage;
		//cv::resize(rightImage, rightImage, cv::Size(1024, 768));
		cv::imwrite(RNUtils::getApplicationPath() + EYES_FOLDER + "right.jpeg", rightImage);
	}*/
}

void RNCameraTask::onKilled(){

}

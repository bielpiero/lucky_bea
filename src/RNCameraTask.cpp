#include "RNCameraTask.h"

const double RNCameraTask::PI_DEGREES = 180.0;

const int RNCameraTask::X1 = 39;
const int RNCameraTask::X2 = 364;
const int RNCameraTask::Y1 = 30;
const int RNCameraTask::Y2 = 359;
const int RNCameraTask::WIDTH1 = 240;
const int RNCameraTask::WIDTH2 = 236;
const int RNCameraTask::HEIGHT = 192;

RNCameraTask::RNCameraTask(const char* name, const char* description) : RNRecurrentTask(name, description){
	this->videodevice = 0;
	this->started = false;
	landmarks = new std::vector<RNLandmark*>();
}

RNCameraTask::~RNCameraTask(){
	if(capture.isOpened()){
		capture.release();
	}
	clearLandmarks();
	delete landmarks;
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

void RNCameraTask::clearLandmarks(){
	for (int i = 0; i < landmarks->size(); i++){
		delete landmarks->at(i);
	}
	landmarks->clear();
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

		//cv::imwrite("Imagen.jpg", panoramica);

		cv::Mat panoramicaGRAY;
		cv::cvtColor(panoramica, img2, CV_BGR2GRAY);
		cv::equalizeHist(img2, panoramicaGRAY);
		cv::GaussianBlur(panoramicaGRAY, img2, cv::Size(9, 9), 2, 2);

		cv::Mat imgThresholded;

		int histSize = 256; //bin size
		float range[] = { 0, 256 } ;
 		const float* histRange = { range };

		cv::Mat hist;
		cv::calcHist(&img2, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, true, false);

		float hist_vector[256];
		hist_vector[0] = 0;
		int histWidth = 512; 
		int histHeight = 400;
		int bin_w = cvRound((double)histWidth / histSize);

		cv::Mat histImage(histHeight, histWidth, CV_8UC3, cv::Scalar(0, 0, 0));

		cv::normalize(hist, hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat());

		/*for (int i = 1; i < histSize; i++){
			hist_vector[i] = hist.at<float>(i);
			line(histImage, cv::Point(bin_w*(i - 1), histHeight - cvRound(hist.at<float>(i - 1))), cv::Point(bin_w*(i), histHeight - cvRound(hist.at<float>(i))), cv::Scalar(255, 0, 0), 2, 8, 0);
		}*/

		//int thresholdValue = MinMax(hist_vector);

		//line(histImage, cv::Point((thresholdValue * histWidth) / 255, 0), cv::Point((thresholdValue * histWidth) / 255, 400), cv::Scalar(0, 0, 255), 2, 8, 0);

		int thresholdValue = 80;
		cv::threshold(img2, imgThresholded, thresholdValue, 255, cv::THRESH_BINARY);

		cv::Mat imgErosion;
		cv::Mat imgFiltered;
		cv::Mat final;

		cv::erode(imgThresholded, imgErosion, cv::Mat());
		cv::dilate(imgErosion, imgFiltered, cv::Mat());
		final = imgFiltered.clone();

		cv::Mat edges, traces;
		//int fontFace = FONT_HERSHEY_PLAIN;
		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;

		std::vector<cv::Point> approx;

		cv::Canny(final, edges, 30, 90, 3);
		cv::findContours(edges, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

		std::vector<cv::Moments> mu(contours.size()); // Moments
		std::vector<cv::Point2f> mc(contours.size()); // Mass centers

		for (int i = 0; i < contours.size(); i++){
			mu[i] = cv::moments(contours[i], false);
			mc[i] = cv::Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);
		}

		cv::Mat mask(panoramica.rows, panoramica.cols, CV_8UC1, cv::Scalar(0, 0, 0));

		cv::Scalar avg;
		clearLandmarks();
		std::vector<int> contoursIds;
		for (int i = 0; i < contours.size(); i++){
			int k = i;
			int c = 0;

			cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true)*0.02, true);

			while (hierarchy[k][2] != -1){
				k = hierarchy[k][2];
				c++;
			}

			if (hierarchy[k][2] != -1){
				c++;
			}

			double currentCountour = cv::contourArea(contours[i]);
			if ((c >= 1) && (currentCountour < 3500) && (currentCountour > 50) && (approx.size() >= 3) && (approx.size() <= 8)){

				cv::drawContours(mask, contours, i, cv::Scalar(255, 255, 255), 1, 8, hierarchy, 0);
				/*cv::Mat contPan = panoramica.clone();
				cv::drawContours(contPan, contours, i, cv::Scalar(255, 200, 0), 2, 8, hierarchy, 0);
				std::ostringstream filename;
				filename << "mikel/Panoramica contour " << i << ".jpg";
				cv::imwrite(filename.str().c_str(), contPan);*/

				cv::floodFill(mask, mc[i], cv::Scalar(255,255,255));
				
				avg = cv::mean(panoramica, mask);
				
				if (((avg.val[2] > 105) and (avg.val[1] < 70) and (avg.val[0] < 75)) and ((currentCountour < 499) and (currentCountour > 79)) || 
				   (((avg.val[2] > 92) and (avg.val[1] < 50) and (avg.val[0] < 60)) and (currentCountour < 80)) || 
				    ((avg.val[2] > 135) and (avg.val[1] < 65) and (avg.val[0] < 82)) and (currentCountour > 500)){   ///MODIFICAR ESTOS VALORES PARA DETECTAR EL COLOR QUE QUERAMOS
					// aqui se guarda el identificador del contorno que cumple con las características de la baliza. cada (i) es una baliza
					//RNUtils::printLn("Contour %d: size: %f, {R: %f, G: %f, B: %f}", i, currentCountour, avg.val[2], avg.val[1], avg.val[0]);
					//std::cout << "m00: " << mu[i].m00 << ", m10: " << mu[i].m10 << std::endl;
					contoursIds.push_back(i);
				}
			}
		}

		

		for (int i = 0; i < contoursIds.size(); i++){
			RNLandmark* currentLandmark = new RNLandmark;

			double landmarkDistance = (-6.46309e-9 * std::pow(mu[i].m00 , 3)) + (0.000045627 * std::pow(mu[i].m00 , 2)) - (0.118347 * mu[i].m00) + 183.327;
			currentLandmark->setPointsXMean(landmarkDistance);

			//RNUtils::printLn("{moment10: %f, moment00: %f}", i, mu[i].m10, mu[i].m00);

			double landmarkAngle = (((mu[i].m10 / mu[i].m00) * (2 * PI_DEGREES)) / panoramica.cols) * M_PI / PI_DEGREES;
			//RNUtils::printLn("angle: %f", landmarkAngle);
			currentLandmark->setPointsYMean(landmarkAngle);

			landmarks->push_back(currentLandmark);
		}

		//RNUtils::printLn("There are %d visual landmarks", landmarks->size());
		/*for (int i = 0; i < landmarks->size(); i++){
			RNUtils::printLn("Landmark @ %d: {d: %f, \u03d1: %f}", i, landmarks->at(i)->getPointsXMean(), landmarks->at(i)->getPointsYMean());
		}*/

	}
}

void RNCameraTask::createRect(cv::Rect& rect, int x, int y, int width, int height){
	rect.x = x;
	rect.y = y;
	rect.width = width;
	rect.height = height;
}

int RNCameraTask::minMax(float* histogram){
	float histogramAux[256];
	int max1 = 0, max2 = 0, min;

	for (int i = 1; i < 256; i++){
		if (histogram[max1] < histogram[i])
			max1 = i;
	}
	for (int i = 0; i < 256; i++){
		histogramAux[i] = histogram[i] * std::abs(i - max1);
	}

	for (int i = 1; i < 256; i++){
		if (histogramAux[max2] < histogramAux[i])
			max2 = i;
	}
	
	if (max1 < max2){
		min = max1;
		for (int i = min + 1; i < max2; i++){
			if (histogram[i] < histogram[min]){
				min = i;
			}
		}
	} else {
		min = max2;
		for (int i = min + 1; i < max1; i++){
			if (histogram[i] < histogram[min]){
				min = i;
			}
		}
	}

	return (min);
}

cv::Point RNCameraTask::calcPoint(cv::Point2f center, double R, double angle){
	return center + cv::Point2f((float)std::cos(angle), (float)-std::sin(angle)) * (float)R;
}

void RNCameraTask::onKilled(){
	started = false;
}
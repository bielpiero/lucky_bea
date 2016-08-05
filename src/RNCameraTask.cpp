#include "RNCameraTask.h"

const double RNCameraTask::PI_DEGREES = 180.0;

const std::string RNCameraTask::cameraUrl = "http://192.168.1.33/record/current.jpg";

RNCameraTask::RNCameraTask(const char* name, const char* description) : RNRecurrentTask(name, description){
	minContourLengthAllowed = 1000.0;
	maxContourLengthAllowed = 4000.0;
	markerSize = cv::Size(133, 194);

	markerCorners2d.push_back(cv::Point2f(0, 0));
	markerCorners2d.push_back(cv::Point2f(markerSize.width - 1, 0));
	markerCorners2d.push_back(cv::Point2f(markerSize.width - 1, markerSize.height - 1));
	markerCorners2d.push_back(cv::Point2f(0, markerSize.height - 1));

	camMatrix = cv::Mat(3, 3, CV_32F);
	camMatrix.at<float>(0, 0) = 3.3609061519256187e+002;
	camMatrix.at<float>(0, 1) = 0.0;
	camMatrix.at<float>(0, 2) = 6.3950000000000000e+002;
	camMatrix.at<float>(1, 0) = 0.0;
	camMatrix.at<float>(1, 1) = 3.3609061519256187e+002;
	camMatrix.at<float>(1, 2) = 4.7950000000000000e+002;
	camMatrix.at<float>(2, 0) = 0.0;
	camMatrix.at<float>(2, 1) = 0.0;
	camMatrix.at<float>(2, 2) = 1.0;

	distCoeff = cv::Mat(5, 1, CV_32F);
	distCoeff.at<float>(0, 0) = -2.1524901378355338e-001;
	distCoeff.at<float>(1, 0) = 4.0218153464358121e-002;
	distCoeff.at<float>(2, 0) = 0.0;
	distCoeff.at<float>(3, 0) = 0.0;
	distCoeff.at<float>(4, 0) = -3.2911593249722450e-003;

	landmarks = new std::vector<RNLandmark*>();
}

RNCameraTask::~RNCameraTask(){
	clearLandmarks();
	delete landmarks;
}

void RNCameraTask::clearLandmarks(){
	for (int i = 0; i < landmarks->size(); i++){
		delete landmarks->at(i);
	}
	landmarks->clear();
}

int RNCameraTask::getFrameFromCamera(cv::Mat &frame){
	int result = 0;
	cv::VideoCapture capture(cameraUrl);
	if (capture.isOpened()){
		capture.read(frame);
		capture.release();
	}
	else {
		result = RN_NONE;
	}
	return result;
}

void RNCameraTask::rgbToGrayscale(const cv::Mat& input, cv::Mat& output){
	cv::cvtColor(input, output, CV_BGR2GRAY);
}

void RNCameraTask::thresholding(const cv::Mat& inputGrayscale, cv::Mat& output){
	int thresholdValue = 100;
	cv::threshold(inputGrayscale, output, thresholdValue, 255, cv::THRESH_BINARY_INV);
}

void RNCameraTask::clearNoisyDots(const cv::Mat input, cv::Mat& output){
	int erosionSize = 0;
	cv::Mat erosion;
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * erosionSize + 1, 2 * erosionSize + 1), cv::Point(erosionSize - 1, erosionSize - 1));
	erode(input, erosion, element);
	dilate(erosion, output, element);
}

void RNCameraTask::findContours(const cv::Mat& input, std::vector<std::vector<cv::Point> > &contours, int minContourPointsAllowed){
	std::vector<std::vector<cv::Point> > allContours;
	std::vector<cv::Vec4i> hierarchy;
	cv::Mat edges;
	cv::Canny(input, edges, 100, 180, 5);
	cv::findContours(edges, allContours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	//cv::findContours(edges, allContours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
	contours.clear();
	for (unsigned int i = 0; i < allContours.size(); i++){
		if (allContours.at(i).size() > minContourPointsAllowed){
			contours.push_back(allContours.at(i));
		}
	}
	//contours = allContours;
}

void RNCameraTask::findCandidates(const std::vector<std::vector<cv::Point> > &contours, std::vector<std::vector<cv::Point2f> >& markerPoints){
	std::vector<cv::Point>  approxCurve;
	std::vector<std::vector<cv::Point2f> > possibleMarkersPoints;
	for (unsigned int i = 0; i < contours.size(); i++){
		double eps = contours.at(i).size() * .1;
		cv::approxPolyDP(contours.at(i), approxCurve, eps, true);
		if (approxCurve.size() == CURVE_SIZE && cv::isContourConvex(approxCurve)){
			float minDist = std::numeric_limits<float>::max();
			for (int i = 0; i < approxCurve.size(); i++){
				cv::Point side = approxCurve.at(i) - approxCurve.at((i + 1) % 4);
				minDist = std::min(minDist, (float)side.dot(side));
			}
			
			if (minDist > this->minContourLengthAllowed && minDist < this->maxContourLengthAllowed){
				std::vector<cv::Point2f> marker;
				for (int i = 0; i < approxCurve.size(); i++){
					marker.push_back(cv::Point2f(approxCurve.at(i).x, approxCurve.at(i).y));
				}
				
				cv::Point2f v1 = marker.at(1) - marker.at(0);
				cv::Point2f v2 = marker.at(2) - marker.at(0);
				double o = (v1.x * v2.y) - (v1.y * v2.x);
				if (o < 0.0){
					std::swap(marker.at(1), marker.at(3));
				}
				possibleMarkersPoints.push_back(marker);
			}
		}
	}

	std::vector<std::pair<int, int> > closestCandidates;

	for (int i = 0; i < possibleMarkersPoints.size(); i++){
		std::vector<cv::Point2f> markerA = possibleMarkersPoints.at(i);
		for (int j = i + 1; j < possibleMarkersPoints.size(); j++){
			std::vector<cv::Point2f> markerB = possibleMarkersPoints.at(j);
			float distSquared = 0;
			for (int k = 0; k < CURVE_SIZE; k++){
				cv::Point v = markerA.at(k) - markerB.at(k);
				distSquared += v.dot(v);
				
			}
			distSquared /= 4;
			if (distSquared < 20000){
				closestCandidates.push_back(std::pair<int, int>(i, j));
			}
		}
	}
	std::vector<bool> removalMask(possibleMarkersPoints.size(), false);
	for (int i = 0; i < closestCandidates.size(); i++){
		float p1 = perimeter(possibleMarkersPoints.at(closestCandidates.at(i).first));
		float p2 = perimeter(possibleMarkersPoints.at(closestCandidates.at(i).second));

		int index;
		if (p1 > p2){
			index = closestCandidates.at(i).first;
		} else {
			index = closestCandidates.at(i).second;
		}
		removalMask.at(index) = true;
	}
	markerPoints.clear();
	for (int i = 0; i < possibleMarkersPoints.size(); i++){
		if (!removalMask[i])
			markerPoints.push_back(possibleMarkersPoints[i]);
	}
}

void RNCameraTask::recognizeMarkers(const cv::Mat& inputGrayscale, std::vector<std::vector<cv::Point2f> >& markerPoints){
	std::vector<std::vector<cv::Point2f> > goodMarkersPoints;
	for (int i = 0; i < markerPoints.size(); i++){
		std::vector<cv::Point2f> marker = markerPoints.at(i);
		cv::Mat markerTransform = cv::getPerspectiveTransform(marker, markerCorners2d);

		cv::warpPerspective(inputGrayscale, canonicalMarkerImage, markerTransform, markerSize);
		std::ostringstream windowName;
		
		int rotations = 0;
		if (markerDecoder(canonicalMarkerImage, rotations) == 0){
			std::rotate(marker.begin(), marker.begin() + 4 - rotations, marker.end());
			goodMarkersPoints.push_back(marker);
		}
	}
	markerPoints = goodMarkersPoints;
}

void RNCameraTask::poseEstimation(std::vector<std::vector<cv::Point2f> >& markerPoints){
	for (size_t i = 0; i < markerPoints.size(); i++){
		std::vector<cv::Point2f> marker = markerPoints.at(i);
		cv::Mat Rvec;
		cv::Mat_<float> Tvec;
		cv::Mat raux, taux;
		
		cv::solvePnP(markerCorners2d, marker, camMatrix, distCoeff, raux, taux);
		raux.convertTo(Rvec, CV_32F);
		taux.convertTo(Tvec, CV_32F);
		cv::Mat_<float> rotMat(3, 3);
		cv::Rodrigues(Rvec, rotMat);

		for (int col = 0; col < 3; col++){
			for (int row = 0; row < 3; row++){
				//m.transformation.r().mat[row][col] = rotMat(row, col); // Copy rotation component
			}
			//m.transformation.t().data[col] = Tvec(col); // Copy translation component
		}

		// Since solvePnP finds camera location, w.r.t to marker pose, to get marker pose w.r.t to the camera we invert it.
		//m.transformation = m.transformation.getInverted();
	}
}

int RNCameraTask::markerDecoder(const cv::Mat& inputGrayscale, int& nRrotations){
	int result = 0;
	cv::Mat grey = inputGrayscale;
	cv::threshold(grey, grey, 127, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
	//cv::imshow("tikiti", grey);
	int cellHeigth = inputGrayscale.rows / CELL_MARKER_SIZE;
	int cellWidth = inputGrayscale.cols / CELL_MARKER_SIZE;
	for (int y = 0; y < CELL_MARKER_SIZE; y++){
		int inc = 6;
		if (y == 0 || y == 6){
			inc = 1;
		}

		for (int x = 0; x < CELL_MARKER_SIZE; x += inc){
			int cellX = x * cellWidth;
			int cellY = y * cellHeigth;
			cv::Mat cell = grey(cv::Rect(cellX, cellY, cellWidth, cellHeigth));
			//cv::imshow("tikiti2", cell);
			int nZ = cv::countNonZero(cell);
			if (nZ >(cellWidth*cellHeigth) / 2){
				result = -1;
			}
		}
	}

	if (result == 0){
		//RNUtils::printLn("pase por aqui.. todos zeros");
		cv::Mat bitMatrix = cv::Mat::zeros(5, 5, CV_8UC1);
		for (int y = 0; y < 5; y++){
			for (int x = 0; x < 5; x++){
				int cellX = (x + 1) * cellWidth;
				int cellY = (y + 1) * cellHeigth;
				cv::Mat cell = grey(cv::Rect(cellX, cellY, cellWidth, cellHeigth));
				//cv::imshow("tikiti2", cell);
				int nZ = cv::countNonZero(cell);
				if (nZ >(cellWidth*cellHeigth) / 2){
					bitMatrix.at<uchar>(y, x) = 1;
				}
			}
		}

		cv::Mat rotations[4];
		int distances[4];
		rotations[0] = bitMatrix;
		distances[0] = hammingDistance(bitMatrix);

		std::pair<int, int> minDist(distances[0], 0);

		for (int i = 1; i<4; i++)
		{
			//get the hamming distance to the nearest possible word
			rotations[i] = rotate(rotations[i - 1]);
			distances[i] = hammingDistance(rotations[i]);

			if (distances[i] < minDist.first)
			{
				minDist.first = distances[i];
				minDist.second = i;
			}
		}

		nRrotations = minDist.second;
		if (minDist.first != 0){
			result = -1;
		}
	}
	return result;
}

int RNCameraTask::hammingDistance(cv::Mat bits){
	int ids[4][5] = {
		{ 1, 0, 0, 0, 0 },
		{ 1, 0, 1, 1, 1 },
		{ 0, 1, 0, 0, 1 },
		{ 0, 1, 1, 1, 0 }
	};

	int dist = 0;

	for (int y = 0; y < 5; y++){
		int minSum = 1e5; //hamming distance to each possible word
		for (int p = 0; p < 4; p++){
			int sum = 0;
			//now, count
			for (int x = 0; x < 5; x++){
				sum += bits.at<uchar>(y, x) == ids[p][x] ? 0 : 1;
			}

			if (minSum > sum){
				minSum = sum;
			}
		}

		//do the and
		dist += minSum;
	}

	return dist;
}

cv::Mat RNCameraTask::rotate(cv::Mat input)
{
	cv::Mat out;
	input.copyTo(out);
	for (int i = 0; i<input.rows; i++)
	{
		for (int j = 0; j<input.cols; j++)
		{
			out.at<uchar>(i, j) = input.at<uchar>(input.cols - j - 1, i);
		}
	}
	return out;
}

float RNCameraTask::perimeter(const std::vector<cv::Point2f> &a){
	float result = 0, dx, dy;
	for (int i = 0; i < a.size(); i++){
		dx = a[i].x - a[(i + 1) % a.size()].x;
		dy = a[i].y - a[(i + 1) % a.size()].y;

		result += std::sqrt(dx * dx + dy * dy);
	}

	return result;
}

void RNCameraTask::task(){
	cv::Mat tiki;
	if(getFrameFromCamera(tiki) != RN_NONE){
		if (tiki.data){
			cv::Mat tikiGray, tikiThreshold;
			std::vector<std::vector<cv::Point2f> > markerTikiPoints;
			rgbToGrayscale(tiki, tikiGray);
			thresholding(tikiGray, tikiThreshold);
			findContours(tikiThreshold.clone(), contours, 100);
			findCandidates(contours, markerTikiPoints);
			recognizeMarkers(tikiGray, markerTikiPoints);
			RNUtils::printLn("markers: %d", markerTikiPoints.size());
			poseEstimation(markerTikiPoints);
		}
	} else {
		RNUtils::printLn("chuta y ahora?");
	}
}

void RNCameraTask::onKilled(){

}
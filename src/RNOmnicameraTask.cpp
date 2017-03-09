#include "RNOmnicameraTask.h"

const double RNOmnicameraTask::PI_DEGREES = 180.0;

const std::string RNOmnicameraTask::cameraUrl = "http://192.168.0.19/record/current.jpg";
//const std::string RNOmnicameraTask::cameraUrl = "http://admin:C0n7r01_au70@192.168.1.33/control/faststream.jpg?stream=full&fps=24&noaudio&data=v.mjpg";

RNOmnicameraTask::RNOmnicameraTask(const char* name, const char* description) : RNRecurrentTask(name, description){
	minContourLengthAllowed = 100.0;
	maxContourLengthAllowed = 4000.0;
	markerSize = cv::Size(215, 345);

	markerCorners2d.push_back(cv::Point2f(0, 0));
	markerCorners2d.push_back(cv::Point2f(markerSize.width - 1, 0));
	markerCorners2d.push_back(cv::Point2f(markerSize.width - 1, markerSize.height - 1));
	markerCorners2d.push_back(cv::Point2f(0, markerSize.height - 1));

	markerCorners3d.push_back(cv::Point3f(-.5f, -.5f, 0));
	markerCorners3d.push_back(cv::Point3f(.5f, -.5f, 0));
	markerCorners3d.push_back(cv::Point3f(.5f, .5f, 0));
	markerCorners3d.push_back(cv::Point3f(.5f, .5f, 0));

	camMatrix = cv::Mat(3, 3, CV_32F);
	camMatrix.at<float>(0, 0) = 2.6959417772420113e+002;
	camMatrix.at<float>(0, 1) = 0.0;
	camMatrix.at<float>(0, 2) = 5.1150000000000000e+002;
	camMatrix.at<float>(1, 0) = 0.0;
	camMatrix.at<float>(1, 1) = 2.6959417772420113e+002;
	camMatrix.at<float>(1, 2) = 3.8350000000000000e+002;
	camMatrix.at<float>(2, 0) = 0.0;
	camMatrix.at<float>(2, 1) = 0.0;
	camMatrix.at<float>(2, 2) = 1.0;

	distCoeff = cv::Mat(5, 1, CV_32F);
	distCoeff.at<float>(0, 0) = -2.6019586095779829e-001;
	distCoeff.at<float>(1, 0) = 5.5052401922323718e-002;
	distCoeff.at<float>(2, 0) = 0.0;
	distCoeff.at<float>(3, 0) = 0.0;
	distCoeff.at<float>(4, 0) = -4.5449850126361765e-003;

	landmarks = new std::vector<RNLandmark*>();
	
}

RNOmnicameraTask::~RNOmnicameraTask(){
	clearLandmarks();
	delete landmarks;
}

void RNOmnicameraTask::clearLandmarks(){
	for (int i = 0; i < landmarks->size(); i++){
		delete landmarks->at(i);
	}
	landmarks->clear();
}

size_t RNOmnicameraTask::write_data(char *ptr, size_t size, size_t nmemb, void *userdata) {
	std::ostringstream *stream = (std::ostringstream*)userdata;
	size_t count = size * nmemb;
	stream->write(ptr, count);
	return count;
}

int RNOmnicameraTask::getFrameFromCamera(cv::Mat &frame){
	int result = 0;
	/*cv::VideoCapture capture(cameraUrl);
	if (capture.isOpened()){
		capture.read(frame);
		capture.release();
	} else {
		result = RN_NONE;
	}*/
	CURL* curl;
	CURLcode res;
	std::ostringstream stream;
	curl = curl_easy_init();
	curl_easy_setopt(curl, CURLOPT_URL, cameraUrl.c_str());
	curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, &RNOmnicameraTask::write_data);
	curl_easy_setopt(curl, CURLOPT_WRITEDATA, &stream);
	res = curl_easy_perform(curl);
	std::string output = stream.str();
	curl_easy_cleanup(curl);
	std::vector<char> data = std::vector<char>(output.begin(), output.end());
	cv::Mat data_mat = cv::Mat(data);
	frame = cv::imdecode(data_mat, 1);
	return result;
}

void RNOmnicameraTask::rgbToGrayscale(const cv::Mat& input, cv::Mat& output){
	cv::cvtColor(input, output, CV_BGR2GRAY);
}

void RNOmnicameraTask::thresholding(const cv::Mat& inputGrayscale, cv::Mat& output){
	int thresholdValue = 100;
	cv::threshold(inputGrayscale, output, thresholdValue, 255, cv::THRESH_BINARY_INV);
}

void RNOmnicameraTask::clearNoisyDots(const cv::Mat input, cv::Mat& output){
	int erosionSize = 0;
	cv::Mat erosion;
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * erosionSize + 1, 2 * erosionSize + 1), cv::Point(erosionSize - 1, erosionSize - 1));
	erode(input, erosion, element);
	dilate(erosion, output, element);
}

void RNOmnicameraTask::findContours(const cv::Mat& input, std::vector<std::vector<cv::Point> > &contours, int minContourPointsAllowed){
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

void RNOmnicameraTask::findCandidates(const std::vector<std::vector<cv::Point> > &contours, std::vector<RNMarker>& markerPoints){
	std::vector<cv::Point>  approxCurve;
	std::vector<RNMarker> possibleMarkersPoints;
	for (unsigned int i = 0; i < contours.size(); i++){
		double eps = contours.at(i).size() * .1;
		cv::approxPolyDP(contours.at(i), approxCurve, eps, true);
		if (approxCurve.size() == CURVE_SIZE && cv::isContourConvex(approxCurve)){
			float minDist = std::numeric_limits<float>::max();
			for (int j = 0; j < approxCurve.size(); j++){
				cv::Point side = approxCurve.at(j) - approxCurve.at((j + 1) % 4);
				minDist = std::min(minDist, (float)side.dot(side));
			}
			
			if (minDist > this->minContourLengthAllowed){
				RNMarker marker;
				for (int j = 0; j < approxCurve.size(); j++){
					marker.addPoint(cv::Point2f(approxCurve.at(j).x, approxCurve.at(j).y));
				}
				
				cv::Point2f v1 = marker.getPoint(1) - marker.getPoint(0);
				cv::Point2f v2 = marker.getPoint(2) - marker.getPoint(0);
				double o = (v1.x * v2.y) - (v1.y * v2.x);
				if (o < 0.0){
					cv::Point2f auxPoint = marker.getPoint(1);
					marker.setPoint(marker.getPoint(3), 1);
					marker.setPoint(auxPoint, 3);
					//std::swap(marker.getPoint(1), marker.getPoint(3));
				}
				marker.setContourIdx(i);
				marker.setRotatedRect(cv::minAreaRect(approxCurve));
				marker.setArea(cv::contourArea(approxCurve));
				possibleMarkersPoints.push_back(marker);
			}
		}
	}

	std::vector<std::pair<int, int> > closestCandidates;

	for (int i = 0; i < possibleMarkersPoints.size(); i++){
		RNMarker markerA = possibleMarkersPoints.at(i);
		for (int j = i + 1; j < possibleMarkersPoints.size(); j++){
			RNMarker markerB = possibleMarkersPoints.at(j);
			float distSquared = 0;
			for (int k = 0; k < CURVE_SIZE; k++){
				cv::Point v = markerA.getPoint(k) - markerB.getPoint(k);
				distSquared += v.dot(v);
				
			}
			distSquared /= 4;
			if (distSquared < 100){
				closestCandidates.push_back(std::pair<int, int>(i, j));
			}
		}
	}
	std::vector<bool> removalMask(possibleMarkersPoints.size(), false);
	for (int i = 0; i < closestCandidates.size(); i++){
		float p1 = perimeter(possibleMarkersPoints.at(closestCandidates.at(i).first).getMarkerPoints());
		float p2 = perimeter(possibleMarkersPoints.at(closestCandidates.at(i).second).getMarkerPoints());

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

void RNOmnicameraTask::recognizeMarkers(const cv::Mat& inputGrayscale, std::vector<RNMarker>& markerPoints){
	std::vector<RNMarker> goodMarkersPoints;
	for (int i = 0; i < markerPoints.size(); i++){
		RNMarker marker = markerPoints.at(i);
		cv::Mat markerTransform = cv::getPerspectiveTransform(marker.getMarkerPoints(), markerCorners2d);

		cv::warpPerspective(inputGrayscale, canonicalMarkerImage, markerTransform, markerSize);
		std::ostringstream windowName;
		
		int rotations = 0;
		if (markerDecoder(canonicalMarkerImage, rotations) == 0){
			std::vector<cv::Point2f> markerPoints = marker.getMarkerPoints();
			std::rotate(markerPoints.begin(), markerPoints.begin() + 4 - rotations, markerPoints.end());
			marker.setMarkerPoints(markerPoints);
			goodMarkersPoints.push_back(marker);
		}
	}
	markerPoints = goodMarkersPoints;
}

void RNOmnicameraTask::poseEstimation(const cv::Point& center, std::vector<RNMarker>& markerPoints){
	for (size_t i = 0; i < markerPoints.size(); i++){
		RNMarker &marker = markerPoints.at(i);

		cv::Point markerCenter = marker.getRotatedRect().center;
		cv::Point tikiPoint = center - markerCenter;
		double angleInRadians = std::atan2(tikiPoint.y, tikiPoint.x) - (90 * M_PI / 180);
		if (angleInRadians > M_PI){
			angleInRadians = angleInRadians - 2*M_PI;
		} else if (angleInRadians < -M_PI) {
			angleInRadians = angleInRadians + 2*M_PI;
		}
		marker.setThRad(angleInRadians);
		//RNUtils::printLn("Marker (%d) angle: %lf", i, angleInRadians);
	}
}

int RNOmnicameraTask::markerDecoder(const cv::Mat& inputGrayscale, int& nRrotations){
	int result = 0;
	cv::Mat grey = inputGrayscale;
	cv::threshold(grey, grey, 127, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
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
			int nZ = cv::countNonZero(cell);
			if (nZ >(cellWidth*cellHeigth) / 2){
				result = -1;
			}
		}
	}

	if (result == 0){
		cv::Mat bitMatrix = cv::Mat::zeros(5, 5, CV_8UC1);
		for (int y = 0; y < 5; y++){
			for (int x = 0; x < 5; x++){
				int cellX = (x + 1) * cellWidth;
				int cellY = (y + 1) * cellHeigth;
				cv::Mat cell = grey(cv::Rect(cellX, cellY, cellWidth, cellHeigth));
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

		for (int i = 1; i<4; i++){
			//get the hamming distance to the nearest possible word
			rotations[i] = rotate(rotations[i - 1]);
			distances[i] = hammingDistance(rotations[i]);

			if (distances[i] < minDist.first){
				minDist.first = distances[i];
				minDist.second = i;
			}
		}

		nRrotations = minDist.second;
		if (minDist.first == 0){
            //int a = 0, b = 0, c = 0;
            //markerIdNumber(rotations[nRrotations], a, b, c);
        } else {
            result = -1;
        }
	}
	return result;
}

int RNOmnicameraTask::hammingDistance(cv::Mat bits){
	int ids[4][5] = {
		{ 1, 0, 0, 0, 1 },
		{ 1, 0, 1, 1, 1 },
		{ 1, 1, 0, 0, 1 },
		{ 0, 1, 0, 1, 0 }
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

cv::Mat RNOmnicameraTask::rotate(cv::Mat input)
{
	cv::Mat out;
	input.copyTo(out);
	for (int i = 0; i < input.rows; i++){
		for (int j = 0; j < input.cols; j++){
			out.at<uchar>(i, j) = input.at<uchar>(input.cols - j - 1, i);
		}
	}
	return out;
}

void RNOmnicameraTask::markerIdNumber(const cv::Mat &bits, int &mapId, int &sectorId, int &markerId){
    int count = 9;
    mapId = 0;
    sectorId = 0;
    markerId = 0;
    for(int j = 0; j < 2; j++){
        for(int i = 0; i < 5; i++){
            if(bits.at<uchar>(j, i)){
                mapId |= (1 << count);
            }
            count--;
        }
    }
    
    count = 9;
    for(int j = 2; j < 4; j++){
        for(int i = 0; i < 5; i++){
            if(bits.at<uchar>(j, i)){
                sectorId |= (1 << count);
            }
            count--;
        }
    }
    count = 4;
    for(int i = 0; i < 5; i++){
        if(bits.at<uchar>(4, i)){
            markerId |= (1 << count);
        }
        count--;
    }
}

float RNOmnicameraTask::perimeter(const std::vector<cv::Point2f> &a){
	float result = 0, dx, dy;
	for (int i = 0; i < a.size(); i++){
		dx = a[i].x - a[(i + 1) % a.size()].x;
		dy = a[i].y - a[(i + 1) % a.size()].y;

		result += std::sqrt(dx * dx + dy * dy);
	}

	return result;
}

void RNOmnicameraTask::task(){
	cv::Mat tiki;
	if(getFrameFromCamera(tiki) != RN_NONE){
		if (tiki.data){
			cv::Point imageCenter(tiki.cols / 2 + IMAGE_OFFSET_X, tiki.rows / 2 + IMAGE_OFFSET_Y);
			cv::Mat tikiGray, tikiThreshold;
			std::vector<RNMarker> tikiMarkers;
			rgbToGrayscale(tiki, tikiGray);
			thresholding(tikiGray, tikiThreshold);
			findContours(tikiThreshold.clone(), contours, 35);
			findCandidates(contours, tikiMarkers);
			recognizeMarkers(tikiGray, tikiMarkers);
			//RNUtils::printLn("markers: %d", tikiMarkers.size());
			poseEstimation(imageCenter, tikiMarkers);

			clearLandmarks();
			for(size_t i = 0; i < tikiMarkers.size(); i++){
				//RNLandmark* visualLand = new RNLandmark();
				//RNUtils::printLn("Marker (%d) angle: %lf", i, tikiMarkers.at(i).getThRad());
				//visualLand->addPoint(0, tikiMarkers.at(i).getThRad());
				//landmarks->push_back(visualLand);
			}
			gn->setVisualLandmarks(landmarks);
		}
	} else {
		RNUtils::printLn("chuta y ahora?");
	}
}

void RNOmnicameraTask::onKilled(){

}
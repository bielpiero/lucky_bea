#include "RNOmnicameraTask.h"

const double RNOmnicameraTask::PI_DEGREES = 180.0;

const std::string RNOmnicameraTask::cameraUrl = "http://192.168.0.19/record/current.jpg";
//const std::string RNOmnicameraTask::cameraUrl = "http://admin:C0n7r01_au70@192.168.1.33/control/faststream.jpg?stream=full&fps=24&noaudio&data=v.mjpg";

RNOmnicameraTask::RNOmnicameraTask(const GeneralController* gn, const char* name, const char* description) : RNRecurrentTask(gn, name, description){
	this->gn = (GeneralController*)gn;
	//RNUtils::printLn(".......................%s", this->gn->getClassName());
	curl = curl_easy_init();
	curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 1);
	curl_easy_setopt(curl, CURLOPT_URL, cameraUrl.c_str());
	curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, &RNOmnicameraTask::write_data);
	curl_easy_setopt(curl, CURLOPT_WRITEDATA, &cameraStream);

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
	camMatrix.at<float>(0, 0) = 3.3148337972624245e+02;
	camMatrix.at<float>(0, 1) = 0;
	camMatrix.at<float>(0, 2) = 6.5050896530720797e+02;
	camMatrix.at<float>(1, 0) = 0.0;
	camMatrix.at<float>(1, 1) = 3.3296507853901846e+02;
	camMatrix.at<float>(1, 2) = 4.9324794942591592e+02;
	camMatrix.at<float>(2, 0) = 0.0;
	camMatrix.at<float>(2, 1) = 0.0;
	camMatrix.at<float>(2, 2) = 1.0;

	distCoeff = cv::Mat(4, 1, CV_32F);
	distCoeff.at<float>(0, 0) = -5.0278669230113635e-02;
  	distCoeff.at<float>(1, 0) = 2.7927571053875219e-02;
  	distCoeff.at<float>(2, 0) = -9.7303697830329119e-03;
  	distCoeff.at<float>(3, 0) = 0;

	landmarks = NULL;

	enableVideoProcessing = true;
}

RNOmnicameraTask::~RNOmnicameraTask(){
	curl_easy_cleanup(curl);
	
}

size_t RNOmnicameraTask::write_data(char *ptr, size_t size, size_t nmemb, void *userdata) {
	std::ostringstream *stream = (std::ostringstream*)userdata;
	size_t count = size * nmemb;
	stream->write(ptr, count);
	return count;
}

int RNOmnicameraTask::getFrameFromCamera(cv::Mat &frame){
	int result = 0;
	data.clear();
	cameraStream.str("");
	cameraStream.clear();

	res = curl_easy_perform(curl);
	if(res == CURLE_OK){
		std::string strCameraStream = cameraStream.str();
		std::copy(strCameraStream.begin(), strCameraStream.end(), std::back_inserter(data));
	}
	if(data.size() > 0){
		cv::Mat data_mat;
		try{
			data_mat = cv::Mat(data);	
		} catch(cv::Exception& e1){
			RNUtils::printLn("Couldn't do convertion from data array to mat: %s", e1.what());
		}
		
		try{
			frame = cv::Mat(cv::imdecode(data_mat, 1));
		} catch(cv::Exception& e2){
			RNUtils::printLn("Couldn't decode image: %s", e2.what());
		}
	}
	if(not enableVideoProcessing){
		RNUtils::printLn("...........................FinishedCapture...........................");	
	}
	
	return result;
}

void RNOmnicameraTask::rgbToGrayscale(const cv::Mat& input, cv::Mat& output){
	cv::cvtColor(input, output, CV_BGR2GRAY);
}

void RNOmnicameraTask::thresholding(const cv::Mat& inputGrayscale, cv::Mat& output){
	int avgBrightness = getBrightnessAverageFromHistogram(inputGrayscale);
  	float index = 75.0 / 255.0 * (float)avgBrightness;
  	//RNUtils::printLn("[Brightness avg: %d, Index: %f]", avgBrightness, index);
	//int thresholdValue = 100;
	//cv::threshold(inputGrayscale, output, thresholdValue, 255, cv::THRESH_BINARY_INV);
	cv::adaptiveThreshold(inputGrayscale, output, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY_INV, BLOCK_SIZE_FOR_ADAPTIVE_THRESHOLD, (int)index);
	//cv::imwrite("threshold.jpg", output);
}

void RNOmnicameraTask::clearNoisyDots(const cv::Mat input, cv::Mat& output){
	int erosionSize = 0;
	cv::Mat erosion;
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * erosionSize + 1, 2 * erosionSize + 1), cv::Point(erosionSize - 1, erosionSize - 1));
	erode(input, erosion, element);
	dilate(erosion, output, element);
}

void RNOmnicameraTask::findContours(int minContourPointsAllowed){
	std::vector<std::vector<cv::Point> > allContours;
	std::vector<std::vector<cv::Point> > contours;

	cv::Canny(tikiThreshold, edges, 100, 180, 5);
	try{
		cv::findContours(edges, allContours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);	
	} catch (const std::bad_alloc& e){
		RNUtils::printLn("Allocation failed: %s", e.what());
	}
	
	edges.release();
	//cv::findContours(edges, allContours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
	
	for (unsigned int i = 0; i < allContours.size(); i++){
		if (allContours.at(i).size() > minContourPointsAllowed){
			contours.push_back(allContours.at(i));
		}
	}
	this->contours = contours;
}

void RNOmnicameraTask::findCandidates(){
	std::vector<cv::Point>  approxCurve;
	possibleMarkersPoints.clear();
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
	tikiMarkers.clear();
	for (int i = 0; i < possibleMarkersPoints.size(); i++){
		if (!removalMask[i])
			tikiMarkers.push_back(possibleMarkersPoints[i]);
	}
}

void RNOmnicameraTask::recognizeMarkers(){
	std::vector<RNMarker> goodMarkersPoints;
	for (int i = 0; i < tikiMarkers.size(); i++){
		RNMarker marker = tikiMarkers.at(i);
		cv::Mat markerTransform = cv::getPerspectiveTransform(marker.getMarkerPoints(), markerCorners2d);

		cv::warpPerspective(tikiGray, canonicalMarkerImage, markerTransform, markerSize);
		std::ostringstream windowName;
		
		int rotations = 0;
		if (markerDecoder(canonicalMarkerImage, rotations, marker) == 0){
			std::vector<cv::Point2f> markerDots = marker.getMarkerPoints();
			std::rotate(markerDots.begin(), markerDots.begin() + 4 - rotations, markerDots.end());
			marker.setMarkerPoints(markerDots);
			goodMarkersPoints.push_back(marker);
		}
	}
	tikiMarkers = goodMarkersPoints;
}

void RNOmnicameraTask::poseEstimation(){
	for (size_t i = 0; i < tikiMarkers.size(); i++){
		RNMarker &marker = tikiMarkers.at(i);
		std::vector<cv::Point2f> distortedPoint;
		std::vector<cv::Point2f> undistortedPoint;
		cv::Point imageCenter(tiki.cols / 2 + IMAGE_OFFSET_X, tiki.rows / 2 + IMAGE_OFFSET_Y);
		cv::Point markerCenter = marker.getRotatedRect().center;

		cv::Point tikiPoint = imageCenter - markerCenter;
		distortedPoint.push_back(cv::Point2f(marker.getRotatedRect().center.x, marker.getRotatedRect().center.y));
		undistortPoints(distortedPoint, undistortedPoint, camMatrix, distCoeff, cv::Mat::eye(3, 3, CV_32F), camMatrix);

		float distanceToCenter = 0;
		float theta = 0;
		float realWorldDistance = 0;

		distanceToCenter = sqrt(pow(abs(undistortedPoint.at(0).x - camMatrix.at<float>(0, 2)),2) + std::pow(abs(undistortedPoint.at(0).y - camMatrix.at<float>(1, 2)),2));

		theta = std::atan(distanceToCenter / camMatrix.at<float>(0, 0));
		marker.setOpticalTheta(theta);
		//realWorldDistance = MARKER_HEIGHT * std::tan(theta);

		//std::cout << "Real world distance: " << realWorldDistance << " cm, should be " << cmCounter << " cm. Error: " << abs(realWorldDistance - cmCounter) << " cm" << std::endl;

	
		double angleInRadians = std::atan2(tikiPoint.y, tikiPoint.x) - (M_PI / 2);

		if(angleInRadians > M_PI){
			angleInRadians = angleInRadians - 2 * M_PI;
		} else if(angleInRadians < -M_PI){
			angleInRadians = angleInRadians + 2 * M_PI;
		}
		marker.setThRad(angleInRadians);
		//RNUtils::printLn("Marker (%d) angle: %lf", i, angleInRadians);
	}

}

int RNOmnicameraTask::markerDecoder(const cv::Mat& inputGrayscale, int& nRrotations, RNMarker &marker){
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

		cv::Mat rotations[CURVE_SIZE];
		int distances[CURVE_SIZE];
		rotations[0] = bitMatrix;
		distances[0] = hammingDistance(bitMatrix);

		std::pair<int, int> minDist(distances[0], 0);

		for (int i = 1; i < CURVE_SIZE; i++){
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
            int mapId = 0, sectorId = 0, markerId = 0;
            markerIdNumber(rotations[nRrotations], mapId, sectorId, markerId);
            marker.setMapId(mapId);
            marker.setSectorId(sectorId);
            marker.setMarkerId(markerId);
        } else {
            result = -1;
        }
	}
	return result;
}

int RNOmnicameraTask::hammingDistance(cv::Mat bits){
	int ids[2][5] = {
		{ 1, 0, 0, 0, 1 },
		{ 1, 0, 1, 1, 1 }
	};
	
	int dist = 0;
	int sum = 0;

	//Compares the first and last row of the bit matrix with the template matrix ids

	for (int x = 0; x < 5; x++){
		sum += bits.at<uchar>(0, x) == ids[0][x] ? 0 : 1;
	}

	if (1e5 > sum){
		dist = sum;
	}

	sum = 0;

	for (int x = 0; x < 5; x++){
		sum += bits.at<uchar>(4, x) == ids[1][x] ? 0 : 1;
	}

	dist += sum;

	return dist;
}

cv::Mat RNOmnicameraTask::rotate(cv::Mat input){
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
    
    for (int j = 0; j < 5; j++){
    	if(bits.at<uchar>(1, j)){
    		mapId += (16 / std::pow(2, j)) * bits.at<uchar>(1, j);
    	}
    }
    for (int j = 0; j < 5; j++){
    	if(bits.at<uchar>(2, j)){
    		sectorId += (16 / std::pow(2, j)) * bits.at<uchar>(2, j);
    	}
    }
    for (int j = 0; j < 5; j++){
    	if(bits.at<uchar>(3, j)){
    		markerId += (16 / std::pow(2,j)) * bits.at<uchar>(3, j);
    	}
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

int RNOmnicameraTask::getBrightnessAverageFromHistogram(const cv::Mat& input){
	cv::Mat src = input;

	int histSize = 256;

	float range[] = { 0, 256 } ;
	const float* histRange = { range };

	bool uniform = true; bool accumulate = false;

	cv::Mat hist;

	cv::calcHist( &src, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, uniform, accumulate );

	int hist_w = 512; int hist_h = 400;
	int bin_w = cvRound( (double) hist_w/histSize );

	cv::Mat histImage( hist_h, hist_w, CV_8UC3, cv::Scalar(0, 0, 0) );

	cv::normalize(hist, hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );

	for( int i = 1; i < histSize; i++ ){
		line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(hist.at<float>(i-1)) ) ,
			cv::Point( bin_w*(i), hist_h - cvRound(hist.at<float>(i)) ),
			cv::Scalar( 0, 0, 255), 2, 8, 0  );
	}

	float avgBrightness = 0;

	for(int i =0; i< 256; i++){
	    avgBrightness += hist.at<float>(i)*i/cv::sum(hist)[0];  //Media ponderada
	}

	//std::cout << "Greyscale average brightness (0-255): " << (int)avgBrightness << std::endl;

	//cv::namedWindow("calcHist Demo", CV_WINDOW_AUTOSIZE );
	//cv::imwrite("calcHist Demo.jpg", histImage);
	//std::cout << "ahora.." << std::endl;
	//RNUtils::sleep(3000);
	//cv::imshow("calcHist Demo", histImage );

	return (int)avgBrightness;
}

void RNOmnicameraTask::undistortPoints(cv::InputArray distorted, cv::OutputArray undistorted, cv::InputArray K, cv::InputArray D, cv::InputArray R, cv::InputArray P){
    //CV_INSTRUMENT_REGION()

    // will support only 2-channel data now for points
    CV_Assert(distorted.type() == CV_32FC2 || distorted.type() == CV_64FC2);
    undistorted.create(distorted.size(), distorted.type());

    CV_Assert(P.empty() || P.size() == cv::Size(3, 3) || P.size() == cv::Size(4, 3));
    CV_Assert(R.empty() || R.size() == cv::Size(3, 3) || R.total() * R.channels() == 3);
    CV_Assert(D.total() == 4 && K.size() == cv::Size(3, 3) && (K.depth() == CV_32F || K.depth() == CV_64F));

    cv::Vec2d f, c;
    if (K.depth() == CV_32F){
        cv::Matx33f camMat = K.getMat();
        f = cv::Vec2f(camMat(0, 0), camMat(1, 1));
        c = cv::Vec2f(camMat(0, 2), camMat(1, 2));
    } else {
        cv::Matx33d camMat = K.getMat();
        f = cv::Vec2d(camMat(0, 0), camMat(1, 1));
        c = cv::Vec2d(camMat(0, 2), camMat(1, 2));
    }

    cv::Vec4d k = D.depth() == CV_32F ? (cv::Vec4d)*D.getMat().ptr<cv::Vec4f>(): *D.getMat().ptr<cv::Vec4d>();

    cv::Matx33d RR = cv::Matx33d::eye();
    if (!R.empty() && R.total() * R.channels() == 3)
    {
        cv::Vec3d rvec;
        R.getMat().convertTo(rvec, CV_64F);
        RR = cv::Affine3d(rvec).rotation();
    }
    else if (!R.empty() && R.size() == cv::Size(3, 3))
        R.getMat().convertTo(RR, CV_64F);

    if(!P.empty())
    {
        cv::Matx33d PP;
        P.getMat().colRange(0, 3).convertTo(PP, CV_64F);
        RR = PP * RR;
    }

    // start undistorting
    const cv::Vec2f* srcf = distorted.getMat().ptr<cv::Vec2f>();
    const cv::Vec2d* srcd = distorted.getMat().ptr<cv::Vec2d>();
    cv::Vec2f* dstf = undistorted.getMat().ptr<cv::Vec2f>();
    cv::Vec2d* dstd = undistorted.getMat().ptr<cv::Vec2d>();

    size_t n = distorted.total();
    int sdepth = distorted.depth();

    for(size_t i = 0; i < n; i++ )
    {
        cv::Vec2d pi = sdepth == CV_32F ? (cv::Vec2d)srcf[i] : srcd[i];  // image point
        cv::Vec2d pw((pi[0] - c[0])/f[0], (pi[1] - c[1])/f[1]);      // world point

        float scale = 1.0;

        float theta_d = sqrt(pw[0]*pw[0] + pw[1]*pw[1]);

        // the current camera model is only valid up to 180° FOV
        // for larger FOV the loop below does not converge
        // clip values so we still get plausible results for super fisheye images > 180°
        theta_d = min(max(-CV_PI/2., theta_d), CV_PI/2.);

        if (theta_d > 1e-8)
        {
            // compensate distortion iteratively
            float theta = theta_d;
            for(int j = 0; j < 10; j++ )
            {
                float theta2 = theta*theta, theta4 = theta2*theta2, theta6 = theta4*theta2, theta8 = theta6*theta2;
                theta = theta_d / (1 + k[0] * theta2 + k[1] * theta4 + k[2] * theta6 + k[3] * theta8);
            }

            scale = std::tan(theta) / theta_d;
        }

        cv::Vec2d pu = pw * scale; //undistorted point

        // reproject
        cv::Vec3d pr = RR * cv::Vec3d(pu[0], pu[1], 1.0); // rotated point optionally multiplied by new camera matrix
        cv::Vec2d fi(pr[0]/pr[2], pr[1]/pr[2]);       // final

        if( sdepth == CV_32F )
            dstf[i] = fi;
        else
            dstd[i] = fi;
    }
}

void RNOmnicameraTask::task(){
	//gn->lockSensorsReadings();
	if(enableVideoProcessing and getFrameFromCamera(tiki) != RN_NONE){

		if (tiki.data){
			//cv::imwrite("real-1.4m.jpg", flipped);
			rgbToGrayscale(tiki, tikiGray);
			//cv::imwrite("real-gray.jpg", tikiGray);
			thresholding(tikiGray, tikiThreshold);
			//contours.clear();
			findContours(35);
			tikiMarkers.clear();
			findCandidates();
			recognizeMarkers();
			//RNUtils::printLn("markers: %d", tikiMarkers.size());
			poseEstimation();
			landmarks = gn->getVisualLandmarks();
			landmarks->clear();
			//cv::Mat rectImage = flipped.clone();
			for(size_t i = 0; i < tikiMarkers.size(); i++){
				//drawRectangle(rectImage, tikiMarkers[i]);
				//cv::imwrite("que ves.jpg", rectImage);
				
				RNLandmark* visualLand = new RNLandmark();
				
				visualLand->addPoint(0, tikiMarkers.at(i).getThRad());
				visualLand->setMapId(tikiMarkers.at(i).getMapId());
				visualLand->setSectorId(tikiMarkers.at(i).getSectorId());
				visualLand->setMarkerId(tikiMarkers.at(i).getMarkerId());
				visualLand->addExtraParameter(OPTICAL_THETA_STR, tikiMarkers.at(i).getOpticalTheta());
				landmarks->add(visualLand);
			}
			//RNUtils::printLn("%s", landmarks->toString());
			gn->setVisualLandmarks(landmarks);
		}
	} else {
		RNUtils::printLn("chuta y ahora?");
	}
	//gn->unlockSensorsReadings();
}

void RNOmnicameraTask::drawRectangle(cv::Mat &img, RNMarker marker){
    cv::line(img, marker.getPoint(0), marker.getPoint(1), cv::Scalar(255, 0, 0), 3);
    cv::line(img, marker.getPoint(1), marker.getPoint(2), cv::Scalar(255, 0, 0), 3);
    cv::line(img, marker.getPoint(2), marker.getPoint(3), cv::Scalar(255, 0, 0), 3);
    cv::line(img, marker.getPoint(3), marker.getPoint(0), cv::Scalar(255, 0, 0), 3);
}

void RNOmnicameraTask::onKilled(){
	enableVideoProcessing = false;
}
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
	camMatrix.at<float>(0, 0) = 8.5101024687735935e+02;
	camMatrix.at<float>(0, 1) = -2.2255059056366439e-01;
	camMatrix.at<float>(0, 2) = 6.5571465382877625e+02;
	camMatrix.at<float>(1, 0) = 0.0;
	camMatrix.at<float>(1, 1) = 8.5170243585411265e+02;;
	camMatrix.at<float>(1, 2) = 5.1216084358475405e+02;
	camMatrix.at<float>(2, 0) = 0.0;
	camMatrix.at<float>(2, 1) = 0.0;
	camMatrix.at<float>(2, 2) = 1.0;

	distCoeff = cv::Mat(4, 1, CV_32F);
	distCoeff.at<float>(0, 0) = -4.2648301140911193e-01;
  	distCoeff.at<float>(1, 0) = 3.1105618959437248e-01;
  	distCoeff.at<float>(2, 0) = -1.3775384616268102e-02;
  	distCoeff.at<float>(3, 0) = -1.9560559208606078e-03;

	landmarks = new std::vector<RNLandmark*>();

	xi = cv::Mat(1, 1, CV_32FC1);
	xi.at<float>(0, 0) = 1.5861076761699640e+00;
	
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
	int avgBrightness = getBrightnessAverageFromHistogram(inputGrayscale);
  	float index = 75.0 / 255.0 * avgBrightness;
  	//RNUtils::printLn("[Brightness avg: %d, Index: %f]", avgBrightness, index);
	//int thresholdValue = 100;
	//cv::threshold(inputGrayscale, output, thresholdValue, 255, cv::THRESH_BINARY_INV);
	cv::adaptiveThreshold(inputGrayscale, output, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY_INV, 75, (int)index);
	//cv::imwrite("threshold.jpg", output);
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

void RNOmnicameraTask::poseEstimation(std::vector<RNMarker>& markerPoints){
	for (size_t i = 0; i < markerPoints.size(); i++){
		RNMarker &marker = markerPoints.at(i);
		cv::Point markerCenter = marker.getRotatedRect().center;
		double angleInRadians = RNUtils::linearInterpolator((float)markerCenter.x, PointXY(0, 0), PointXY((float)RECTIFIED_IMAGE_WIDTH, 2 * M_PI)) - M_PI/2;
		//double angleInRadians = (markerCenter.x * 2 * M_PI / RECTIFIED_IMAGE_WIDTH) - M_PI/2;
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

void RNOmnicameraTask::initUndistortRectifyMap(cv::InputArray K, cv::InputArray D, cv::InputArray xi, cv::InputArray R, cv::InputArray P, const cv::Size& size,
        int m1type, cv::OutputArray map1, cv::OutputArray map2, int flags){
	CV_Assert( m1type == CV_16SC2 || m1type == CV_32F || m1type <=0 );
    map1.create( size, m1type <= 0 ? CV_16SC2 : m1type );
    map2.create( size, map1.type() == CV_16SC2 ? CV_16UC1 : CV_32F );

    CV_Assert((K.depth() == CV_32F || K.depth() == CV_64F) && (D.depth() == CV_32F || D.depth() == CV_64F));
    CV_Assert(K.size() == cv::Size(3, 3) && (D.empty() || D.total() == 4));
    CV_Assert(P.empty()|| (P.depth() == CV_32F || P.depth() == CV_64F));
    CV_Assert(P.empty() || P.size() == cv::Size(3, 3) || P.size() == cv::Size(4, 3));
    CV_Assert(R.empty() || (R.depth() == CV_32F || R.depth() == CV_64F));
    CV_Assert(R.empty() || R.size() == cv::Size(3, 3) || R.total() * R.channels() == 3);
    CV_Assert(flags == RECTIFY_PERSPECTIVE || flags == RECTIFY_CYLINDRICAL || flags == RECTIFY_LONGLATI
        || flags == RECTIFY_STEREOGRAPHIC);
    CV_Assert(xi.total() == 1 && (xi.depth() == CV_32F || xi.depth() == CV_64F));

    cv::Vec2d f, c;
    double s;
    if (K.depth() == CV_32F)
    {
        cv::Matx33f camMat = K.getMat();
        f = cv::Vec2f(camMat(0, 0), camMat(1, 1));
        c = cv::Vec2f(camMat(0, 2), camMat(1, 2));
        s = (double)camMat(0, 1);
    }
    else
    {
        cv::Matx33d camMat = K.getMat();
        f = cv::Vec2d(camMat(0, 0), camMat(1, 1));
        c = cv::Vec2d(camMat(0, 2), camMat(1, 2));
        s = camMat(0, 1);
    }

    cv::Vec4d kp = cv::Vec4d::all(0);
    if (!D.empty())
        kp = D.depth() == CV_32F ? (cv::Vec4d)*D.getMat().ptr<cv::Vec4f>(): *D.getMat().ptr<cv::Vec4d>();
    double _xi = xi.depth() == CV_32F ? (double)*xi.getMat().ptr<float>() : *xi.getMat().ptr<double>();
    cv::Vec2d k = cv::Vec2d(kp[0], kp[1]);
    cv::Vec2d p = cv::Vec2d(kp[2], kp[3]);
    cv::Matx33d RR  = cv::Matx33d::eye();
    if (!R.empty() && R.total() * R.channels() == 3)
    {
        cv::Vec3d rvec;
        R.getMat().convertTo(rvec, CV_64F);
        cv::Rodrigues(rvec, RR);
    }
    else if (!R.empty() && R.size() == cv::Size(3, 3))
        R.getMat().convertTo(RR, CV_64F);

    cv::Matx33d PP = cv::Matx33d::eye();
    if (!P.empty())
        P.getMat().colRange(0, 3).convertTo(PP, CV_64F);
    else
        PP = K.getMat();

    cv::Matx33d iKR = (PP*RR).inv(cv::DECOMP_SVD);
    cv::Matx33d iK = PP.inv(cv::DECOMP_SVD);
    cv::Matx33d iR = RR.inv(cv::DECOMP_SVD);

    if (flags == RECTIFY_PERSPECTIVE)
    {
        for (int i = 0; i < size.height; ++i)
        {
            float* m1f = map1.getMat().ptr<float>(i);
            float* m2f = map2.getMat().ptr<float>(i);
            short*  m1 = (short*)m1f;
            ushort* m2 = (ushort*)m2f;

            double _x = i*iKR(0, 1) + iKR(0, 2),
                   _y = i*iKR(1, 1) + iKR(1, 2),
                   _w = i*iKR(2, 1) + iKR(2, 2);
            for(int j = 0; j < size.width; ++j, _x+=iKR(0,0), _y+=iKR(1,0), _w+=iKR(2,0))
            {
                // project back to unit sphere
                double r = sqrt(_x*_x + _y*_y + _w*_w);
                double Xs = _x / r;
                double Ys = _y / r;
                double Zs = _w / r;
                // project to image plane
                double xu = Xs / (Zs + _xi),
                    yu = Ys / (Zs + _xi);
                // add distortion
                double r2 = xu*xu + yu*yu;
                double r4 = r2*r2;
                double xd = (1+k[0]*r2+k[1]*r4)*xu + 2*p[0]*xu*yu + p[1]*(r2+2*xu*xu);
                double yd = (1+k[0]*r2+k[1]*r4)*yu + p[0]*(r2+2*yu*yu) + 2*p[1]*xu*yu;
                // to image pixel
                double u = f[0]*xd + s*yd + c[0];
                double v = f[1]*yd + c[1];

                if( m1type == CV_16SC2 )
                {
                    int iu = cv::saturate_cast<int>(u*cv::INTER_TAB_SIZE);
                    int iv = cv::saturate_cast<int>(v*cv::INTER_TAB_SIZE);
                    m1[j*2+0] = (short)(iu >> cv::INTER_BITS);
                    m1[j*2+1] = (short)(iv >> cv::INTER_BITS);
                    m2[j] = (ushort)((iv & (cv::INTER_TAB_SIZE-1))*cv::INTER_TAB_SIZE + (iu & (cv::INTER_TAB_SIZE-1)));
                }
                else if( m1type == CV_32FC1 )
                {
                    m1f[j] = (float)u;
                    m2f[j] = (float)v;
                }
            }
        }
    }
    else if(flags == RECTIFY_CYLINDRICAL || flags == RECTIFY_LONGLATI ||
        flags == RECTIFY_STEREOGRAPHIC)
    {
        for (int i = 0; i < size.height; ++i)
        {
            float* m1f = map1.getMat().ptr<float>(i);
            float* m2f = map2.getMat().ptr<float>(i);
            short*  m1 = (short*)m1f;
            ushort* m2 = (ushort*)m2f;

            // for RECTIFY_LONGLATI, theta and h are longittude and latitude
            double theta = i*iK(0, 1) + iK(0, 2),
                   h     = i*iK(1, 1) + iK(1, 2);

            for (int j = 0; j < size.width; ++j, theta+=iK(0,0), h+=iK(1,0))
            {
                double _xt = 0.0, _yt = 0.0, _wt = 0.0;
                if (flags == RECTIFY_CYLINDRICAL)
                {
                    //_xt = std::sin(theta);
                    //_yt = h;
                    //_wt = std::cos(theta);
                    _xt = std::cos(theta);
                    _yt = std::sin(theta);
                    _wt = h;
                }
                else if (flags == RECTIFY_LONGLATI)
                {
                    _xt = -std::cos(theta);
                    _yt = -std::sin(theta) * std::cos(h);
                    _wt = std::sin(theta) * std::sin(h);
                }
                else if (flags == RECTIFY_STEREOGRAPHIC)
                {
                    double a = theta*theta + h*h + 4;
                    double b = -2*theta*theta - 2*h*h;
                    double c2 = theta*theta + h*h -4;

                    _yt = (-b-std::sqrt(b*b - 4*a*c2))/(2*a);
                    _xt = theta*(1 - _yt) / 2;
                    _wt = h*(1 - _yt) / 2;
                }
                double _x = iR(0,0)*_xt + iR(0,1)*_yt + iR(0,2)*_wt;
                double _y = iR(1,0)*_xt + iR(1,1)*_yt + iR(1,2)*_wt;
                double _w = iR(2,0)*_xt + iR(2,1)*_yt + iR(2,2)*_wt;

                double r = sqrt(_x*_x + _y*_y + _w*_w);
                double Xs = _x / r;
                double Ys = _y / r;
                double Zs = _w / r;
                // project to image plane
                double xu = Xs / (Zs + _xi),
                       yu = Ys / (Zs + _xi);
                // add distortion
                double r2 = xu*xu + yu*yu;
                double r4 = r2*r2;
                double xd = (1+k[0]*r2+k[1]*r4)*xu + 2*p[0]*xu*yu + p[1]*(r2+2*xu*xu);
                double yd = (1+k[0]*r2+k[1]*r4)*yu + p[0]*(r2+2*yu*yu) + 2*p[1]*xu*yu;
                // to image pixel
                double u = f[0]*xd + s*yd + c[0];
                double v = f[1]*yd + c[1];

                if( m1type == CV_16SC2 )
                {
                    int iu = cv::saturate_cast<int>(u*cv::INTER_TAB_SIZE);
                    int iv = cv::saturate_cast<int>(v*cv::INTER_TAB_SIZE);
                    m1[j*2+0] = (short)(iu >> cv::INTER_BITS);
                    m1[j*2+1] = (short)(iv >> cv::INTER_BITS);
                    m2[j] = (ushort)((iv & (cv::INTER_TAB_SIZE-1))*cv::INTER_TAB_SIZE + (iu & (cv::INTER_TAB_SIZE-1)));
                }
                else if( m1type == CV_32FC1 )
                {
                    m1f[j] = (float)u;
                    m2f[j] = (float)v;
                }
            }
        }
    }
}

void RNOmnicameraTask::undistortImage(cv::InputArray distorted, cv::OutputArray undistorted, cv::InputArray K, cv::InputArray D, cv::InputArray xi, int flags,
        cv::InputArray knew, const cv::Size& newSize, cv::InputArray R){
	cv::Size size = newSize.area() != 0 ? newSize : distorted.size();

    cv::Mat map1, map2;
    initUndistortRectifyMap(K, D, xi, R, knew, size, CV_16SC2, map1, map2, flags);
    cv::remap(distorted, undistorted, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
}

void RNOmnicameraTask::task(){
	cv::Mat tiki, rectified, mapX, mapY, flipped;
	if(getFrameFromCamera(tiki) != RN_NONE){
		cv::Size newSize = cv::Size (RECTIFIED_IMAGE_WIDTH, RECTIFIED_IMAGE_HEIGHT);
		cv::Matx33f Knew = cv::Matx33f(newSize.width / (2 * M_PI), 0, 0, 0, newSize.height / M_PI, 0, 0, 0, 1);
		undistortImage(tiki, rectified, camMatrix, distCoeff, xi, RECTIFY_CYLINDRICAL, Knew, newSize);

		flipped.create(rectified.size(), rectified.type());
		mapX.create(flipped.size(), CV_32FC1);
		mapY.create(flipped.size(), CV_32FC1);

		for( int j = 0; j < newSize.height; j++ ){
			for( int i = 0; i < newSize.width; i++ ){
				mapX.at<float>(j,i) = newSize.width - i ;
				mapY.at<float>(j,i) = newSize.height - j ;                    
			}
		}

		//Corrects flipped image      
		cv::remap(rectified, flipped, mapX, mapY, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
		if (flipped.data){
			cv::Mat tikiGray, tikiThreshold;
			std::vector<RNMarker> tikiMarkers;
			cv::imwrite("real-1.4m.jpg", flipped);
			rgbToGrayscale(flipped, tikiGray);
			//cv::imwrite("real-gray.jpg", tikiGray);
			thresholding(tikiGray, tikiThreshold);
			findContours(tikiThreshold.clone(), contours, 35);
			findCandidates(contours, tikiMarkers);
			recognizeMarkers(tikiGray, tikiMarkers);
			//RNUtils::printLn("markers: %d", tikiMarkers.size());
			poseEstimation(tikiMarkers);

			clearLandmarks();
			cv::Mat rectImage = flipped.clone();
			for(size_t i = 0; i < tikiMarkers.size(); i++){
				drawRectangle(rectImage, tikiMarkers[i]);
				//cv::imwrite("que ves.jpg", rectImage);
				//RNLandmark* visualLand = new RNLandmark();
				RNUtils::printLn("Marker (%d) angle: %lf", i, tikiMarkers.at(i).getThRad());
				//visualLand->addPoint(0, tikiMarkers.at(i).getThRad());
				//landmarks->push_back(visualLand);
			}
			gn->setVisualLandmarks(landmarks);
		}
	} else {
		RNUtils::printLn("chuta y ahora?");
	}
}

void RNOmnicameraTask::drawRectangle(cv::Mat &img, RNMarker marker){
    cv::line(img, marker.getPoint(0), marker.getPoint(1), cv::Scalar(255, 0, 0), 3);
    cv::line(img, marker.getPoint(1), marker.getPoint(2), cv::Scalar(255, 0, 0), 3);
    cv::line(img, marker.getPoint(2), marker.getPoint(3), cv::Scalar(255, 0, 0), 3);
    cv::line(img, marker.getPoint(3), marker.getPoint(0), cv::Scalar(255, 0, 0), 3);
}

void RNOmnicameraTask::onKilled(){

}
#include "RNUtils.h"

bool RNUtils::status = false;
std::string RNUtils::applicationPath = "";
std::string RNUtils::applicationName = "";

bool RNUtils::virtualScenario = false;
std::string RNUtils::virtualScenarioPort = "";

bool RNUtils::virtualFace = false;
std::string RNUtils::virtualFaceIpPort = "";

const unsigned long RNUtils::PRINT_BUFFER_SIZE = std::pow(2, 16);

void RNUtils::init(int argc, char** argv){
    setStatus(true);
    setApplicationPathName(argv[0]);
    for(int i = 1; i < argc; i++){
    	if(strcmp(argv[i], "-vp") == 0){
    		virtualScenario = true;
    		virtualScenarioPort = std::string(argv[i + 1]);
    	} else if(strcmp(argv[i], "-vf") == 0){
    		virtualFace = true;
    		virtualFaceIpPort = std::string(argv[i + 1]);
    	}	
    }
}

bool RNUtils::isVirtualScenarioActivated(){
	return virtualScenario;
}

bool RNUtils::isVirtualFaceActivated(){
	return virtualFace;
}

std::string RNUtils::getVirtualScenarioPort(){
	return virtualScenarioPort;
}

void RNUtils::getVirtualFaceIpPort(std::string& ip, int& port){
	std::vector<std::string> ipPort = split((char*)virtualFaceIpPort.c_str(), ":");
	ip = "";
	port = RN_NONE;
	if(ipPort.size() > 1){
		ip = ipPort.at(0);
		port = atoi(ipPort.at(1).c_str());
	}
}

void RNUtils::printLn(const char* _format, ...){
	std::ostringstream timestamp;
	getTimestamp(timestamp);

	char* buffer = new char[PRINT_BUFFER_SIZE];
	sprintf(buffer, "[ DORIS] [%s]: ", timestamp.str().c_str());
	va_list arguments;
	va_start(arguments, _format);

	vsprintf(buffer + 13 + timestamp.str().size(), _format, arguments);

	printf("%s\n", buffer);
	va_end(arguments);
	delete[] buffer;
}

void RNUtils::sleep(int milliseconds){
	usleep(1000*milliseconds);
}

std::vector<std::string> RNUtils::split(std::string buffer, const char* delimiter){
	return split((char*)buffer.c_str(), delimiter);
}

std::vector<std::wstring> RNUtils::wsplit(std::wstring buffer, const wchar_t delimiter){
	std::vector<std::wstring> result;
	std::wstringstream current(buffer);
	std::wstring temp;
	while(std::getline(current, temp, delimiter)){
    	result.push_back(temp);
    }
	return result;
}

std::vector<std::string> RNUtils::split(char* buffer, const char* delimiter){
	std::vector<std::string> result;
	char* buffer_in = new char[strlen(buffer) + 1];
	memcpy(buffer_in, buffer, strlen(buffer) + 1);
	char* current;

	current = std::strtok(buffer_in, delimiter);

	while(current != NULL){
		result.push_back(std::string(current));
		current = std::strtok(NULL, delimiter);
	}
	delete current;
	delete[] buffer_in;
	return result;
}

void RNUtils::spin(){
	while(RNUtils::ok()){
		sleep(1);
	}
}

void RNUtils::spinOnce(){
	sleep(2000);
}

bool RNUtils::ok(){
	return RNUtils::status;
}

void RNUtils::shutdown(){
    setStatus(false);
}

void RNUtils::setStatus(bool m_status){
	RNUtils::status = m_status;
}

void RNUtils::setApplicationPathName(char* str){
	std::vector<std::string> path = split(str, "/");
    std::string appPath;
    if(path.size() > 0){
    	RNUtils::applicationName = path.at(path.size() - 1);
    	int i = 0;
    	if(path.at(0) == "."){
    		i = 1;
    	}
    	for(; i < path.size() - 1; i++){
    		if(i == 0){
    			appPath += "/";
    		}
    		appPath += path.at(i) + "/";
    		
    	}
    	RNUtils::applicationPath = appPath;
    }
    
}

std::string RNUtils::getApplicationPath(){
	return RNUtils::applicationPath;
}

std::string RNUtils::getApplicationName(){
	return RNUtils::applicationName;
}


void RNUtils::getTimestamp(std::ostringstream& timestamp){
	std::time_t ltTime = std::time(NULL);
	std::tm *tstamp = std::localtime(&ltTime);

	timestamp.str("");
	timestamp << tstamp->tm_year + 1900 << "-";
	if((tstamp->tm_mon + 1) < 10){
		timestamp << "0" << tstamp->tm_mon + 1 << "-";
	} else {
		timestamp << tstamp->tm_mon + 1 << "-";
	}

	if(tstamp->tm_mday < 10){
		timestamp << "0" << tstamp->tm_mday << " ";
	} else {
		timestamp << tstamp->tm_mday << " ";
	}

	if(tstamp->tm_hour < 10){
		timestamp << "0" << tstamp->tm_hour << ":";
	} else {
		timestamp << tstamp->tm_hour << ":";
	}

	if(tstamp->tm_min < 10){
		timestamp << "0" << tstamp->tm_min << ":";
	} else {
		timestamp << tstamp->tm_min << ":";
	}

	if(tstamp->tm_sec < 10){
		timestamp << "0" << tstamp->tm_sec;
	} else {
		timestamp << tstamp->tm_sec;
	}
}

void RNUtils::getBezierCurve(std::vector<PointXY> bezierPointXYs, std::vector<PointXY> &bezierCurve){
	int n = bezierPointXYs.size() - 1;
	for(double t = 0; t < 1.0; t+=0.01){
		double px = 0, py = 0;
		for(unsigned int j = 0; j < bezierPointXYs.size(); j++){
			int b = binomialCoeff(n, j);
			px += b * std::pow((1.0 - t), (n - j)) * std::pow(t, j) * bezierPointXYs.at(j).getX();
			py += b * std::pow((1.0 - t), (n - j)) * std::pow(t, j) * bezierPointXYs.at(j).getY();
		}
		bezierCurve.push_back(PointXY(px, py));
	}
}

int RNUtils::binomialCoeff(int n, int k){
	int result = 1;
	if(k == 0 or n == k){
		result = 1;
	} else if(k > n){
		result = 0;
	} else if(k == 1){
		result = n;
	} else {
		for(int i = 1; i <= k; i++){
			result *= (n -(k - i));
			if(result < 0){
				return -1;
			}
			result /= i;
		}
	}
	return result;
}

std::string RNUtils::toLowercase(std::string str){
	std::ostringstream newString;
	newString.str("");
	std::locale loc;
	for (int i = 0; i < str.length(); i++){
    	newString << std::tolower(str[i], loc);
	}
	return newString.str();
}

std::string RNUtils::toUppercase(std::string str){
	std::ostringstream newString;
	newString.str("");
	std::locale loc;
	for (int i = 0; i < str.length(); i++){
    	newString << std::toupper(str[i], loc);
	}
	return newString.str();
}

double RNUtils::milliwattsTodBm(const double& milliwatts){
	return (10 * std::log10(milliwatts));
}

double RNUtils::dBmTomilliwatts(const double& dBm){
	return (std::pow(10, ((double)dBm) / 10));
}

double RNUtils::distanceTo(const double& x1, const double& y1, const double& x2, const double& y2){
	return std::sqrt(((x1 - x2) * (x1 - x2)) + ((y1 - y2) * (y1 - y2)));
}

double RNUtils::angleTo(const double& x1, const double& y1, const double& x2, const double& y2){
	return std::atan2((y1 - y2), (x1 - x2));
}

double RNUtils::deg2Rad(double degrees){
	double result = (degrees * M_PI / 180.0);
	return fixAngleRad(result);
}

double RNUtils::rad2Deg(double rad){
	rad = fixAngleRad(rad);
	return (rad * 180.0 / M_PI);
}

double RNUtils::fixAngleRad(double rad){
	if(rad > M_PI){
		while(rad > M_PI){
			rad = rad - 2 * M_PI;
		}
	} else if(rad < -M_PI){
		while(rad < -M_PI){
			rad = rad + 2 * M_PI;
		}
	}
	return rad;
}

void RNUtils::rotate(const double& x, const double& y, const double& angleRad, double* newx, double* newy){
	*newx = x * std::cos(angleRad) - y * std::sin(angleRad);
	*newy = x * std::sin(angleRad) + y * std::cos(angleRad);
}

void RNUtils::getOdometryPose(const double& xk, const double& yk, const double& thk, const double& deltaDistance, const double& deltaDegrees, double* xk1, double* yk1, double* thk1){
    //printf("Pre-Raw: %lf, %lf, %lf\n", xk, yk, thk);
    *xk1 = xk + deltaDistance * std::cos(thk + (deltaDegrees / 2.0));
    *yk1 = yk + deltaDistance * std::sin(thk + (deltaDegrees / 2.0));
    *thk1 = thk + deltaDegrees;
    *thk1 = fixAngleRad(*thk1);
    //printf("Raw: %lf, %lf, %lf\n", *xk1, *yk1, *thk1);
}

void RNUtils::getOdometryPose(const Matrix& posk, const Matrix& increment, Matrix& posk1){
	double x, y, th;
	getOdometryPose(posk(0, 0), posk(1, 0), posk(2, 0), increment(0, 0), increment(1, 0), &x, &y, &th);
	posk1(0, 0) = x;
	posk1(1, 0) = y;
	posk1(2, 0) = th;
}

void RNUtils::getOdometryPose(const Matrix& posk, const double& deltaDistance, const double& deltaDegrees, Matrix& posk1){
	double x, y, th;
	getOdometryPose(posk(0, 0), posk(1, 0), posk(2, 0), deltaDistance, deltaDegrees, &x, &y, &th);
	posk1(0, 0) = x;
	posk1(1, 0) = y;
	posk1(2, 0) = th;
}

void RNUtils::getOdometryPose(const ArPose& posk, const Matrix& increment, ArPose* posk1){
	double x, y, th;
	getOdometryPose(posk.getX(), posk.getY(), posk.getThRad(), increment(0, 0), increment(1, 0), &x, &y, &th);
	posk1->setX(x);
	posk1->setY(y);
	posk1->setThRad(th);

}

void RNUtils::getOdometryPose(const ArPose& posk, const double& deltaDistance, const double& deltaDegrees, ArPose* posk1){
	double x, y, th;
	getOdometryPose(posk.getX(), posk.getY(), posk.getThRad(), deltaDistance, deltaDegrees, &x, &y, &th);
	posk1->setX(x);
	posk1->setY(y);
	posk1->setThRad(th);

}

double RNUtils::linearInterpolator(const double& x, const PointXY& p1, const PointXY& p2){
	return (((x - p1.getX())/(p2.getX() - p1.getX()) * (p2.getY() - p1.getY())) + p1.getY());
}

double RNUtils::quadraticInterpolator(const double& x, const PointXY& p1, const PointXY& p2, const PointXY& p3){
	double term1 = (((x - p2.getX()) * (x - p3.getX()))/((p1.getX() - p2.getX()) * (p1.getX() - p3.getX()))) * p1.getY();
	double term2 = (((x - p1.getX()) * (x - p3.getX()))/((p2.getX() - p1.getX()) * (p2.getX() - p3.getX()))) * p2.getY();
	double term3 = (((x - p1.getX()) * (x - p2.getX()))/((p3.getX() - p1.getX()) * (p3.getX() - p2.getX()))) * p3.getY();
	return (term3 + term2 + term1);
}


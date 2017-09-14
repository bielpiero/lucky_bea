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
	for(float t = 0; t < 1.0; t+=0.01){
		float px = 0, py = 0;
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

double RNUtils::deg2Rad(double degrees){
	return (degrees * M_PI / 180.0);
}

double RNUtils::rad2Deg(double rad){
	return (rad * 180.0 / M_PI);
}

float RNUtils::linearInterpolator(const float& x, const PointXY& p1, const PointXY& p2){
	return (((x - p1.getX())/(p2.getX() - p1.getX()) * (p2.getY() - p1.getY())) + p1.getY());
}

float RNUtils::quadraticInterpolator(const float& x, const PointXY& p1, const PointXY& p2, const PointXY& p3){
	float term1 = (((x - p2.getX()) * (x - p3.getX()))/((p1.getX() - p2.getX()) * (p1.getX() - p3.getX()))) * p1.getY();
	float term2 = (((x - p1.getX()) * (x - p3.getX()))/((p2.getX() - p1.getX()) * (p2.getX() - p3.getX()))) * p2.getY();
	float term3 = (((x - p1.getX()) * (x - p2.getX()))/((p3.getX() - p1.getX()) * (p3.getX() - p2.getX()))) * p3.getY();
	return (term3 + term2 + term1);
}


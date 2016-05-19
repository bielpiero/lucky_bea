#include "RNUtils.h"

void RNUtils::printLn(const char* _format, ...){
	std::ostringstream timestamp;
	getTimestamp(timestamp);

	char buffer[14 + timestamp.str().size() + strlen(_format)];
	sprintf(buffer, "[ DORIS] [%s]: ", timestamp.str().c_str());
	va_list arguments;
	va_start(arguments, _format);

	vsprintf(buffer + 13 + timestamp.str().size(), _format, arguments);
	sprintf(buffer + strlen(buffer), "\n");

	printf("%s", buffer);
	va_end(arguments);
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
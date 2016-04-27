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

	char* current;

	current = std::strtok(buffer, delimiter);

	while(current != NULL){
		result.push_back(std::string(current));
		current = std::strtok(NULL, delimiter);
	}
	delete current;
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
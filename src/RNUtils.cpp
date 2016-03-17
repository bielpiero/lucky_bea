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
	timestamp << tstamp->tm_year + 1900 << "-" << tstamp->tm_mon + 1 << "-" << tstamp->tm_mday << " " << tstamp->tm_hour << ":" << tstamp->tm_min << ":" << tstamp->tm_sec;
}
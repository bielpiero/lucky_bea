#ifndef RN_UTILS_H
#define RN_UTILS_H

#include <sys/stat.h>
#include <sys/time.h>   
#include <sys/types.h>   
#include <unistd.h>
#include <pthread.h>
#include <errno.h>

#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <cmath>
#include <string>
#include <cstring>
#include <ctime>
#include <cstdio>
#include <cstdarg>

class RNUtils{
public:
	static void sleep(int milliseconds);
	static void printLn(const char* _format, ...);
	static std::vector<std::string> split(char* buffer, const char* delimiter);
	static void getTimestamp(std::ostringstream& timestamp);
};

#endif
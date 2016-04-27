#ifndef RN_UTILS_H
#define RN_UTILS_H

#include <sys/stat.h>
#include <sys/time.h>   
#include <sys/types.h>   
#include <unistd.h>
#include <pthread.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <vector>
#include <list>
#include <fstream>
#include <sstream>
#include <cmath>
#include <string>
#include <cstring>
#include <ctime>
#include <cstdio>
#include <cstdarg>
#include <cassert>

class RNUtils{
public:
	static void sleep(int milliseconds);
	static void printLn(const char* _format, ...);
	static std::vector<std::string> split(char* buffer, const char* delimiter);
	static void getTimestamp(std::ostringstream& timestamp);
};

#endif
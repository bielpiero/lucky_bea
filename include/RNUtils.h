#ifndef RN_UTILS_H
#define RN_UTILS_H

#include <sys/stat.h>
#include <sys/time.h>   
#include <sys/types.h>   
#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <cerrno>
#include <vector>
#include <list>
#include <fstream>
#include <sstream>
#include <cmath>
#include <limits>
#include <string>
#include <cstring>
#include <ctime>
#include <cstdio>
#include <cstdlib>
#include <cstdarg>
#include <cassert>
#include <locale>
#include <algorithm>

#include "Aria.h"



#define RN_NONE -1

#define YES 1
#define NO 0
#define MAYBE 2

class PointXY{
private:
	double x, y;
public:
	PointXY(){
		this->x = 0.0;
		this->y = 0.0;
	}
	PointXY(double x, double y){
		this->x = x;
		this->y = y;
	}
	~PointXY(){}

	double getX(){ return this->x; }
	double getY(){ return this->y; }
	
	void setX(double x){ this->x = x; }
	void setY(double y){ this->y = y; }
};


class RNUtils{
private:
	static bool status;
	static std::string applicationPath;
	static std::string applicationName;
    static const unsigned long PRINT_BUFFER_SIZE;
public: // functions
    static void init(int argc, char** argv);
	static void sleep(int milliseconds);
	static void printLn(const char* _format, ...);
	static std::vector<std::string> split(char* buffer, const char* delimiter);
	static void getTimestamp(std::ostringstream& timestamp);
	static void getBezierCurve(std::vector<PointXY> bezierPointXYs, std::vector<PointXY> &bezierCurve);
	static int binomialCoeff(int n, int k);
	static std::string toLowercase(std::string str);
	static std::string toUppercase(std::string str);
	static void spin();
	static bool ok();
    static void shutdown();
	static std::string getApplicationPath();
	static std::string getApplicationName();
private: //functions
    static void setStatus(bool status);
    static void setApplicationPathName(char* str);
};

#endif
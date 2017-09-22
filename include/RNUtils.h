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
#include <iterator>
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
#include <cstddef>
#include <locale>
#include <algorithm>


#include "Aria.h"

#define RN_DEFAULT_PORT "/dev/ttyS0"

#define RN_NONE -1

#define RN_YES 1
#define RN_NO 0
#define MAYBE 2

template <class T1, class T2, class T3>
class Trio{
private:
	T1 first;
	T2 second;
	T3 third;
public:
	Trio() : first(), second(), third() { }
	Trio(const T1 a, const T2 b, const T3 c) : first(a), second(b), third(c) { }
	Trio(const Trio<T1, T2, T3>& p) : first(p.first), second(p.second), third(p.third) { }

	void setFirst(T1 v){
		first = v;
	}

	void setSecond(T2 v){
		second = v;
	}

	void setThird(T3 v){
		third = v;
	}

	T1 getFirst(){
		return first;
	}

	T2 getSecond(){
		return second;
	}

	T3 getThird(){
		return third;
	}

	Trio& operator=(const Trio<T1, T2, T3>& p){
		this->first = p.first;
		this->second = p.second;
		this->third = p.third;

		return *this;
	}
};

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
	virtual ~PointXY(){}

	double getX() const { return this->x; }
	double getY() const { return this->y; }
	
	void setX(double x){ this->x = x; }
	void setY(double y){ this->y = y; }
};

class PointXYZ : public PointXY{
private:
	double z;
public:
	PointXYZ() : PointXY() {
		this->z = 0.0;
	}
	PointXYZ(double x, double y, double z) : PointXY(x, y) {
		this->z = z;
	}
	virtual ~PointXYZ(){}

	double getZ() const { return this->z; }
	
	void setZ(double z){ this->z = z; }
};


class RNUtils{
private:
	static bool status;
	static std::string applicationPath;
	static std::string applicationName;
    static const unsigned long PRINT_BUFFER_SIZE;
    static bool virtualScenario;
    static bool virtualFace;

    static std::string virtualScenarioPort;
    static std::string virtualFaceIpPort; 
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
	static void spinOnce();
	static bool ok();
    static void shutdown();
	static std::string getApplicationPath();
	static std::string getApplicationName();

	static bool isVirtualScenarioActivated();
	static bool isVirtualFaceActivated();
	static std::string getVirtualScenarioPort();
	static void getVirtualFaceIpPort(std::string& ip, int& port);

	static float linearInterpolator(const float& x, const PointXY& p1, const PointXY& p2);
	static float quadraticInterpolator(const float& x, const PointXY& p1, const PointXY& p2, const PointXY& x3);

	static double deg2Rad(double degrees);
	static double rad2Deg(double rad);

	static double milliwattsTodBm(const double& milliwatts);
	static double dBmTomilliwatts(const double& dBm);

private: //functions
    static void setStatus(bool status);
    static void setApplicationPathName(char* str);
};

#endif
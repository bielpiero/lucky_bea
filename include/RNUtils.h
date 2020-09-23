#ifndef RN_UTILS_H
#define RN_UTILS_H

#include <sys/stat.h>
#include <sys/time.h>   
#include <sys/types.h>   
#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <dirent.h>

#include <iostream>
#include <iterator>
#include <cerrno>
#include <vector>
#include <list>
#include <map>
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
#include <cwctype>
#include <clocale>
#include <locale>
#include <algorithm>
#include <stack>
#include <regex>
#include <queue>

#include "Matrix.h"
#include "stats.h"
#include "Aria.h"
#include "dynamixel_sdk.h"

#define RN_DEFAULT_PORT "/dev/ttyS0"

#define RN_NONE -1

#define RN_YES 1
#define RN_OK 0
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
protected:
	double x, y;
public:
	PointXY(){
		this->x = 0.0;
		this->y = 0.0;
	}
	PointXY(const double& x, const double& y){
		this->x = x;
		this->y = y;
	}

	PointXY(const PointXY& p){
		this->x = p.x;
		this->y = p.y;
	}

	virtual ~PointXY(){}

	double getX() const { return this->x; }
	double getY() const { return this->y; }
	
	void setX(double x){ this->x = x; }
	void setY(double y){ this->y = y; }

	virtual const std::string toString() const{
		std::ostringstream print_str;
		print_str.str("");
		print_str.clear();
		print_str << "{x: " << x << ", y: " << y << "}";
		return print_str.str();
	}
};

class PointXYZ : public PointXY{
protected:
	double z;
public:
	PointXYZ() : PointXY() {
		this->z = 0.0;
	}
	PointXYZ(const double& x, const double& y, const double& z) : PointXY(x, y) {
		this->z = z;
	}
	PointXYZ(const PointXYZ& p) : PointXY(p){
		this->z = p.z;
	}
	virtual ~PointXYZ(){}

	double getZ() const { return this->z; }
	
	void setZ(double z){ this->z = z; }

	virtual const std::string toString() const{
		std::ostringstream print_str;
		print_str.str("");
		print_str.clear();
		print_str << "{x: " << x << ", y: " << y << ", z: " << z << "}";
		return print_str.str();
	}
};

class LaserScan{
private:
	std::vector<double>* ranges;
	std::vector<double>* intensities;

public:
	LaserScan(){
		ranges = new std::vector<double>();
		intensities = new std::vector<double>();
	}
	
	LaserScan(const LaserScan& ls){
		ranges = new std::vector<double>(*ls.ranges);
		intensities = new std::vector<double>(*ls.intensities);
	}

    ~LaserScan(){
        clear();
        delete ranges;
        delete intensities;
    }

	std::vector<double>* getRanges() { return ranges; }
	std::vector<double>* getIntensities() { return intensities; }

	void clear(){
		ranges->clear();
        intensities->clear();
	}

	void addRange(double range, double intensity = 0) { 
		ranges->push_back(range); 
		intensities->push_back(intensity);
	}

	double getRange(int index) const { return ranges->at(index); }
	double getIntensity(int index) { return intensities->at(index); }

	int size() const { return ranges->size(); }
	double getAngleMin() { return (-M_PI / 2.0); }
	double getAngleMax() { return (M_PI / 2.0); }
	double getIncrement() { return (0.5 * M_PI / 180); }
};

class auxPose{
public:
	double x;
	double y;
	double angle;
	int sector;
	double dist;
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

	template <typename T>
	static void printList(std::list<T> l){
		typename std::list<T>::iterator it;
		std::cout << "[";
		for(it = l.begin(); it != l.end(); it++){
			std::cout << *it;
			if(std::next(it, 1) != l.end()){
				std::cout << ", ";
			}
		}
		std::cout << "]" << std::endl;
	}

	template <typename T, typename U>
	static void printMap(std::map<T, U> m){
		typename std::map<T, U>::iterator it;
		std::cout << "{";
		for(it = m.begin(); it != m.end(); it++){
			std::cout << it->first << ": " << it->second; 
			if(std::next(it, 1) != m.end()){
				std::cout << ", ";
			}
		}
		std::cout << "}" << std::endl;
	}

	static std::vector<std::string> split(char* buffer, const char* delimiter);
	static std::vector<std::string> split(std::string buffer, const char* delimiter);
	static std::vector<std::wstring> wsplit(std::wstring buffer, const wchar_t delimiter);
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

	static void rotate(const double& x, const double& y, const double& angleRad, double* newx, double* newy);

	static double linearInterpolator(const double& x, const PointXY& p1, const PointXY& p2);
	static double quadraticInterpolator(const double& x, const PointXY& p1, const PointXY& p2, const PointXY& x3);

	static void getOdometryPose(const double& xk, const double& yk, const double& thk, const double& deltaDistance, const double& deltaDegrees, double* xk1, double* yk1, double* thk1);
	static void getOdometryPose(const Matrix& posk, const Matrix& increment, Matrix& posk1);
	static void getOdometryPose(const Matrix& posk, const double& deltaDistance, const double& deltaDegrees, Matrix& posk1);
	static void getOdometryPose(const ArPose& posk, const Matrix& increment, ArPose* posk1);
	static void getOdometryPose(const ArPose& posk, const double& deltaDistance, const double& deltaDegrees, ArPose* posk1);
	
	static double distanceTo(const double& x1, const double& y1, const double& x2, const double& y2);
	static double angleTo(const double& x1, const double& y1, const double& x2, const double& y2);
	static double deg2Rad(double degrees);
	static double rad2Deg(double rad);

	static double fixAngleRad(double rad);

	static double milliwattsTodBm(const double& milliwatts);
	static double dBmTomilliwatts(const double& dBm);

private: //functions
    static void setStatus(bool status);
    static void setApplicationPathName(char* str);
};

#endif
#include <iostream>
#include <vector>

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "Aria.h"

class LsColor{
private:
	unsigned char red;
	unsigned char green;
	unsigned char blue;
public:
	LsColor(unsigned char red, unsigned char green, unsigned char blue){
		this->red = red;
		this->green = green;
		this->blue = blue;
	}
	~LsColor(){}
	unsigned char getRed(){ return red; }
	unsigned char getGreen(){ return green; }
	unsigned char getBlue(){ return blue; }
	std::string getHexadecimalColorString(){
		char buf[6]={};
		sprintf(buf, "%02x%02x%02x", red & 0xFF, green & 0xFF, blue & 0xFF);
		return buf;

	}
};

class LaserScan{
private:
	std::vector<float>* ranges;
	std::vector<int>* intensities;
	std::vector<LsColor*>* color;

public:
	LaserScan(){
		ranges = new std::vector<float>();
		intensities = new std::vector<int>();
		color = new std::vector<LsColor*>();
	}
	std::vector<float>* getRanges() { return ranges; }
	std::vector<int>* getIntensities() { return intensities; }

	void addLaserScanData(float range, int intensity = 0) { 
		ranges->push_back(range); 
		intensities->push_back(intensity);
	}

	void setScanPrimaryColor(unsigned char red, unsigned char green, unsigned char blue){
		color->push_back(new LsColor(red, green, blue));
	}

	float getRange(int index) { return ranges->at(index); }
	int getIntensity(int index) { return intensities->at(index); }
	LsColor* getColor(int index) { return color->at(index); }

	int size() { return ranges->size(); }
};

class RobotNode{
private:
	ArRobotConnector *connector;
	ArLaserConnector *laserConnector;
	ArLaser *laser;
    ArSick *sick;
    ArActionGoto *gotoPoseAction;
    ArRobot *robot;
    ArPose *myPose;
    ArSonarDevice *sonar;
    
    double maxTransVel;
    double maxAbsoluteTransVel;
    double maxRotVel;
    double maxAbsoluteRotVel;
    
    bool isDirectMotion;
    bool isGoingForward;
    bool wasDeactivated;
    bool doNotMove;

public:
	RobotNode(const char* port);
	virtual ~RobotNode();
    
    bool isGoalAchieved(void);
    
    void gotoPosition(double x, double y, double theta, double transSpeed = 200, double rotSpeed = 10);
	void move(double distance, double speed = 200);
    void moveAtSpeed(double linearVelocity, double angularVelocity);
    void stop(void);
    
    std::vector<bool> getFrontBumpersStatus(void);
    std::vector<bool> getRearBumpersStatus(void);
    
	LaserScan* getLaserScan(void);
    bool getMotorsStatus(void);
	void getRobotPosition(void);
    
    void setMotorsStatus(bool enabled);
    void setRobotPosition(double x, double y, double theta);

};
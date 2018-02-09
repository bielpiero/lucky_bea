#ifndef FINGER_H
#define FINGER_H


//#include "Arm.h"
#include "RNUtils.h"

class Finger{

private:
	std::string fingerName;  
	int dedoPort;
	int openDedo;
	int closeDedo;
	uint16_t  currentPositionSteps;
	uint16_t  currentPositionAngle;
	uint16_t speed;

public:
	Finger(int p, const char* fingerName, int max, int min);

	int getPort();
	const char* getFingerName() const;

	void setAnglePosition(uint16_t);
	uint16_t getAnglePosition();
	void setStepsPosition(uint16_t);
	uint16_t getStepsPosition();
	void setSpeed(uint16_t);
	uint16_t getSpeed(); 
	int getZero();

	int angleToSteps(int);
	int stepsToAngle(int);
	bool checkAngle(int);
	bool checkSteps (int);

};

#endif
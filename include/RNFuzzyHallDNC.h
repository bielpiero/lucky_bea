#ifndef RN_FUZZY_HALL_DNC_H
#define RN_FUZZY_HALL_DNC_H

#include "RNUtils.h"
#include "Headers.h"
#include <vector>

#define LASER_DIM 360
#define PI 3.14159

#define SIT_GR 1
#define SIT_WR 2
#define SIT_NR 3

class RNFuzzyHallDNC{

public:
	RNFuzzyHallDNC();
	~RNFuzzyHallDNC();
	
	float getDeltaTheta(LaserScan* laserData, ArPose* currPose, ArPose* goalPose);

private:
	float radioDoris;
	float distSeg;
	float wideRegLimit;
	int distMax;
	
	
	
	float deltaTheta;
	int sentidoGiro;
	
	fl::Engine* engine;
	fl::InputVariable* distObs;
	fl::OutputVariable* factor;
	fl::RuleBlock* ruleBlock;
	
	float AngleDifference(float alfa, float beta, int* giro);
	float AngleDifferenceCW(float alfa, float beta);
	float AngleDifferenceCCW(float alfa, float beta);
	int SectorDifference(int alfa, int beta);
	float LimitAngle(float angle);
	float AngleMean(float alfa, float beta);
	
	void getSystemInput(double distance, double* factor);
	
	

};

#endif

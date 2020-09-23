#ifndef RN_FUZZY_CONTROL_H
#define RN_FUZZY_CONTROL_H

#include "RNUtils.h"
#include "Headers.h"

class RNFuzzySpeedControl{

public:
	RNFuzzySpeedControl(const char* name = "Fuzzy Speed Control", const char* description = "Doris speed control for DNC");
	~RNFuzzySpeedControl();

	void getSystemInput(float distanceLeft, float distanceRight, float distanceFront, double* linearVelocity, double* angularVelocity);

private:

	std::string name;
	std::string description;
	
	fl::Engine* engineLat;
	fl::InputVariable* distLat;
	fl::OutputVariable* factorLat;
	fl::RuleBlock* ruleBlockLat;
	
	fl::Engine* engineFront;
	fl::InputVariable* distFront;
	fl::OutputVariable* factorFront;
	fl::RuleBlock* ruleBlockFront;
};

#endif

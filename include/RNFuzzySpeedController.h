#ifndef RN_FUZZY_CONTROLLER_H
#define RN_FUZZY_CONTROLLER_H

#include "RNUtils.h"
#include "Headers.h"

class RNFuzzySpeedController{

public:
	RNFuzzySpeedController(const char* name = "Fuzzy Speed Controller", const char* description = "Doris speed controller");
	~RNFuzzySpeedController();

	void getTarget(double* distanceTarget, double* angleTarget);
	void setTarget(const double& distanceTarget, const double& angleTarget);

	void getSystemInput(const double& distance, const double& angle, double* linearVelocity, double* angularVelocity);

private:
	double distanceTarget;
	double angleTarget;

	std::string name;
	std::string description;
	fl::Engine* engine;
	fl::InputVariable* distanceError;
	fl::InputVariable* angleError;
	fl::OutputVariable* linearVelocity;
	fl::OutputVariable* angularVelocity;
	fl::RuleBlock* ruleBlock;
};

#endif
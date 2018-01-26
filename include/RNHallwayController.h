#ifndef RN_HALLWAY_CONTROLLER_H
#define RN_HALLWAY_CONTROLLER_H

#include "RNUtils.h"
#include "Headers.h"

#define NUMBER_FUZZY_INPUTS_FILTER 300

class RNHallwayController{
public:
	RNHallwayController(const char* name = "Doris Hallway Controller");
	~RNHallwayController();

	void setHallwayInputs(const LaserScan* data);
	void setHallwayFuzzyInputs(double lastLeftInput, double lastFrontInput, double lastRightInput);

	double getHallwayLastLeftInput(void);
	double getHallwayLastFrontInput(void);
	double getHallwayLastRightInput(void);

	double getLinearVelocity(void);

	double getAngularVelocity(void);

	void getSystemInput(const LaserScan* data, double* linearVelocity, double* angularVelocity);

	double getTarget(void);
	void setTarget(double target);

	double getLastInput(void);
	void reset(void);

private:
	fl::Engine* engine;
	fl::InputVariable* laserLeftZone;
	fl::InputVariable* laserFrontZone;
	fl::InputVariable* laserRightZone;
	fl::OutputVariable* linearVelocity;
	fl::OutputVariable* angularVelocity;
	fl::RuleBlock* ruleBlock;
	std::string name;

	bool firstIteration;

	bool isCheckingLeft;
	bool isCheckingRight;

	double setPoint;

	double samplingTime;
	double lastInput;
	double lastError;
	double pastLastError;
	double currentError;

	std::vector<double>* hallwayLeftInput;
	std::vector<double>* hallwayFrontInput;
	std::vector<double>* hallwayRightInput;

	double* lastInputsLSector;
	double* lastInputsFSector;
	double* lastInputsRSector;

	double* lastInputsLeftDoorCheck;
	double* lastInputsRightDoorCheck;

	double* linearVelocityCheck;
	double* angularVelocityCheck;

	double hallwayLeftFuzzyInput;
	double hallwayFrontFuzzyInput;
	double hallwayRightFuzzyInput;

	int counterLeftDoorCheckIterations;
	int counterRightDoorCheckIterations;
};

#endif
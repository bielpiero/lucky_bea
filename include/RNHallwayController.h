#ifndef RN_HALLWAY_CONTROLLER_H
#define RN_HALLWAY_CONTROLLER_H

#include "RNUtils.h"
#include "Headers.h"

#define STABLE_CHECKING_NUMBER 20
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
	std::vector<PointXY> ramerDouglasPeucker(std::vector<PointXY> data);
	float identifyWall(const std::vector<PointXY>& data);

private:

	fl::Engine* engine;
	fl::InputVariable* laserLeftZone;
	fl::InputVariable* laserFrontZone;
	fl::InputVariable* laserRightZone;
	fl::OutputVariable* linearVelocity;
	fl::OutputVariable* angularVelocity;
	fl::RuleBlock* ruleBlock;
	std::string name;

	bool firstStabilizationCompleted;

	//bool isCheckingLeft;
	//bool isCheckingRight;

	bool isChecking;

	double setPoint;

	double samplingTime;
	double lastInput;
	double lastError;
	double pastLastError;
	double currentError;

	std::vector<double>* hallwayLeftInput;
	std::vector<double>* hallwayFrontInput;
	std::vector<double>* hallwayRightInput;

	std::vector<Trio<double, double, double> >* acceptedValue;
	std::vector<Trio<double, double, double> >* discardedValue;

	/*double* lastInputsLSector;
	double* lastInputsFSector;
	double* lastInputsRSector;*/

	/*double* lastInputsLeftDoorCheck;
	double* lastInputsRightDoorCheck;*/

	/*double* linearVelocityCheck;
	double* angularVelocityCheck;*/

	double hallwayLeftFuzzyInput;
	double hallwayFrontFuzzyInput;
	double hallwayRightFuzzyInput;

	/*int DoorCheckIterations;

	int counterDoorCheckIterations;*/

	//int counterLeftDoorCheckIterations;
	//int counterRightDoorCheckIterations;


	/** VÍCTOR JIMÉNEZ BERMEJO */
	std::vector<PointXY> pointsLeft, pointsRight;

	// To use the information of the identified walls there must be a security distance to surrounders
	const float FRONT_SECURITY_RANGE = 1.5; // meters
	const float SIDE_SECURITY_RANGE = 0.5;

	// Douglas-Peucker erases does points that are in a range of EPS 
	const float EPS = 0.05; 

	// Use the information of a specific area
	const float MAX_MAP_X = 3.0; // meters
	const float MAX_MAP_Y = 5.0; 

	//// Filters for the lines obtained 

	// In the perfect situation the robot is in the same orientation than the hallway -> desired angle = 90º
	// But it can face situations where he has rotated for example -> there must be a range for errors
	const float EXPECTED_LINE_ANGLE = M_PI / 2.0; // 90º -> radians
	const float ACCEPTABLE_ERROR_ANGLE= M_PI / 6.0;

	// Lines that length less than a value are discarted
	const float MAX_LINE_DISTANCE = 0.5; // meters
	const float MIN_LINE_DISTANCE = 0.5; // meters

	// Reliability of each score 
	const float DISTANCE_PERCENTAGE = 0.65;
	const float ANGLE_PERCENTAGE = 0.35;

	// Lines get a score from 0 to 100
	const float MIN_ACCEPTABLE_SCORE = 50.0;
};

#endif
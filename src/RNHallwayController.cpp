/*
To do:

- Cambiar los valores en las salidas del controlador, con 0 en el Z.

*/
#include "RNHallwayController.h"

RNHallwayController::RNHallwayController(const char* name){
	this->name = std::string(name);
	
	this->hallwayLeftInput = new std::vector<double>();
	this->hallwayFrontInput = new std::vector<double>();
	this->hallwayRightInput = new std::vector<double>();

	lastInputsLSector = (double*)calloc(NUMBER_FUZZY_INPUTS_FILTER, sizeof(double));
	lastInputsFSector = (double*)calloc(NUMBER_FUZZY_INPUTS_FILTER, sizeof(double));
	lastInputsRSector = (double*)calloc(NUMBER_FUZZY_INPUTS_FILTER, sizeof(double));

	lastInputsLeftDoorCheck = (double*)calloc(NUMBER_FUZZY_INPUTS_FILTER, sizeof(double));
	lastInputsRightDoorCheck = (double*)calloc(NUMBER_FUZZY_INPUTS_FILTER, sizeof(double));

	linearVelocityCheck = (double*)calloc(NUMBER_FUZZY_INPUTS_FILTER, sizeof(double));
	angularVelocityCheck = (double*)calloc(NUMBER_FUZZY_INPUTS_FILTER, sizeof(double));

	for(int i = 0; i < NUMBER_FUZZY_INPUTS_FILTER; i++){
		lastInputsLSector[i] = -std::numeric_limits<double>::infinity();
		lastInputsFSector[i] = -std::numeric_limits<double>::infinity();
		lastInputsRSector[i] = -std::numeric_limits<double>::infinity();
		lastInputsLeftDoorCheck[i] = -std::numeric_limits<double>::infinity();
		lastInputsRightDoorCheck[i] = -std::numeric_limits<double>::infinity();
	}
	
	firstIteration = true;

	isCheckingLeft = false;
	isCheckingRight = false;

	this->lastInput = std::numeric_limits<float>::infinity();
	this->lastError = 0;
	this->pastLastError = 0;
	this->currentError = 0;

	engine = new fl::Engine;
	engine->setName("Hallway");
	engine->setDescription("");

	laserLeftZone = new fl::InputVariable;
	laserLeftZone->setName("laserLeftZone");
	laserLeftZone->setDescription("");
	laserLeftZone->setEnabled(true);
	laserLeftZone->setRange(-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
	laserLeftZone->setLockValueInRange(false);
	laserLeftZone->addTerm(new fl::Triangle("VN", -std::numeric_limits<double>::infinity(), 0.250, 0.500));
	laserLeftZone->addTerm(new fl::Triangle("N", 0.250, 0.500, 0.750));
	laserLeftZone->addTerm(new fl::Triangle("M", 0.500, 0.750, 1.000));
	laserLeftZone->addTerm(new fl::Triangle("F", 0.750, 1.000, 1.250));
	laserLeftZone->addTerm(new fl::Triangle("VF", 1.000, 1.250, std::numeric_limits<double>::infinity()));
	engine->addInputVariable(laserLeftZone);

	laserFrontZone = new fl::InputVariable;
	laserFrontZone->setName("laserFrontZone");
	laserFrontZone->setDescription("");
	laserFrontZone->setEnabled(true);
	laserFrontZone->setRange(-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
	laserFrontZone->setLockValueInRange(false);
	laserFrontZone->addTerm(new fl::Triangle("VN", -std::numeric_limits<double>::infinity(), 0.250, 0.500));
	laserFrontZone->addTerm(new fl::Triangle("N", 0.250, 0.500, 0.750));
	laserFrontZone->addTerm(new fl::Triangle("M", 0.500, 0.750, 1.000));
	laserFrontZone->addTerm(new fl::Triangle("F", 0.750, 1.000, 1.250));
	laserFrontZone->addTerm(new fl::Triangle("VF", 1.000, 1.250, std::numeric_limits<double>::infinity()));
	engine->addInputVariable(laserFrontZone);

	laserRightZone = new fl::InputVariable;
	laserRightZone->setName("laserRightZone");
	laserRightZone->setDescription("");
	laserRightZone->setEnabled(true);
	laserRightZone->setRange(-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
	laserRightZone->setLockValueInRange(false);
	laserRightZone->addTerm(new fl::Triangle("VN", -std::numeric_limits<double>::infinity(), 0.250, 0.500));
	laserRightZone->addTerm(new fl::Triangle("N", 0.250, 0.500, 0.750));
	laserRightZone->addTerm(new fl::Triangle("M", 0.500, 0.750, 1.000));
	laserRightZone->addTerm(new fl::Triangle("F", 0.750, 1.000, 1.250));
	laserRightZone->addTerm(new fl::Triangle("VF", 1.000, 1.250, std::numeric_limits<double>::infinity()));
	engine->addInputVariable(laserRightZone);


	/*Velocidad linear más alta cuanto más lejos está de las paredes, tiene que frenar si está cerca de una pared*/
	/*Velocidades en mm/s */
	linearVelocity = new fl::OutputVariable;
	linearVelocity->setName("linearVelocity");
	linearVelocity->setDescription("");
	linearVelocity->setEnabled(true);
	linearVelocity->setRange(-40.000, 80.000);
	linearVelocity->setLockValueInRange(false);
	linearVelocity->setAggregation(new fl::AlgebraicSum);
	linearVelocity->setDefuzzifier(new fl::Centroid(100));
	linearVelocity->setDefaultValue(0.0);
	linearVelocity->setLockPreviousValue(false);
	linearVelocity->addTerm(new fl::Triangle("Backwards", -40.000, -20.000, 0.000));
	linearVelocity->addTerm(new fl::Triangle("Zero", -20.000, 0.000, 20.000));
	linearVelocity->addTerm(new fl::Triangle("SlowForward", 0.000, 20.000, 40.000));
	linearVelocity->addTerm(new fl::Triangle("MediumForward", 20.000, 40.000, 60.000));
	linearVelocity->addTerm(new fl::Triangle("FastForward", 40.000, 60.000, 80.000));
	engine->addOutputVariable(linearVelocity);

	/*Velocidad angular más alta cuanto más cerca está de las paredes, tiene que ser 0 si está muy lejos*/
	/*Velocidad en radians per second*/
	angularVelocity = new fl::OutputVariable;
	angularVelocity->setName("angularVelocity");
	angularVelocity->setDescription("");
	angularVelocity->setEnabled(true);
	angularVelocity->setRange(-1.000, 1.000);
	angularVelocity->setLockValueInRange(false);
	angularVelocity->setAggregation(new fl::AlgebraicSum);
	angularVelocity->setDefuzzifier(new fl::Centroid(100));
	angularVelocity->setDefaultValue(0.0);
	angularVelocity->setLockPreviousValue(false);
	angularVelocity->addTerm(new fl::Triangle("FastRight", -1.000, -0.666, -0.333));
	angularVelocity->addTerm(new fl::Triangle("SlowRight", -0.666, -0.333, 0.000));
	angularVelocity->addTerm(new fl::Triangle("Zero", -0.333, 0.000, 0.333));
	angularVelocity->addTerm(new fl::Triangle("SlowLeft", 0.000, 0.333, 0.666));
	angularVelocity->addTerm(new fl::Triangle("FastLeft", 0.333, 0.666, 1.000));
	engine->addOutputVariable(angularVelocity);

	

	ruleBlock = new fl::RuleBlock;
	ruleBlock->setName("mamdani");
	ruleBlock->setDescription("");
	ruleBlock->setEnabled(true);
	ruleBlock->setConjunction(new fl::AlgebraicProduct);
	ruleBlock->setDisjunction(new fl::AlgebraicSum);
	ruleBlock->setImplication(new fl::AlgebraicProduct);
	ruleBlock->setActivation(new fl::General);

	/*******/
	
	ruleBlock->addRule(new fl::Rule("if (laserLeftZone is VF and laserFrontZone is VF) and laserRightZone is VF then linearVelocity is FastForward and angularVelocity is Zero"));
	
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VF and laserFrontZone is VF and laserRightZone is F  then linearVelocity is MediumForward 	and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VF and laserFrontZone is VF and laserRightZone is M  then linearVelocity is SlowForward 	and angularVelocity is SlowLeft"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VF and laserFrontZone is VF and laserRightZone is N  then linearVelocity is Zero 			and angularVelocity is SlowLeft"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VF and laserFrontZone is VF and laserRightZone is VN then linearVelocity is Backwards 		and angularVelocity is FastLeft"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VF and laserFrontZone is F  and laserRightZone is VF then linearVelocity is MediumForward 	and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VF and laserFrontZone is F  and laserRightZone is F  then linearVelocity is MediumForward 	and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VF and laserFrontZone is F  and laserRightZone is M  then linearVelocity is SlowForward 	and angularVelocity is SlowLeft"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VF and laserFrontZone is F  and laserRightZone is N  then linearVelocity is Zero 			and angularVelocity is SlowLeft"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VF and laserFrontZone is F  and laserRightZone is VN then linearVelocity is Backwards 		and angularVelocity is FastLeft"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VF and laserFrontZone is M  and laserRightZone is VF then linearVelocity is SlowForward 	and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VF and laserFrontZone is M  and laserRightZone is F  then linearVelocity is SlowForward	and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VF and laserFrontZone is M  and laserRightZone is M  then linearVelocity is SlowForward	and angularVelocity is SlowLeft"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VF and laserFrontZone is M  and laserRightZone is N  then linearVelocity is Zero 			and angularVelocity is SlowLeft"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VF and laserFrontZone is M  and laserRightZone is VN then linearVelocity is Backwards 		and angularVelocity is FastLeft"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VF and laserFrontZone is N  and laserRightZone is VF then linearVelocity is Zero 			and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VF and laserFrontZone is N  and laserRightZone is F  then linearVelocity is Zero 			and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VF and laserFrontZone is N  and laserRightZone is M  then linearVelocity is Zero 			and angularVelocity is SlowLeft"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VF and laserFrontZone is N  and laserRightZone is N  then linearVelocity is Zero 			and angularVelocity is SlowLeft"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VF and laserFrontZone is N  and laserRightZone is VN then linearVelocity is Backwards 		and angularVelocity is FastLeft"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VF and laserFrontZone is VN and laserRightZone is VF then linearVelocity is Backwards 		and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VF and laserFrontZone is VN and laserRightZone is F  then linearVelocity is Backwards 		and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VF and laserFrontZone is VN and laserRightZone is M  then linearVelocity is Backwards 		and angularVelocity is SlowLeft"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VF and laserFrontZone is VN and laserRightZone is N  then linearVelocity is Backwards 		and angularVelocity is SlowLeft"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VF and laserFrontZone is VN and laserRightZone is VN then linearVelocity is Backwards 		and angularVelocity is FastLeft"));
	
	/*******/
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is F and laserFrontZone is VF and laserRightZone is VF then linearVelocity is MediumForward 	and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is F and laserFrontZone is VF and laserRightZone is F  then linearVelocity is MediumForward 	and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is F and laserFrontZone is VF and laserRightZone is M  then linearVelocity is SlowForward 	and angularVelocity is SlowLeft"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is F and laserFrontZone is VF and laserRightZone is N  then linearVelocity is Zero 			and angularVelocity is SlowLeft"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is F and laserFrontZone is VF and laserRightZone is VN then linearVelocity is Backwards 		and angularVelocity is FastLeft"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is F and laserFrontZone is F  and laserRightZone is VF then linearVelocity is MediumForward 	and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is F and laserFrontZone is F  and laserRightZone is F  then linearVelocity is MediumForward 	and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is F and laserFrontZone is F  and laserRightZone is M  then linearVelocity is SlowForward 	and angularVelocity is SlowLeft"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is F and laserFrontZone is F  and laserRightZone is N  then linearVelocity is Zero 			and angularVelocity is SlowLeft"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is F and laserFrontZone is F  and laserRightZone is VN then linearVelocity is Backwards 		and angularVelocity is FastLeft"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is F and laserFrontZone is M  and laserRightZone is VF then linearVelocity is SlowForward 	and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is F and laserFrontZone is M  and laserRightZone is F  then linearVelocity is SlowForward 	and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is F and laserFrontZone is M  and laserRightZone is M  then linearVelocity is SlowForward 	and angularVelocity is SlowLeft"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is F and laserFrontZone is M  and laserRightZone is N  then linearVelocity is Zero 			and angularVelocity is SlowLeft"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is F and laserFrontZone is M  and laserRightZone is VN then linearVelocity is Backwards 		and angularVelocity is FastLeft"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is F and laserFrontZone is N  and laserRightZone is VF then linearVelocity is Zero 			and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is F and laserFrontZone is N  and laserRightZone is F  then linearVelocity is Zero 			and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is F and laserFrontZone is N  and laserRightZone is M  then linearVelocity is Zero 			and angularVelocity is SlowLeft"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is F and laserFrontZone is N  and laserRightZone is N  then linearVelocity is Zero 			and angularVelocity is SlowLeft"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is F and laserFrontZone is N  and laserRightZone is VN then linearVelocity is Backwards 		and angularVelocity is FastLeft"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is F and laserFrontZone is VN and laserRightZone is VF then linearVelocity is Backwards 		and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is F and laserFrontZone is VN and laserRightZone is F  then linearVelocity is Backwards 		and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is F and laserFrontZone is VN and laserRightZone is M  then linearVelocity is Backwards 		and angularVelocity is SlowLeft"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is F and laserFrontZone is VN and laserRightZone is N  then linearVelocity is Backwards 		and angularVelocity is SlowLeft"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is F and laserFrontZone is VN and laserRightZone is VN then linearVelocity is Backwards 		and angularVelocity is FastLeft"));
	/*******/
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is M and laserFrontZone is VF and laserRightZone is VF then linearVelocity is SlowForward 	and angularVelocity is SlowRight"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is M and laserFrontZone is VF and laserRightZone is F  then linearVelocity is SlowForward 	and angularVelocity is SlowRight"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is M and laserFrontZone is VF and laserRightZone is M  then linearVelocity is SlowForward 	and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is M and laserFrontZone is VF and laserRightZone is N  then linearVelocity is Zero 			and angularVelocity is SlowLeft"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is M and laserFrontZone is VF and laserRightZone is VN then linearVelocity is Backwards 		and angularVelocity is FastLeft"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is M and laserFrontZone is F  and laserRightZone is VF then linearVelocity is SlowForward 	and angularVelocity is SlowRight"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is M and laserFrontZone is F  and laserRightZone is F  then linearVelocity is SlowForward 	and angularVelocity is SlowRight"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is M and laserFrontZone is F  and laserRightZone is M  then linearVelocity is SlowForward 	and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is M and laserFrontZone is F  and laserRightZone is N  then linearVelocity is Zero 			and angularVelocity is SlowLeft"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is M and laserFrontZone is F  and laserRightZone is VN then linearVelocity is Backwards 		and angularVelocity is FastLeft"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is M and laserFrontZone is M  and laserRightZone is VF then linearVelocity is SlowForward 	and angularVelocity is SlowRight"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is M and laserFrontZone is M  and laserRightZone is F  then linearVelocity is SlowForward 	and angularVelocity is SlowRight"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is M and laserFrontZone is M  and laserRightZone is M  then linearVelocity is SlowForward 	and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is M and laserFrontZone is M  and laserRightZone is N  then linearVelocity is Zero 			and angularVelocity is SlowLeft"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is M and laserFrontZone is M  and laserRightZone is VN then linearVelocity is Backwards 		and angularVelocity is FastLeft"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is M and laserFrontZone is N  and laserRightZone is VF then linearVelocity is Zero 			and angularVelocity is SlowRight"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is M and laserFrontZone is N  and laserRightZone is F  then linearVelocity is Zero 			and angularVelocity is SlowRight"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is M and laserFrontZone is N  and laserRightZone is M  then linearVelocity is Zero 			and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is M and laserFrontZone is N  and laserRightZone is N  then linearVelocity is Zero 			and angularVelocity is SlowLeft"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is M and laserFrontZone is N  and laserRightZone is VN then linearVelocity is Backwards 		and angularVelocity is FastLeft"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is M and laserFrontZone is VN and laserRightZone is VF then linearVelocity is Backwards 		and angularVelocity is SlowRight"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is M and laserFrontZone is VN and laserRightZone is F  then linearVelocity is Backwards 		and angularVelocity is SlowRight"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is M and laserFrontZone is VN and laserRightZone is M  then linearVelocity is Backwards 		and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is M and laserFrontZone is VN and laserRightZone is N  then linearVelocity is Backwards 		and angularVelocity is SlowLeft"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is M and laserFrontZone is VN and laserRightZone is VN then linearVelocity is Backwards 		and angularVelocity is FastLeft"));
	/*******/
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is N and laserFrontZone is VF and laserRightZone is VF then linearVelocity is Zero 		and angularVelocity is SlowRight"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is N and laserFrontZone is VF and laserRightZone is F  then linearVelocity is Zero 		and angularVelocity is SlowRight"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is N and laserFrontZone is VF and laserRightZone is M  then linearVelocity is Zero 		and angularVelocity is SlowRight"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is N and laserFrontZone is VF and laserRightZone is N  then linearVelocity is Zero 		and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is N and laserFrontZone is VF and laserRightZone is VN then linearVelocity is Backwards 	and angularVelocity is SlowLeft"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is N and laserFrontZone is F  and laserRightZone is VF then linearVelocity is Zero 		and angularVelocity is SlowRight"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is N and laserFrontZone is F  and laserRightZone is F  then linearVelocity is Zero 		and angularVelocity is SlowRight"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is N and laserFrontZone is F  and laserRightZone is M  then linearVelocity is Zero 		and angularVelocity is SlowRight"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is N and laserFrontZone is F  and laserRightZone is N  then linearVelocity is Zero 		and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is N and laserFrontZone is F  and laserRightZone is VN then linearVelocity is Backwards 	and angularVelocity is SlowLeft"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is N and laserFrontZone is M  and laserRightZone is VF then linearVelocity is Zero 		and angularVelocity is SlowRight"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is N and laserFrontZone is M  and laserRightZone is F  then linearVelocity is Zero 		and angularVelocity is SlowRight"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is N and laserFrontZone is M  and laserRightZone is M  then linearVelocity is Zero 		and angularVelocity is SlowRight"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is N and laserFrontZone is M  and laserRightZone is N  then linearVelocity is Zero 		and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is N and laserFrontZone is M  and laserRightZone is VN then linearVelocity is Backwards 	and angularVelocity is SlowLeft"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is N and laserFrontZone is N  and laserRightZone is VF then linearVelocity is Zero 		and angularVelocity is SlowRight"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is N and laserFrontZone is N  and laserRightZone is F  then linearVelocity is Zero 		and angularVelocity is SlowRight"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is N and laserFrontZone is N  and laserRightZone is M  then linearVelocity is Zero 		and angularVelocity is SlowRight"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is N and laserFrontZone is N  and laserRightZone is N  then linearVelocity is Zero 		and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is N and laserFrontZone is N  and laserRightZone is VN then linearVelocity is Backwards 	and angularVelocity is SlowLeft"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is N and laserFrontZone is VN and laserRightZone is VF then linearVelocity is Backwards 	and angularVelocity is SlowRight"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is N and laserFrontZone is VN and laserRightZone is F  then linearVelocity is Backwards 	and angularVelocity is SlowRight"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is N and laserFrontZone is VN and laserRightZone is M  then linearVelocity is Backwards 	and angularVelocity is SlowRight"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is N and laserFrontZone is VN and laserRightZone is N  then linearVelocity is Backwards 	and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is N and laserFrontZone is VN and laserRightZone is VN then linearVelocity is Backwards 	and angularVelocity is SlowLeft"));
	/*******/
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VN and laserFrontZone is VF and laserRightZone is VF then linearVelocity is Backwards and angularVelocity is FastRight"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VN and laserFrontZone is VF and laserRightZone is F  then linearVelocity is Backwards and angularVelocity is FastRight"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VN and laserFrontZone is VF and laserRightZone is M  then linearVelocity is Backwards and angularVelocity is FastRight"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VN and laserFrontZone is VF and laserRightZone is N  then linearVelocity is Backwards and angularVelocity is SlowRight"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VN and laserFrontZone is VF and laserRightZone is VN then linearVelocity is Backwards and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VN and laserFrontZone is F  and laserRightZone is VF then linearVelocity is Backwards and angularVelocity is FastRight"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VN and laserFrontZone is F  and laserRightZone is F  then linearVelocity is Backwards and angularVelocity is FastRight"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VN and laserFrontZone is F  and laserRightZone is M  then linearVelocity is Backwards and angularVelocity is FastRight"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VN and laserFrontZone is F  and laserRightZone is N  then linearVelocity is Backwards and angularVelocity is SlowRight"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VN and laserFrontZone is F  and laserRightZone is VN then linearVelocity is Backwards and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VN and laserFrontZone is M  and laserRightZone is VF then linearVelocity is Backwards and angularVelocity is FastRight"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VN and laserFrontZone is M  and laserRightZone is F  then linearVelocity is Backwards and angularVelocity is FastRight"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VN and laserFrontZone is M  and laserRightZone is M  then linearVelocity is Backwards and angularVelocity is FastRight"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VN and laserFrontZone is M  and laserRightZone is N  then linearVelocity is Backwards and angularVelocity is SlowRight"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VN and laserFrontZone is M  and laserRightZone is VN then linearVelocity is Backwards and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VN and laserFrontZone is N  and laserRightZone is VF then linearVelocity is Backwards and angularVelocity is FastRight"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VN and laserFrontZone is N  and laserRightZone is F  then linearVelocity is Backwards and angularVelocity is FastRight"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VN and laserFrontZone is N  and laserRightZone is M  then linearVelocity is Backwards and angularVelocity is FastRight"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VN and laserFrontZone is N  and laserRightZone is N  then linearVelocity is Backwards and angularVelocity is SlowRight"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VN and laserFrontZone is N  and laserRightZone is VN then linearVelocity is Backwards and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VN and laserFrontZone is VN and laserRightZone is VF then linearVelocity is Backwards and angularVelocity is FastRight"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VN and laserFrontZone is VN and laserRightZone is F  then linearVelocity is Backwards and angularVelocity is FastRight"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VN and laserFrontZone is VN and laserRightZone is M  then linearVelocity is Backwards and angularVelocity is FastRight"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VN and laserFrontZone is VN and laserRightZone is N  then linearVelocity is Backwards and angularVelocity is SlowRight"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VN and laserFrontZone is VN and laserRightZone is VN then linearVelocity is Backwards and angularVelocity is Zero"));
	ruleBlock->loadRules(engine);
	engine->addRuleBlock(ruleBlock);
	fl::fuzzylite::setDebugging(true);
	
	RNUtils::printLn("%s has been instantiated...", name);
}

RNHallwayController::~RNHallwayController(){
	hallwayLeftInput->clear();
	delete hallwayLeftInput;

	hallwayFrontInput->clear();
	delete hallwayFrontInput;

	hallwayRightInput->clear();
	delete hallwayRightInput;

	delete engine;
		
}

void RNHallwayController::reset(void){
	firstIteration = true;
	this->lastInput = std::numeric_limits<float>::infinity();
	this->lastError = 0;
	this->pastLastError = 0;
	this->currentError = 0;
}

double RNHallwayController::getTarget(void){
	return this->setPoint;
}

void RNHallwayController::setTarget(double target){
	this->setPoint = target;
}

double RNHallwayController::getLastInput(void){
	return this->lastInput;
}

void RNHallwayController::getSystemInput(const LaserScan* data, double* linearVelocity, double* angularVelocity){
	double lastLeftInput;
	double lastFrontInput;
	double lastRightInput;
	if(data != NULL){
		setHallwayInputs(data);

		lastLeftInput = getHallwayLastLeftInput();
		lastFrontInput = getHallwayLastFrontInput();
		lastRightInput = getHallwayLastRightInput();
		printf("L: %lf, F: %lf, R: %lf\n", lastLeftInput, lastFrontInput, lastRightInput);
		setHallwayFuzzyInputs(lastLeftInput, lastFrontInput, lastRightInput);
		printf("Rules: %d\n", engine->getRuleBlock(0)->numberOfRules());
		engine->process();

		int i;

		for(int i = NUMBER_FUZZY_INPUTS_FILTER - 1; i > 0; i--){
			linearVelocityCheck[i]=linearVelocityCheck[i - 1];
			angularVelocityCheck[i]=angularVelocityCheck[i - 1];
		}
		linearVelocityCheck[0]=this->linearVelocity->getValue();
		angularVelocityCheck[0]=this->linearVelocity->getValue();

		if(std::abs(*linearVelocity) == std::numeric_limits<double>::quiet_NaN() || std::abs(*angularVelocity) == std::numeric_limits<double>::quiet_NaN()){
			*linearVelocity = 0.0;
			*angularVelocity = 0.0;
			i = NUMBER_FUZZY_INPUTS_FILTER;
		}
		else if(isCheckingLeft || isCheckingRight){
			*angularVelocity = linearVelocityCheck[i];
			*linearVelocity = angularVelocityCheck[i];
			i--;
		}
		else{
			*linearVelocity = this->linearVelocity->getValue();
			*angularVelocity = this->angularVelocity->getValue();
			i = NUMBER_FUZZY_INPUTS_FILTER;
		}
	}
}

void RNHallwayController::setHallwayInputs(const LaserScan* data){

    hallwayLeftInput->clear();
    hallwayFrontInput->clear();
    hallwayRightInput->clear();

    for(int i = 0; i < data->size()-241; i++){
    	this->hallwayLeftInput->push_back(data->getRange(i));	//Obtiene las distancias medidas por el láser en los primeros 120 grados
    }
    for(int i = data->size()-240; i < data->size()-120; i++){
    	this->hallwayFrontInput->push_back(data->getRange(i));
    }
    for(int i = data->size()-119; i < data->size(); i++){
    	this->hallwayRightInput->push_back(data->getRange(i));
    }
}

void RNHallwayController::setHallwayFuzzyInputs(double lastLeftInput, double lastFrontInput, double lastRightInput){
	/*Hay que establecer el número de inputs*/
	int leftCounter = 0;
	int frontCounter = 0;
	int rightCounter = 0;

	double counterLeftInputs = 0.0;
	double counterFrontInputs = 0.0;
	double counterRightInputs = 0.0;

	/*Cuidado con el orden de magnitud del check distance y de las iteraciones*/
	double doorCheckDistance = 0.1;
	int doorCheckIterations = 100;

	
	if(firstIteration){
		lastInputsLSector[0] = lastLeftInput;	
		lastInputsFSector[0] = lastFrontInput;	
		lastInputsRSector[0] = lastRightInput;

		lastInputsLeftDoorCheck[0] = lastLeftInput;
		lastInputsRightDoorCheck[0] = lastRightInput;

		counterLeftDoorCheckIterations = 0;
		counterRightDoorCheckIterations = 0;

		hallwayLeftFuzzyInput = lastLeftInput;
		hallwayFrontFuzzyInput = lastFrontInput;
		hallwayRightFuzzyInput = lastRightInput;

		printf("Entra al fuzzy Left: %lf\n", hallwayLeftFuzzyInput);
		this->laserLeftZone->setValue(hallwayLeftFuzzyInput);

		printf("Entra al fuzzy Front: %lf\n", hallwayFrontFuzzyInput);
		this->laserFrontZone->setValue(hallwayFrontFuzzyInput);

		printf("Entra al fuzzy Right: %lf\n", hallwayRightFuzzyInput);
		this->laserRightZone->setValue(hallwayRightFuzzyInput);
		firstIteration = false;
	} else {
		if ((lastLeftInput - hallwayLeftFuzzyInput) > doorCheckDistance){
			isCheckingLeft = true;
			if (counterLeftDoorCheckIterations < doorCheckIterations){
				for(int i = NUMBER_FUZZY_INPUTS_FILTER - 1; i > 0; i--){
					lastInputsLeftDoorCheck[i] = lastInputsLeftDoorCheck[i - 1];
				}
				lastInputsLeftDoorCheck[0] = lastLeftInput;	
				counterLeftDoorCheckIterations++;
			}
			else{
				for(int i = NUMBER_FUZZY_INPUTS_FILTER - 1; i > 0; i--){
					lastInputsLSector[i] = lastInputsLeftDoorCheck[i];
					counterLeftDoorCheckIterations = 0;
				}
			}
		}
		else{
			for(int i = NUMBER_FUZZY_INPUTS_FILTER; i > 0; i--){
				lastInputsLSector[i]=lastInputsLSector[i - 1];
			}
			lastInputsLSector[0] = lastLeftInput;
			for(int i = 0; i < NUMBER_FUZZY_INPUTS_FILTER; i++){
				if(lastInputsLSector[i] != -std::numeric_limits<double>::infinity()){
					counterLeftInputs+=lastInputsLSector[i];
					leftCounter++;
				}
			}
			isCheckingLeft = false;
			hallwayLeftFuzzyInput = counterLeftInputs/(double)leftCounter;
			counterLeftDoorCheckIterations = 0;
		}
		printf("Entra al fuzzy Left: %lf\n", hallwayLeftFuzzyInput);
		this->laserLeftZone->setValue(hallwayLeftFuzzyInput);

		for(int i = NUMBER_FUZZY_INPUTS_FILTER - 1; i > 0; i--){
			lastInputsFSector[i]=lastInputsFSector[i - 1];
		}
		lastInputsFSector[0]=lastFrontInput;

		for(int i = 0; i < NUMBER_FUZZY_INPUTS_FILTER; i++){
			if(lastInputsFSector[i] != -std::numeric_limits<double>::infinity()){
				counterFrontInputs += lastInputsFSector[i];
				frontCounter++;
			}
		}
		hallwayFrontFuzzyInput = counterFrontInputs/(double)frontCounter;

		printf("Entra al fuzzy Front: %lf\n", hallwayFrontFuzzyInput);
		this->laserFrontZone->setValue(hallwayFrontFuzzyInput);
		
		if ((lastRightInput-hallwayRightFuzzyInput)>doorCheckDistance){
			isCheckingRight = true;
			if (counterRightDoorCheckIterations<doorCheckIterations){
				for(int i = NUMBER_FUZZY_INPUTS_FILTER - 1; i > 0; i--){
					lastInputsRightDoorCheck[i] = lastInputsRightDoorCheck[i - 1];
				}
				lastInputsRightDoorCheck[0] = lastRightInput;	
				counterRightDoorCheckIterations++;
			}
			else{
				for(int i = NUMBER_FUZZY_INPUTS_FILTER - 1; i > 0; i--){
					lastInputsRSector[i] = lastInputsRightDoorCheck[i];
					counterRightDoorCheckIterations = 0;
				}
			}
		}
		else{
			for(int i = NUMBER_FUZZY_INPUTS_FILTER - 1; i > 0; i--){
				lastInputsRSector[i]=lastInputsRSector[i - 1];
			}
			lastInputsRSector[0]=lastRightInput;
			for(int i = 0; i < NUMBER_FUZZY_INPUTS_FILTER; i++){
				if(lastInputsFSector[i] != -std::numeric_limits<double>::infinity()){
					counterRightInputs+=lastInputsRSector[i];
					rightCounter++;
				}
			}
			isCheckingRight = false;
			hallwayRightFuzzyInput = counterRightInputs/(double)rightCounter;
			counterRightDoorCheckIterations = 0;
		}
		printf("Entra al fuzzy Right: %lf\n", hallwayRightFuzzyInput);
		this->laserRightZone->setValue(hallwayRightFuzzyInput);
	}
	

	
}

double RNHallwayController::getHallwayLastLeftInput(void){
	double hallwayLSectorVN = 0;
	double hallwayLSectorN = 0;
	double hallwayLSectorM = 0;
	double hallwayLSectorF = 0;
	double hallwayLSectorVF = 0;
	int counterVN = 0;
	int counterN = 0;
	int counterM = 0;
	int counterF = 0;
	int counterVF = 0;
	double lastLeftInput;

	for(int i=0; i < hallwayLeftInput->size(); i++){
		if (hallwayLeftInput->at(i) < 0.5){
			counterVN++;
			hallwayLSectorVN += hallwayLeftInput->at(i);
		}
		else if (hallwayLeftInput->at(i)<1.0){
			counterN++;
			hallwayLSectorN += hallwayLeftInput->at(i);
		}
		else if (hallwayLeftInput->at(i)<1.5){
			counterM++;
			hallwayLSectorM += hallwayLeftInput->at(i);
		}
		else if (hallwayLeftInput->at(i)<2.0){
			counterF++;
			hallwayLSectorF += hallwayLeftInput->at(i);
		}
		else {
			counterVF++;
			hallwayLSectorVF += hallwayLeftInput->at(i);
		}
	}

	if (counterVN > 10){
		lastLeftInput = (double)hallwayLSectorVN/(double)counterVN;
	} else if (counterN > 10){
		lastLeftInput = (double)hallwayLSectorN/(double)counterN;
	} else if (counterM > 10){
		lastLeftInput = (double)hallwayLSectorM/(double)counterM;
	} else if (counterF > 10){
		lastLeftInput = (double)hallwayLSectorF/(double)counterF;
	}else {
		lastLeftInput = (double)hallwayLSectorVF/(double)counterVF;
	}
		
	return lastLeftInput;
}

double RNHallwayController::getHallwayLastFrontInput(void){
	double hallwayFSectorVN = 0;
	double hallwayFSectorN = 0;
	double hallwayFSectorM = 0;
	double hallwayFSectorF = 0;
	double hallwayFSectorVF = 0;
	int counterVN = 0;
	int counterN = 0;
	int counterM = 0;
	int counterF = 0;
	int counterVF = 0;
	double lastFrontInput;
	

	for(int i=0; i < hallwayFrontInput->size(); i++){
		if (hallwayFrontInput->at(i) < 0.5){
			counterVN++;
			hallwayFSectorVN += hallwayFrontInput->at(i);
		}
		else if (hallwayFrontInput->at(i)<1.0){
			counterN++;
			hallwayFSectorN += hallwayFrontInput->at(i);
		}
		else if (hallwayFrontInput->at(i)<1.5){
			counterM++;
			hallwayFSectorM += hallwayFrontInput->at(i);
		}
		else if (hallwayFrontInput->at(i)<2.0){
			counterF++;
			hallwayFSectorF += hallwayFrontInput->at(i);
		}
		else {
			counterVF++;
			hallwayFSectorVF += hallwayFrontInput->at(i);
		}
	}

	if (counterVN > 10){
		lastFrontInput = (double)hallwayFSectorVN/(double)counterVN;
	} else if (counterN > 10){
		lastFrontInput = (double)hallwayFSectorN/(double)counterN;
	} else if (counterM > 10){
		lastFrontInput = (double)hallwayFSectorM/(double)counterM;
	} else if (counterF > 10){
		lastFrontInput = (double)hallwayFSectorF/(double)counterF;
	}else {
		lastFrontInput = (double)hallwayFSectorVF/(double)counterVF;
	}

	return lastFrontInput;
}

double RNHallwayController::getHallwayLastRightInput(void){
	double hallwayRSectorVN = 0;
	double hallwayRSectorN = 0;
	double hallwayRSectorM = 0;
	double hallwayRSectorF = 0;
	double hallwayRSectorVF = 0;
	int counterVN = 0;
	int counterN = 0;
	int counterM = 0;
	int counterF = 0;
	int counterVF = 0;
	double lastRightInput;

	for(int i = 0; i < hallwayRightInput->size(); i++){
		if (hallwayRightInput->at(i)<0.5){
			counterVN++;
			hallwayRSectorVN += hallwayRightInput->at(i);
		}
		else if (hallwayRightInput->at(i)<1.0){
			counterN++;
			hallwayRSectorN += hallwayRightInput->at(i);
		}
		else if (hallwayRightInput->at(i)<1.5){
			counterM++;
			hallwayRSectorM += hallwayRightInput->at(i);
		}
		else if (hallwayRightInput->at(i)<2.0){
			counterF++;
			hallwayRSectorF += hallwayRightInput->at(i);
		}
		else {
			counterVF++;
			hallwayRSectorVF += hallwayRightInput->at(i);
		}
	}

	if (counterVN > 10){
		lastRightInput = (double)hallwayRSectorVN/(double)counterVN;
	} else if (counterN > 10){
		lastRightInput = (double)hallwayRSectorN/(double)counterN;
	} else if (counterM > 10){
		lastRightInput = (double)hallwayRSectorM/(double)counterM;
	} else if (counterF > 10){
		lastRightInput = (double)hallwayRSectorF/(double)counterF;
	}else {
		lastRightInput = (double)hallwayRSectorVF/(double)counterVF;
	}

	return lastRightInput;
}


double RNHallwayController::getLinearVelocity(void){
	return linearVelocity->getValue();
}

double RNHallwayController::getAngularVelocity(void){
	return angularVelocity->getValue();
}


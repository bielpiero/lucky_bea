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

	acceptedValue = new std::vector<Trio<double,double,double> >();
	discardedValue = new std::vector<Trio<double,double,double> >();
	
	firstStabilizationCompleted = false;

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
	linearVelocity->addTerm(new fl::Triangle("Backwards", -80.000, -40.000, 0.000));
	linearVelocity->addTerm(new fl::Triangle("Zero", -40.000, 0.000, 40.000));
	linearVelocity->addTerm(new fl::Triangle("SlowForward", 0.000, 40.000, 80.000));
	linearVelocity->addTerm(new fl::Triangle("MediumForward", 40.000, 80.000, 120.000));
	linearVelocity->addTerm(new fl::Triangle("FastForward", 80.000, 120.000, 160.000));
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
	fl::fuzzylite::setDebugging(false);
	
	
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
	firstStabilizationCompleted = false;
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
		RNUtils::printLn("L: %lf, F: %lf, R: %lf", lastLeftInput, lastFrontInput, lastRightInput);
		setHallwayFuzzyInputs(lastLeftInput, lastFrontInput, lastRightInput);
		engine->process();

		*linearVelocity = this->linearVelocity->getValue();
		*angularVelocity = this->angularVelocity->getValue();

		if(std::abs(*linearVelocity) == std::numeric_limits<double>::quiet_NaN() || std::abs(*angularVelocity) == std::numeric_limits<double>::quiet_NaN()){
			*linearVelocity = 0.0;
			*angularVelocity = 0.0;
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




	/** VICTOR JIMÉNEZ BERMEJO */
	/** 2D map of 1 cicle of the LIDAR */
	float ang = 0.0;
	float x = 0.0;
	float y = 0.0;
	for(int i = 0; i < data->size(); i++)
	{
		ang = ( (i - 180.0)/2.0 ) * M_PI / 180.0; // Angulo desde -180 hasta 180 en radianes
		x = data->getRange(i) * std::cos(ang);
		y = data->getRange(i) * std::sin(ang);

		// Filter Points too far 
		if(std::fabs(x) <= MAX_MAP_X and std::fabs(y) <= MAX_MAP_Y){
			if(x < 0){
				pointsLeft.push_back(PointXY(x, y));
			} else {
				pointsRight.push_back(PointXY(x, y));
			}
		}
	}
}

void RNHallwayController::setHallwayFuzzyInputs(double lastLeftInput, double lastFrontInput, double lastRightInput){

	float leftWallX;
	float rightWallX; 


	
	/*int doorCheckIterations = 400;
	double doorCheckDistance = 0.03;

	bool isStable = true;
	bool neutrilizeAngle = false;

	if(acceptedValue->size() > STABLE_CHECKING_NUMBER){
		for(int i = 0; i < STABLE_CHECKING_NUMBER and not isStable; i++){
			if(std::abs(acceptedValue->at(i).getFirst() - acceptedValue->at(i).getThird()) > doorCheckDistance){
				isStable = false;
				RNUtils::printLn("It's not stable yet...");
			}
		}
	} else {
		RNUtils::printLn("It's not stable yet...");
		isStable = false;
	}

	if((std::abs(lastLeftInput - lastRightInput) >= doorCheckDistance) and isStable){
		std::vector<Trio<double, double, double> >::iterator it = discardedValue->begin();
		discardedValue->insert(it, Trio<double, double, double>(lastLeftInput, lastFrontInput, lastRightInput));
		RNUtils::printLn("Added to discardedValue");
		neutrilizeAngle = true;
		if(discardedValue->size() > doorCheckIterations){
			RNUtils::printLn("Hallway has changed....");
			acceptedValue->clear();
			std::copy(discardedValue->begin(), discardedValue->end(), std::back_inserter(*acceptedValue));
			discardedValue->clear();
		}
		RNUtils::printLn("discardedValue size: %d", discardedValue->size());
	}
	else{
		std::vector<Trio<double, double, double> >::iterator it = acceptedValue->begin();
		acceptedValue->insert(it, Trio<double, double, double>(lastLeftInput, lastFrontInput, lastRightInput));
		discardedValue->clear();
	}

	hallwayLeftFuzzyInput = acceptedValue->at(0).getFirst();
	hallwayFrontFuzzyInput = acceptedValue->at(0).getSecond();
	hallwayRightFuzzyInput = acceptedValue->at(0).getThird();
	if(neutrilizeAngle){
		hallwayRightFuzzyInput = acceptedValue->at(0).getFirst();
	}*/
	

	/** VÍCTOR JIMÉNEZ BERMEJO*/

	/** Ramer-Douglas-Peucker to simplify de measures of the LIDAR into main points */
	pointsLeft = ramerDouglasPeucker(pointsLeft); 
	pointsRight = ramerDouglasPeucker(pointsRight);

	/** Identify walls within these points */
	leftWallX = identifyWall(pointsLeft);
	rightWallX = identifyWall(pointsRight);
	printf("Las distancias al pasillo identificado son: izquierda %.4f y derecha %.4f\n", leftWallX, rightWallX);


	/** If the situation is favourable use this information */
	if( (hallwayFrontFuzzyInput > FRONT_SECURITY_RANGE) and (std::fabs(hallwayLeftFuzzyInput) > SIDE_SECURITY_RANGE) 
		and (std::fabs(hallwayRightFuzzyInput) > SIDE_SECURITY_RANGE) and (std::fabs(leftWallX) > SIDE_SECURITY_RANGE) 
		and (std::fabs(rightWallX) > SIDE_SECURITY_RANGE) ) {
		hallwayLeftFuzzyInput = leftWallX;
		hallwayRightFuzzyInput = rightWallX;

		hallwayFrontFuzzyInput = lastFrontInput; 

		printf("Las condiciones son favorables, se usa el reconocimiento de paredes\n");
	} else {
		/** If we can't reach a reliable identification -> Act in function of the last input*/	
		hallwayLeftFuzzyInput = lastLeftInput;
		hallwayFrontFuzzyInput = lastFrontInput;
		hallwayRightFuzzyInput = lastRightInput;
	}

	/** end VÍCTOR JIMÉNEZ BERMEJO*/
	
	RNUtils::printLn("FL: %lf, FF: %lf, FR: %lf", hallwayLeftFuzzyInput, hallwayFrontFuzzyInput, hallwayRightFuzzyInput);
	this->laserLeftZone->setValue(hallwayLeftFuzzyInput);
	this->laserFrontZone->setValue(hallwayFrontFuzzyInput);
	this->laserRightZone->setValue(hallwayRightFuzzyInput);	
}

/** VÍCTOR JIMÉNEZ BERMEJO */

/** Ramer_Douglas_Peucker 
Recursive function */
std::vector<PointXY> RNHallwayController::ramerDouglasPeucker(std::vector<PointXY> data)
{
	std::vector<PointXY> newData, newData1, newData2;

	/** Recta formada por el primer y último punto */
	float m = (data.front().getY() - data.back().getY()) / (data.front().getX() - data.back().getX());
	float n = data.front().getY() - m * data.front().getX();

	/** Calculo deĺ punto más alejado de la recta (se calcula la mínima distancia de cada punto a la recta y se coge la mayor de todas) */
	float dist = 0.0;
	float dist_max = 0.0;
	int index_dist_max = -1;
	for(int i = 1; i < (data.size() - 1); i++) // i = 1 & size()-1 para que no se compare con ellos mismos
	{
		dist = std::fabs(m*data.at(i).getX() - data.at(i).getY() + n) / std::sqrt(m*m + 1.0);
		if(dist > dist_max)
		{
			index_dist_max = i;
			dist_max = dist;
		}
	}

	/** Si la distancia es mayor o menor que EPS se simplifica o se vuelve a llamar a esta función recurisvamente con un intervalo de puntos menor*/ 
	if(dist_max >= EPS) {
		// Si es mayor hay que dividir los datos en 2 partes y simplificar ambas
		newData1 = ramerDouglasPeucker(std::vector<PointXY>(data.begin(), data.begin() + index_dist_max + 1));
		newData2 = ramerDouglasPeucker(std::vector<PointXY>(data.begin() + index_dist_max, data.end()));

		// Juntar resultado de ambas 
		//newData.reserve(newData1.size() + newData2.size());
		newData.insert(newData.end(), newData1.begin(), newData1.end());
		newData.insert(newData.end(), newData2.begin(), newData2.end());
	} else {
		// Si todas son menores pues nos quedamos únicamente con el primero y el último
		newData.push_back(data.front());
		newData.push_back(data.back());
	}
	return newData;
}

/** Identify wall from the points of de 2D map 
Create a line between each two points.
Each line will get a score based in his probability to be a wall*/
float RNHallwayController::identifyWall(const std::vector<PointXY>& data)
{
	float dist = 0.0;
	float ang = 0.0;
	float m = 0.0;
	float n = 0.0;
	float error = 0.0;
	float score = 0.0;
	float maxScore = std::numeric_limits<float>::infinity();
	float distanceScore = 0.0;
	float angleScore = 0.0;
	float wallPositionX = 0.0;


	float maxLineDistance = (3.0 / 4.0) * MAX_MAP_Y;

	// Line obtained by each point and his FOLLOWER
	for(int i = 0; i < (data.size() - 1); i++) // size() - 1 
	{
		/** Data and filters */
		// Distancia de la recta
		dist = std::sqrt( pow( (data.at(i).getX() - data.at(i+1).getX()), 2) + pow( (data.at(i).getY() - data.at(i+1).getY()), 2) );

		// First filter -> Short lines are discarted
		if(dist < MIN_LINE_DISTANCE){
			// Ecuación de la recta
			m = (data.at(i).getY() - data.at(i+1).getY()) / (data.at(i).getX() - data.at(i+1).getX());
			n = data.at(i).getY()- m*data.at(i).getX();

			// Second filter -> Too incline lines are discarted
			ang = std::atan(m);
			// Want to have angle in range 0 - 180 
			while(ang > M_PI){
				ang = ang - M_PI;
			}
			while(ang < 0.0){
				ang = ang + M_PI;
			}
			if(ang < (EXPECTED_LINE_ANGLE + ACCEPTABLE_ERROR_ANGLE) and ang > (EXPECTED_LINE_ANGLE - ACCEPTABLE_ERROR_ANGLE)){
				/** Score */
				// Score for distance
				distanceScore = (dist * 100.0 / MAX_LINE_DISTANCE);
				if(distanceScore > 100.0){
					distanceScore = 100.0;
				}
				distanceScore *= DISTANCE_PERCENTAGE;

				// Score for angle
				error = std::abs(EXPECTED_LINE_ANGLE - ang);
				angleScore = (100.0 - error * 100.0 / ACCEPTABLE_ERROR_ANGLE);
				angleScore *= ANGLE_PERCENTAGE;

				// There might be a score for tracking. But I'm scared of losing the info and be unable to retake it

				// Final score
				score = distanceScore + angleScore;

				// If it is the best line at the moment -> calculate the data of the wall
				if(score > maxScore)
				{
					maxScore = score;

					// Intersection of the line with the axis X -> y = 0 -> x = -n/m
					wallPositionX = (-1) * n / m;
				}
			}
		}
	} // for

	// Last filter
	/*if(max_score > minAcceptableScore)
		
	else
		return 0.0;*/
	return wallPositionX;
}

/** end VÍCTOR JIMÉNEZ BERMEJO*/

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


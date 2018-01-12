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

	firstIteration = true;

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
	laserLeftZone->setRange(0.000, 3.000);
	laserLeftZone->setLockValueInRange(false);
	laserLeftZone->addTerm(new fl::Triangle("VN", 0.000, 0.500, 1.000));
	laserLeftZone->addTerm(new fl::Triangle("N", 0.500, 1.000, 1.500));
	laserLeftZone->addTerm(new fl::Triangle("M", 1.000, 1.500, 2.000));
	laserLeftZone->addTerm(new fl::Triangle("F", 1.500, 2.000, 2.500));
	laserLeftZone->addTerm(new fl::Triangle("VF", 2.000, 2.500, 3.000));
	engine->addInputVariable(laserLeftZone);

	laserFrontZone = new fl::InputVariable;
	laserFrontZone->setName("laserFrontZone");
	laserFrontZone->setDescription("");
	laserFrontZone->setEnabled(true);
	laserFrontZone->setRange(0.000, 3.000);
	laserFrontZone->setLockValueInRange(false);
	laserFrontZone->addTerm(new fl::Triangle("VN", 0.000, 0.500, 1.000));
	laserFrontZone->addTerm(new fl::Triangle("N", 0.500, 1.000, 1.500));
	laserFrontZone->addTerm(new fl::Triangle("M", 1.000, 1.500, 2.000));
	laserFrontZone->addTerm(new fl::Triangle("F", 1.500, 2.000, 2.500));
	laserFrontZone->addTerm(new fl::Triangle("VF", 2.000, 2.500, 3.000));
	engine->addInputVariable(laserFrontZone);

	laserRightZone = new fl::InputVariable;
	laserRightZone->setName("laserRightZone");
	laserRightZone->setDescription("");
	laserRightZone->setEnabled(true);
	laserRightZone->setRange(0.000, 3.000);
	laserRightZone->setLockValueInRange(false);
	laserRightZone->addTerm(new fl::Triangle("VN", 0.000, 0.500, 1.000));
	laserRightZone->addTerm(new fl::Triangle("N", 0.500, 1.000, 1.500));
	laserRightZone->addTerm(new fl::Triangle("M", 1.000, 1.500, 2.000));
	laserRightZone->addTerm(new fl::Triangle("F", 1.500, 2.000, 2.500));
	laserRightZone->addTerm(new fl::Triangle("VF", 2.000, 2.500, 3.000));
	engine->addInputVariable(laserRightZone);


	/*Velocidad linear más alta cuanto más lejos está de las paredes, tiene que frenar si está cerca de una pared*/
	/*Velocidades en cm/s */
	linearVelocity = new fl::OutputVariable;
	linearVelocity->setName("linearVelocity");
	linearVelocity->setDescription("");
	linearVelocity->setEnabled(true);
	linearVelocity->setRange(-4.000, 8.000);
	linearVelocity->setLockValueInRange(false);
	linearVelocity->setAggregation(new fl::Maximum);
	linearVelocity->setDefuzzifier(new fl::Centroid(200));
	linearVelocity->setDefaultValue(fl::nan);
	linearVelocity->setLockPreviousValue(false);
	linearVelocity->addTerm(new fl::Triangle("Backwards", -4.000, -2.000, 0.000));
	linearVelocity->addTerm(new fl::Triangle("Zero", -2.000, 0.000, 2.000));
	linearVelocity->addTerm(new fl::Triangle("Slow Forward", 0.000, 2.000, 4.000));
	linearVelocity->addTerm(new fl::Triangle("Medium Forward", 2.000, 4.000, 6.000));
	linearVelocity->addTerm(new fl::Triangle("Fast Forward", 4.000, 6.000, 8.000));
	engine->addOutputVariable(linearVelocity);

	/*Velocidad angular más alta cuanto más cerca está de las paredes, tiene que ser 0 si está muy lejos*/
	/*Velocidad en radianes por segundo*/
	angularVelocity = new fl::OutputVariable;
	angularVelocity->setName("angularVelocity");
	angularVelocity->setDescription("");
	angularVelocity->setEnabled(true);
	angularVelocity->setRange(-1.000, 1.000);
	angularVelocity->setLockValueInRange(false);
	angularVelocity->setAggregation(new fl::Maximum);
	angularVelocity->setDefuzzifier(new fl::Centroid(200));
	angularVelocity->setDefaultValue(fl::nan);
	angularVelocity->setLockPreviousValue(false);
	angularVelocity->addTerm(new fl::Triangle("Fast Right", -1.000, -0.666, -0.333));
	angularVelocity->addTerm(new fl::Triangle("Slow Right", -0.666, -0.333, 0.000));
	angularVelocity->addTerm(new fl::Triangle("Zero", -0.333, 0.000, 0.333));
	angularVelocity->addTerm(new fl::Triangle("Slow Left", 0.000, 0.333, 0.666));
	angularVelocity->addTerm(new fl::Triangle("Fast Left", 0.333, 0.666, 1.000));
	engine->addOutputVariable(angularVelocity);

	

	ruleBlock = new fl::RuleBlock;
	ruleBlock->setName("mamdani");
	ruleBlock->setDescription("");
	ruleBlock->setEnabled(true);
	ruleBlock->setConjunction(fl::null);
	ruleBlock->setDisjunction(fl::null);
	ruleBlock->setImplication(new fl::Minimum);
	ruleBlock->setActivation(new fl::General);

	/*******/
	
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VF and laserFrontZone is VF and laserRightZone is VF then linearVelocity is Fast Forward and angularVelocity is Zero"));
	
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VF and laserFrontZone is VF and laserRightZone is F  then linearVelocity is Medium Forward 	and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VF and laserFrontZone is VF and laserRightZone is M  then linearVelocity is Slow Forward 	and angularVelocity is Slow Left"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VF and laserFrontZone is VF and laserRightZone is N  then linearVelocity is Zero 			and angularVelocity is Slow Left"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VF and laserFrontZone is VF and laserRightZone is VN then linearVelocity is Backwards 		and angularVelocity is Fast Left"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VF and laserFrontZone is F  and laserRightZone is VF then linearVelocity is Medium Forward 	and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VF and laserFrontZone is F  and laserRightZone is F  then linearVelocity is Medium Forward 	and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VF and laserFrontZone is F  and laserRightZone is M  then linearVelocity is Slow Forward 	and angularVelocity is Slow Left"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VF and laserFrontZone is F  and laserRightZone is N  then linearVelocity is Zero 			and angularVelocity is Slow Left"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VF and laserFrontZone is F  and laserRightZone is VN then linearVelocity is Backwards 		and angularVelocity is Fast Left"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VF and laserFrontZone is M  and laserRightZone is VF then linearVelocity is Slow Forward 	and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VF and laserFrontZone is M  and laserRightZone is F  then linearVelocity is Slow Forward	and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VF and laserFrontZone is M  and laserRightZone is M  then linearVelocity is Slow Forward	and angularVelocity is Slow Left"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VF and laserFrontZone is M  and laserRightZone is N  then linearVelocity is Zero 			and angularVelocity is Slow Left"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VF and laserFrontZone is M  and laserRightZone is VN then linearVelocity is Backwards 		and angularVelocity is Fast Left"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VF and laserFrontZone is N  and laserRightZone is VF then linearVelocity is Zero 			and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VF and laserFrontZone is N  and laserRightZone is F  then linearVelocity is Zero 			and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VF and laserFrontZone is N  and laserRightZone is M  then linearVelocity is Zero 			and angularVelocity is Slow Left"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VF and laserFrontZone is N  and laserRightZone is N  then linearVelocity is Zero 			and angularVelocity is Slow Left"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VF and laserFrontZone is N  and laserRightZone is VN then linearVelocity is Backwards 		and angularVelocity is Fast Left"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VF and laserFrontZone is VN and laserRightZone is VF then linearVelocity is Backwards 		and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VF and laserFrontZone is VN and laserRightZone is F  then linearVelocity is Backwards 		and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VF and laserFrontZone is VN and laserRightZone is M  then linearVelocity is Backwards 		and angularVelocity is Slow Left"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VF and laserFrontZone is VN and laserRightZone is N  then linearVelocity is Backwards 		and angularVelocity is Slow Left"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VF and laserFrontZone is VN and laserRightZone is VN then linearVelocity is Backwards 		and angularVelocity is Fast Left"));
	
	/*******/
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is F and laserFrontZone is VF and laserRightZone is VF then linearVelocity is Medium Forward 	and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is F and laserFrontZone is VF and laserRightZone is F  then linearVelocity is Medium Forward 	and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is F and laserFrontZone is VF and laserRightZone is M  then linearVelocity is Slow Forward 	and angularVelocity is Slow Left"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is F and laserFrontZone is VF and laserRightZone is N  then linearVelocity is Zero 			and angularVelocity is Slow Left"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is F and laserFrontZone is VF and laserRightZone is VN then linearVelocity is Backwards 		and angularVelocity is Fast Left"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is F and laserFrontZone is F  and laserRightZone is VF then linearVelocity is Medium Forward 	and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is F and laserFrontZone is F  and laserRightZone is F  then linearVelocity is Medium Forward 	and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is F and laserFrontZone is F  and laserRightZone is M  then linearVelocity is Slow Forward 	and angularVelocity is Slow Left"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is F and laserFrontZone is F  and laserRightZone is N  then linearVelocity is Zero 			and angularVelocity is Slow Left"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is F and laserFrontZone is F  and laserRightZone is VN then linearVelocity is Backwards 		and angularVelocity is Fast Left"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is F and laserFrontZone is M  and laserRightZone is VF then linearVelocity is Slow Forward 	and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is F and laserFrontZone is M  and laserRightZone is F  then linearVelocity is Slow Forward 	and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is F and laserFrontZone is M  and laserRightZone is M  then linearVelocity is Slow Forward 	and angularVelocity is Slow Left"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is F and laserFrontZone is M  and laserRightZone is N  then linearVelocity is Zero 			and angularVelocity is Slow Left"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is F and laserFrontZone is M  and laserRightZone is VN then linearVelocity is Backwards 		and angularVelocity is Fast Left"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is F and laserFrontZone is N  and laserRightZone is VF then linearVelocity is Zero 			and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is F and laserFrontZone is N  and laserRightZone is F  then linearVelocity is Zero 			and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is F and laserFrontZone is N  and laserRightZone is M  then linearVelocity is Zero 			and angularVelocity is Slow Left"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is F and laserFrontZone is N  and laserRightZone is N  then linearVelocity is Zero 			and angularVelocity is Slow Left"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is F and laserFrontZone is N  and laserRightZone is VN then linearVelocity is Backwards 		and angularVelocity is Fast Left"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is F and laserFrontZone is VN and laserRightZone is VF then linearVelocity is Backwards 		and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is F and laserFrontZone is VN and laserRightZone is F  then linearVelocity is Backwards 		and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is F and laserFrontZone is VN and laserRightZone is M  then linearVelocity is Backwards 		and angularVelocity is Slow Left"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is F and laserFrontZone is VN and laserRightZone is N  then linearVelocity is Backwards 		and angularVelocity is Slow Left"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is F and laserFrontZone is VN and laserRightZone is VN then linearVelocity is Backwards 		and angularVelocity is Fast Left"));
	/*******/
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is M and laserFrontZone is VF and laserRightZone is VF then linearVelocity is Slow Forward 	and angularVelocity is Slow Right"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is M and laserFrontZone is VF and laserRightZone is F  then linearVelocity is Slow Forward 	and angularVelocity is Slow Right"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is M and laserFrontZone is VF and laserRightZone is M  then linearVelocity is Slow Forward 	and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is M and laserFrontZone is VF and laserRightZone is N  then linearVelocity is Zero 			and angularVelocity is Slow Left"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is M and laserFrontZone is VF and laserRightZone is VN then linearVelocity is Backwards 		and angularVelocity is Fast Left"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is M and laserFrontZone is F  and laserRightZone is VF then linearVelocity is Slow Forward 	and angularVelocity is Slow Right"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is M and laserFrontZone is F  and laserRightZone is F  then linearVelocity is Slow Forward 	and angularVelocity is Slow Right"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is M and laserFrontZone is F  and laserRightZone is M  then linearVelocity is Slow Forward 	and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is M and laserFrontZone is F  and laserRightZone is N  then linearVelocity is Zero 			and angularVelocity is Slow Left"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is M and laserFrontZone is F  and laserRightZone is VN then linearVelocity is Backwards 		and angularVelocity is Fast Left"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is M and laserFrontZone is M  and laserRightZone is VF then linearVelocity is Slow Forward 	and angularVelocity is Slow Right"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is M and laserFrontZone is M  and laserRightZone is F  then linearVelocity is Slow Forward 	and angularVelocity is Slow Right"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is M and laserFrontZone is M  and laserRightZone is M  then linearVelocity is Slow Forward 	and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is M and laserFrontZone is M  and laserRightZone is N  then linearVelocity is Zero 			and angularVelocity is Slow Left"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is M and laserFrontZone is M  and laserRightZone is VN then linearVelocity is Backwards 		and angularVelocity is Fast Left"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is M and laserFrontZone is N  and laserRightZone is VF then linearVelocity is Zero 			and angularVelocity is Slow Right"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is M and laserFrontZone is N  and laserRightZone is F  then linearVelocity is Zero 			and angularVelocity is Slow Right"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is M and laserFrontZone is N  and laserRightZone is M  then linearVelocity is Zero 			and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is M and laserFrontZone is N  and laserRightZone is N  then linearVelocity is Zero 			and angularVelocity is Slow Left"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is M and laserFrontZone is N  and laserRightZone is VN then linearVelocity is Backwards 		and angularVelocity is Fast Left"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is M and laserFrontZone is VN and laserRightZone is VF then linearVelocity is Backwards 		and angularVelocity is Slow Right"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is M and laserFrontZone is VN and laserRightZone is F  then linearVelocity is Backwards 		and angularVelocity is Slow Right"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is M and laserFrontZone is VN and laserRightZone is M  then linearVelocity is Backwards 		and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is M and laserFrontZone is VN and laserRightZone is N  then linearVelocity is Backwards 		and angularVelocity is Slow Left"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is M and laserFrontZone is VN and laserRightZone is VN then linearVelocity is Backwards 		and angularVelocity is Fast Left"));
	/*******/
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is N and laserFrontZone is VF and laserRightZone is VF then linearVelocity is Zero 		and angularVelocity is Slow Right"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is N and laserFrontZone is VF and laserRightZone is F  then linearVelocity is Zero 		and angularVelocity is Slow Right"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is N and laserFrontZone is VF and laserRightZone is M  then linearVelocity is Zero 		and angularVelocity is Slow Right"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is N and laserFrontZone is VF and laserRightZone is N  then linearVelocity is Zero 		and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is N and laserFrontZone is VF and laserRightZone is VN then linearVelocity is Backwards 	and angularVelocity is Slow Left"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is N and laserFrontZone is F  and laserRightZone is VF then linearVelocity is Zero 		and angularVelocity is Slow Right"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is N and laserFrontZone is F  and laserRightZone is F  then linearVelocity is Zero 		and angularVelocity is Slow Right"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is N and laserFrontZone is F  and laserRightZone is M  then linearVelocity is Zero 		and angularVelocity is Slow Right"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is N and laserFrontZone is F  and laserRightZone is N  then linearVelocity is Zero 		and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is N and laserFrontZone is F  and laserRightZone is VN then linearVelocity is Backwards 	and angularVelocity is Slow Left"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is N and laserFrontZone is M  and laserRightZone is VF then linearVelocity is Zero 		and angularVelocity is Slow Right"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is N and laserFrontZone is M  and laserRightZone is F  then linearVelocity is Zero 		and angularVelocity is Slow Right"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is N and laserFrontZone is M  and laserRightZone is M  then linearVelocity is Zero 		and angularVelocity is Slow Right"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is N and laserFrontZone is M  and laserRightZone is N  then linearVelocity is Zero 		and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is N and laserFrontZone is M  and laserRightZone is VN then linearVelocity is Backwards 	and angularVelocity is Slow Left"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is N and laserFrontZone is N  and laserRightZone is VF then linearVelocity is Zero 		and angularVelocity is Slow Right"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is N and laserFrontZone is N  and laserRightZone is F  then linearVelocity is Zero 		and angularVelocity is Slow Right"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is N and laserFrontZone is N  and laserRightZone is M  then linearVelocity is Zero 		and angularVelocity is Slow Right"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is N and laserFrontZone is N  and laserRightZone is N  then linearVelocity is Zero 		and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is N and laserFrontZone is N  and laserRightZone is VN then linearVelocity is Backwards 	and angularVelocity is Slow Left"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is N and laserFrontZone is VN and laserRightZone is VF then linearVelocity is Backwards 	and angularVelocity is Slow Right"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is N and laserFrontZone is VN and laserRightZone is F  then linearVelocity is Backwards 	and angularVelocity is Slow Right"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is N and laserFrontZone is VN and laserRightZone is M  then linearVelocity is Backwards 	and angularVelocity is Slow Right"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is N and laserFrontZone is VN and laserRightZone is N  then linearVelocity is Backwards 	and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is N and laserFrontZone is VN and laserRightZone is VN then linearVelocity is Backwards 	and angularVelocity is Slow Left"));
	/*******/
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VN and laserFrontZone is VF and laserRightZone is VF then linearVelocity is Backwards and angularVelocity is Fast Right"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VN and laserFrontZone is VF and laserRightZone is F  then linearVelocity is Backwards and angularVelocity is Fast Right"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VN and laserFrontZone is VF and laserRightZone is M  then linearVelocity is Backwards and angularVelocity is Fast Right"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VN and laserFrontZone is VF and laserRightZone is N  then linearVelocity is Backwards and angularVelocity is Slow Right"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VN and laserFrontZone is VF and laserRightZone is VN then linearVelocity is Backwards and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VN and laserFrontZone is F  and laserRightZone is VF then linearVelocity is Backwards and angularVelocity is Fast Right"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VN and laserFrontZone is F  and laserRightZone is F  then linearVelocity is Backwards and angularVelocity is Fast Right"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VN and laserFrontZone is F  and laserRightZone is M  then linearVelocity is Backwards and angularVelocity is Fast Right"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VN and laserFrontZone is F  and laserRightZone is N  then linearVelocity is Backwards and angularVelocity is Slow Right"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VN and laserFrontZone is F  and laserRightZone is VN then linearVelocity is Backwards and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VN and laserFrontZone is M  and laserRightZone is VF then linearVelocity is Backwards and angularVelocity is Fast Right"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VN and laserFrontZone is M  and laserRightZone is F  then linearVelocity is Backwards and angularVelocity is Fast Right"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VN and laserFrontZone is M  and laserRightZone is M  then linearVelocity is Backwards and angularVelocity is Fast Right"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VN and laserFrontZone is M  and laserRightZone is N  then linearVelocity is Backwards and angularVelocity is Slow Right"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VN and laserFrontZone is M  and laserRightZone is VN then linearVelocity is Backwards and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VN and laserFrontZone is N  and laserRightZone is VF then linearVelocity is Backwards and angularVelocity is Fast Right"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VN and laserFrontZone is N  and laserRightZone is F  then linearVelocity is Backwards and angularVelocity is Fast Right"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VN and laserFrontZone is N  and laserRightZone is M  then linearVelocity is Backwards and angularVelocity is Fast Right"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VN and laserFrontZone is N  and laserRightZone is N  then linearVelocity is Backwards and angularVelocity is Slow Right"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VN and laserFrontZone is N  and laserRightZone is VN then linearVelocity is Backwards and angularVelocity is Zero"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VN and laserFrontZone is VN and laserRightZone is VF then linearVelocity is Backwards and angularVelocity is Fast Right"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VN and laserFrontZone is VN and laserRightZone is F  then linearVelocity is Backwards and angularVelocity is Fast Right"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VN and laserFrontZone is VN and laserRightZone is M  then linearVelocity is Backwards and angularVelocity is Fast Right"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VN and laserFrontZone is VN and laserRightZone is N  then linearVelocity is Backwards and angularVelocity is Slow Right"));
	ruleBlock->addRule(new fl::Rule("if laserLeftZone is VN and laserFrontZone is VN and laserRightZone is VN then linearVelocity is Backwards and angularVelocity is Zero"));
	engine->addRuleBlock(ruleBlock);
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
		lastFrontInput = getHallwayLastRightInput();
		lastRightInput = getHallwayLastFrontInput();

		setHallwayFuzzyInputs(lastLeftInput, lastFrontInput, lastRightInput);

		engine->process();

		*linearVelocity = this->linearVelocity->getValue();
		*angularVelocity = this->angularVelocity->getValue();
	}
}

void RNHallwayController::setHallwayInputs(const LaserScan* data){

    hallwayLeftInput->clear();
    hallwayFrontInput->clear();
    hallwayRightInput->clear();

    for(int i=0; i < data->size()-241; i++){
    	this->hallwayLeftInput->push_back(data->getRange(i));	//Obtiene las distancias medidas por el láser en los primeros 120 grados
    }
    for(int i=120; i < data->size()-120; i++){
    	this->hallwayFrontInput->push_back(data->getRange(i));
    }
    for(int i=241; i < data->size(); i++){
    	this->hallwayRightInput->push_back(data->getRange(i));
    }
}

void RNHallwayController::setHallwayFuzzyInputs(double lastLeftInput, double lastFrontInput, double lastRightInput){
	/*Hay que establecer el número de inputs*/
	const int numberOfInputs = 50;
	
	int counterLeftInputs = 0;
	int counterFrontInputs = 0;
	int counterRightInputs = 0;

	/*Cuidado con el orden de magnitud del check distance y de las iteraciones*/
	double doorCheckDistance = 0.1;
	double doorCheckIterations = 100;

	double counterLeftDoorCheckIterations;
	double counterRightDoorCheckIterations;

	double hallwayLeftFuzzyInput;
	double hallwayFrontFuzzyInput;
	double hallwayRightFuzzyInput;

	double lastInputsLSector[numberOfInputs];
	double lastInputsFSector[numberOfInputs];
	double lastInputsRSector[numberOfInputs];

	double lastInputsLeftDoorCheck[numberOfInputs];
	double lastInputsRightDoorCheck[numberOfInputs];

	if ((lastLeftInput-hallwayLeftFuzzyInput)>doorCheckDistance){
		if (counterLeftDoorCheckIterations<doorCheckIterations){
			for(int i=0; i<numberOfInputs; i++){
				lastInputsLeftDoorCheck[i + 1] = lastInputsLeftDoorCheck[i];
			}
			lastInputsLeftDoorCheck[0] = lastLeftInput;	
			counterLeftDoorCheckIterations++;
		}
		else{
			for(int i=0; i<numberOfInputs; i++){
				lastInputsLSector[i] = lastInputsLeftDoorCheck[i];
				counterLeftDoorCheckIterations = 0;
			}
		}
	}
	else{
		for(int i=0; i<numberOfInputs; i++){
			lastInputsLSector[i+1]=lastInputsLSector[i];
		}
		lastInputsLSector[0]=lastLeftInput;
		for(int i=0; i<numberOfInputs; i++){
			counterLeftInputs+=lastInputsLSector[i];
		}
		hallwayLeftFuzzyInput = counterLeftInputs/numberOfInputs;
		counterLeftDoorCheckIterations = 0;
	}

	this->laserLeftZone->setValue(hallwayLeftFuzzyInput);

	for(int i=0; i<numberOfInputs; i++){
			lastInputsFSector[i+1]=lastInputsFSector[i];
		}
	lastInputsFSector[0]=lastFrontInput;
	for(int i=0; i<numberOfInputs; i++){
		counterFrontInputs+=lastInputsFSector[i];
	}
	hallwayFrontFuzzyInput = counterFrontInputs/numberOfInputs;
	this->laserFrontZone->setValue(hallwayFrontFuzzyInput);
	
	if ((lastRightInput-hallwayRightFuzzyInput)>doorCheckDistance){
		if (counterRightDoorCheckIterations<doorCheckIterations){
			for(int i=0; i<numberOfInputs; i++){
				lastInputsRightDoorCheck[i+1] = lastInputsRightDoorCheck[i];
			}
			lastInputsRightDoorCheck[0] = lastRightInput;	
			counterRightDoorCheckIterations++;
		}
		else{
			for(int i=0; i<numberOfInputs; i++){
				lastInputsRSector[i] = lastInputsRightDoorCheck[i];
				counterRightDoorCheckIterations = 0;
			}
		}
	}
	else{
		for(int i=0; i<numberOfInputs; i++){
			lastInputsRSector[i+1]=lastInputsRSector[i];
		}
		lastInputsRSector[0]=lastRightInput;
		for(int i=0; i<numberOfInputs; i++){
			counterRightInputs+=lastInputsRSector[i];
		}
		hallwayRightFuzzyInput = counterRightInputs/numberOfInputs;
		counterRightDoorCheckIterations = 0;
	}
	this->laserRightZone->setValue(hallwayRightFuzzyInput);
}

double RNHallwayController::getHallwayLastLeftInput(void){
	int hallwayLeftFuzzyInput = 0;
	int hallwayLSectorVN = 0;
	int hallwayLSectorN = 0;
	int hallwayLSectorM = 0;
	int hallwayLSectorF = 0;
	int hallwayLSectorVF = 0;
	int counterVN = 0;
	int counterN = 0;
	int counterM = 0;
	int counterF = 0;
	int counterVF = 0;
	float lastLeftInput;

	for(int i=0; i < hallwayLeftInput->size(); i++){
		if (hallwayLeftInput->at(i)<0.5){
			counterVN++;

			hallwayLSectorVN += hallwayLeftInput->at(i);
		}
		else if (0.5<hallwayLeftInput->at(i)<1){
			counterN++;
			hallwayLSectorN += hallwayLeftInput->at(i);
		}
		else if (1<hallwayLeftInput->at(i)<1.5){
			counterM++;
			hallwayLSectorM += hallwayLeftInput->at(i);
		}
		else if (1.5<hallwayLeftInput->at(i)<2){
			counterF++;
			hallwayLSectorF += hallwayLeftInput->at(i);
		}
		else {
			counterVF++;
			hallwayLSectorVF += hallwayLeftInput->at(i);
		}
	}

	if (counterVN > 10){
		lastLeftInput = (float)hallwayLSectorVN/(float)counterVN;
	} else if (counterN > 10){
		lastLeftInput = (float)hallwayLSectorN/(float)counterN;
	} else if (counterM > 10){
		lastLeftInput = (float)hallwayLSectorM/(float)counterM;
	} else if (counterF > 10){
		lastLeftInput = (float)hallwayLSectorF/(float)counterF;
	}else {
		lastLeftInput = (float)hallwayLSectorVF/(float)counterVF;
	}
		
	return lastLeftInput;
}

double RNHallwayController::getHallwayLastFrontInput(void){
	int hallwayFrontFuzzyInput = 0;
	int hallwayFSectorVN = 0;
	int hallwayFSectorN = 0;
	int hallwayFSectorM = 0;
	int hallwayFSectorF = 0;
	int hallwayFSectorVF = 0;
	int counterVN = 0;
	int counterN = 0;
	int counterM = 0;
	int counterF = 0;
	int counterVF = 0;
	float lastFrontInput;
	

	for(int i=0; i < hallwayFrontInput->size(); i++){
		if (hallwayFrontInput->at(i) < 0.5){
			counterVN += 1;
			hallwayFSectorVN += hallwayFrontInput->at(i);
		}
		else if (0.5<hallwayFrontInput->at(i)<1){
			counterN += 1;
			hallwayFSectorN += hallwayFrontInput->at(i);
		}
		else if (1<hallwayFrontInput->at(i)<1.5){
			counterM += 1;
			hallwayFSectorM += hallwayFrontInput->at(i);
		}
		else if (1.5<hallwayFrontInput->at(i)<2){
			counterF += 1;
			hallwayFSectorF += hallwayFrontInput->at(i);
		}
		else {
			counterVF += 1;
			hallwayFSectorVF += hallwayFrontInput->at(i);
		}
	}

	if (counterVN > 10){
		lastFrontInput = (float)hallwayFSectorVN/(float)counterVN;
	} else if (counterN > 10){
		lastFrontInput = (float)hallwayFSectorN/(float)counterN;
	} else if (counterM > 10){
		lastFrontInput = (float)hallwayFSectorM/(float)counterM;
	} else if (counterF > 10){
		lastFrontInput = (float)hallwayFSectorF/(float)counterF;
	}else {
		lastFrontInput = (float)hallwayFSectorVF/(float)counterVF;
	}

	return lastFrontInput;
}

double RNHallwayController::getHallwayLastRightInput(void){
	int hallwayRightFuzzyInput = 0;
	int hallwayRSectorVN = 0;
	int hallwayRSectorN = 0;
	int hallwayRSectorM = 0;
	int hallwayRSectorF = 0;
	int hallwayRSectorVF = 0;
	int counterVN = 0;
	int counterN = 0;
	int counterM = 0;
	int counterF = 0;
	int counterVF = 0;
	float lastRightInput;

	for(int i=241; i < hallwayRightInput->size(); i++){
		if (hallwayRightInput->at(i)<0.5){
			counterVN += 1;
			hallwayRSectorVN += hallwayRightInput->at(i);
		}
		else if (0.5<hallwayRightInput->at(i)<1){
			counterN += 1;
			hallwayRSectorN += hallwayRightInput->at(i);
		}
		else if (1<hallwayRightInput->at(i)<1.5){
			counterM += 1;
			hallwayRSectorM += hallwayRightInput->at(i);
		}
		else if (1.5<hallwayRightInput->at(i)<2){
			counterF += 1;
			hallwayRSectorF += hallwayRightInput->at(i);
		}
		else {
			counterVF += 1;
			hallwayRSectorVF += hallwayRightInput->at(i);
		}
	}

	if (counterVN > 10){
		lastRightInput = (float)hallwayRSectorVN/(float)counterVN;
	} else if (counterN > 10){
		lastRightInput = (float)hallwayRSectorN/(float)counterN;
	} else if (counterM > 10){
		lastRightInput = (float)hallwayRSectorM/(float)counterM;
	} else if (counterF > 10){
		lastRightInput = (float)hallwayRSectorF/(float)counterF;
	}else {
		lastRightInput = (float)hallwayRSectorVF/(float)counterVF;
	}

	return lastRightInput;
}


double RNHallwayController::getLinearVelocity(void){
	return linearVelocity->getValue();
}

double RNHallwayController::getAngularVelocity(void){
	return angularVelocity->getValue();
}


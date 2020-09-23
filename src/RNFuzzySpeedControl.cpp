#include "RNFuzzySpeedControl.h"

RNFuzzySpeedControl::RNFuzzySpeedControl(const char* name, const char* description){
	/*********** Speed control - Lateral obstacle *************/
	engineLat = new fl::Engine;
	engineLat->setName("SpeedControlLat");
	engineLat->setDescription("");
	
	//distances in meters
	distLat = new fl::InputVariable;
	distLat->setName("distObs");
	distLat->setDescription("");
	distLat->setEnabled(true);
	distLat->setRange(0, std::numeric_limits<double>::infinity());
	distLat->setLockValueInRange(true);
	distLat->addTerm(new fl::Triangle("close", -std::numeric_limits<double>::infinity(), 0.3, 1));
	distLat->addTerm(new fl::Triangle("far", 0.3, 1, std::numeric_limits<double>::infinity()));
	engineLat->addInputVariable(distLat);


	//Fuzzy Factor adimensional
	factorLat = new fl::OutputVariable;
	factorLat->setName("factor");
	factorLat->setDescription("");
	factorLat->setEnabled(true);
	factorLat->setRange(0, 1);
	factorLat->setLockValueInRange(true);
	factorLat->setAggregation(new fl::AlgebraicSum);
	factorLat->setDefuzzifier(new fl::Centroid(100));
	factorLat->setDefaultValue(0.0);
	factorLat->setLockPreviousValue(false);
	factorLat->addTerm(new fl::Triangle("null", 0.15, 0.65, 1.15));
	factorLat->addTerm(new fl::Triangle("full", 0.5, 1, 1.5));
	engineLat->addOutputVariable(factorLat);


	ruleBlockLat = new fl::RuleBlock;
	ruleBlockLat->setName("mamdani");
	ruleBlockLat->setDescription("");
	ruleBlockLat->setEnabled(true);
	ruleBlockLat->setConjunction(new fl::AlgebraicProduct);
	ruleBlockLat->setDisjunction(new fl::AlgebraicSum);
	ruleBlockLat->setImplication(new fl::AlgebraicProduct);
	ruleBlockLat->setActivation(new fl::General);

	ruleBlockLat->addRule(fl::Rule::parse("if distLat is close then factorLat is null", engineLat));
	ruleBlockLat->addRule(fl::Rule::parse("if distLat is far then factorLat is full", engineLat));
	
	engineLat->addRuleBlock(ruleBlockLat);
	fl::fuzzylite::setDebugging(false);
	RNUtils::printLn("%s has been instantiated...", name);
	
	/*********** Speed control - Lateral obstacle *************/
	engineFront = new fl::Engine;
	engineFront->setName("SpeedControlFront");
	engineFront->setDescription("");
	
	//distances in meters
	distFront = new fl::InputVariable;
	distFront->setName("distFront");
	distFront->setDescription("");
	distFront->setEnabled(true);
	distFront->setRange(0, std::numeric_limits<double>::infinity());
	distFront->setLockValueInRange(true);
	distFront->addTerm(new fl::Triangle("close", -std::numeric_limits<double>::infinity(), 0.5, 2));
	distFront->addTerm(new fl::Triangle("far", 0.5, 2, std::numeric_limits<double>::infinity()));
	engineFront->addInputVariable(distFront);


	//Fuzzy Factor adimensional
	factorFront = new fl::OutputVariable;
	factorFront->setName("factorFront");
	factorFront->setDescription("");
	factorFront->setEnabled(true);
	factorFront->setRange(0, 1);
	factorFront->setLockValueInRange(true);
	factorFront->setAggregation(new fl::AlgebraicSum);
	factorFront->setDefuzzifier(new fl::Centroid(100));
	factorFront->setDefaultValue(0.0);
	factorFront->setLockPreviousValue(false);
	factorFront->addTerm(new fl::Triangle("null", -0.5, 0, 0.5));
	factorFront->addTerm(new fl::Triangle("full", 0.5, 1, 1.5));
	engineFront->addOutputVariable(factorFront);


	ruleBlockFront = new fl::RuleBlock;
	ruleBlockFront->setName("mamdani");
	ruleBlockFront->setDescription("");
	ruleBlockFront->setEnabled(true);
	ruleBlockFront->setConjunction(new fl::AlgebraicProduct);
	ruleBlockFront->setDisjunction(new fl::AlgebraicSum);
	ruleBlockFront->setImplication(new fl::AlgebraicProduct);
	ruleBlockFront->setActivation(new fl::General);

	ruleBlockFront->addRule(fl::Rule::parse("if distFront is close then factorFront is null", engineFront));
	ruleBlockFront->addRule(fl::Rule::parse("if distFront is far then factorFront is full", engineFront));
	
	engineFront->addRuleBlock(ruleBlockFront);
	fl::fuzzylite::setDebugging(false);
	RNUtils::printLn("%s has been instantiated...", name);
}

RNFuzzySpeedControl::~RNFuzzySpeedControl(){
	delete engineFront;
	delete engineLat;
}	


void RNFuzzySpeedControl::getSystemInput(float distanceLeft, float distanceRight, float distanceFront, double* linearVelocity, double* angularVelocity){
	
	float factorLeftLocal = 0; float factorRightLocal = 0; float factorFrontLocal = 0;
	
	//LEFT
	this->distLat->setValue(distanceLeft);
	engineLat->process();
	factorLeftLocal = this->factorLat->getValue();
	
	//RIGHT
	this->distLat->setValue(distanceRight);
	engineLat->process();
	factorRightLocal = this->factorLat->getValue();
	
	//FRONT
	this->distFront->setValue(distanceFront);
	engineFront->process();
	factorFrontLocal = this->factorFront->getValue();
	

	*linearVelocity = 0.5 * factorLeftLocal * factorRightLocal * factorFrontLocal; //CHECK
	*angularVelocity = 0.1; //CHECK
}


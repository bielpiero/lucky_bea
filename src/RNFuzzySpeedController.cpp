#include "RNFuzzySpeedController.h"

RNFuzzySpeedController::RNFuzzySpeedController(const char* name, const char* description){
	engine = new fl::Engine;
	engine->setName("SpeedController");
	engine->setDescription("");

	//distances in meters
	distanceError = new fl::InputVariable;
	distanceError->setName("distanceError");
	distanceError->setDescription("");
	distanceError->setEnabled(true);
	distanceError->setRange(-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
	distanceError->setLockValueInRange(false);
	distanceError->addTerm(new fl::Triangle("NVeryFar", -std::numeric_limits<double>::infinity(), -0.750, -0.250));
	distanceError->addTerm(new fl::Triangle("NFar", -0.750, -0.250, -0.070));
	distanceError->addTerm(new fl::Triangle("NNear", -0.250, -0.070, -0.05));
	distanceError->addTerm(new fl::Trapezoid("Zero", -0.070, -0.05, 0.050, 0.070));
	distanceError->addTerm(new fl::Triangle("PNear", 0.005, 0.070, 0.250));
	distanceError->addTerm(new fl::Triangle("PFar", 0.010, 0.250, 0.750));
	distanceError->addTerm(new fl::Triangle("PVeryFar", 0.250, 0.750, std::numeric_limits<double>::infinity()));
	engine->addInputVariable(distanceError);

	//radians
	angleError = new fl::InputVariable;
	angleError->setName("angleError");
	angleError->setDescription("");
	angleError->setEnabled(true);
	angleError->setRange(-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
	angleError->setLockValueInRange(false);
	angleError->addTerm(new fl::Triangle("NVeryFar", -std::numeric_limits<double>::infinity(), -0.02, -0.0157));
	angleError->addTerm(new fl::Triangle("NFar", -0.02, -0.0157, -0.0122));
	angleError->addTerm(new fl::Triangle("NNear", -0.0157, -0.0122, -0.0087));
	angleError->addTerm(new fl::Trapezoid("Zero", -0.0122, -0.0087, 0.0087, 0.0122));
	angleError->addTerm(new fl::Triangle("PNear", 0.0087, 0.0122, 0.0157));
	angleError->addTerm(new fl::Triangle("PFar", 0.0122, 0.0157, 0.02));
	angleError->addTerm(new fl::Triangle("PVeryFar", 0.0157, 0.02, std::numeric_limits<double>::infinity()));
	engine->addInputVariable(angleError);


	//linear speed in mm/s
	linearVelocity = new fl::OutputVariable;
	linearVelocity->setName("linearVelocity");
	linearVelocity->setDescription("");
	linearVelocity->setEnabled(true);
	linearVelocity->setRange(-80.000, 80.000);
	linearVelocity->setLockValueInRange(false);
	linearVelocity->setAggregation(new fl::AlgebraicSum);
	linearVelocity->setDefuzzifier(new fl::Centroid(100));
	linearVelocity->setDefaultValue(0.0);
	linearVelocity->setLockPreviousValue(false);
	linearVelocity->addTerm(new fl::Triangle("FastBackwards", -80.000, -60.000, -40.000));
	linearVelocity->addTerm(new fl::Triangle("Backwards", -60.000, -40.000, 20.000));
	linearVelocity->addTerm(new fl::Triangle("MediumBackwards", -40.000, -20.000, 0.000));
	linearVelocity->addTerm(new fl::Triangle("Zero", -20.000, 0.000, 20.000));
	linearVelocity->addTerm(new fl::Triangle("MediumForward", 0.000, 20.000, 40.000));
	linearVelocity->addTerm(new fl::Triangle("Forward", 20.000, 40.000, 60.000));
	linearVelocity->addTerm(new fl::Triangle("FastForward", 40.000, 60.000, 80.000));
	engine->addOutputVariable(linearVelocity);

	
	/*angular speed in radians per second*/
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
	angularVelocity->addTerm(new fl::Triangle("FastRight", -0.088, -0.066, -0.044));
	angularVelocity->addTerm(new fl::Triangle("SlowRight", -0.066, -0.044, 0.022));
	angularVelocity->addTerm(new fl::Triangle("VSlowRight", -0.044, -0.022, 0.000));
	angularVelocity->addTerm(new fl::Triangle("Zero", -0.022, 0.000, 0.022));
	angularVelocity->addTerm(new fl::Triangle("VSlowLeft", 0.000, 0.080, 0.044));
	angularVelocity->addTerm(new fl::Triangle("SlowLeft", 0.022, 0.044, 0.066));
	angularVelocity->addTerm(new fl::Triangle("FastLeft", 0.044, 0.066, 0.088));
	engine->addOutputVariable(angularVelocity);

	ruleBlock = new fl::RuleBlock;
	ruleBlock->setName("mamdani");
	ruleBlock->setDescription("");
	ruleBlock->setEnabled(true);
	ruleBlock->setConjunction(new fl::AlgebraicProduct);
	ruleBlock->setDisjunction(new fl::AlgebraicSum);
	ruleBlock->setImplication(new fl::AlgebraicProduct);
	ruleBlock->setActivation(new fl::General);

	ruleBlock->addRule(fl::Rule::parse("if angleError is NVeryFar then linearVelocity is Zero and angularVelocity is FastRight", engine));
	ruleBlock->addRule(fl::Rule::parse("if angleError is NFar then linearVelocity is Zero and angularVelocity is SlowRight", engine));
	ruleBlock->addRule(fl::Rule::parse("if angleError is NNear then linearVelocity is Zero and angularVelocity is VSlowRight", engine));
	ruleBlock->addRule(fl::Rule::parse("if angleError is PNear then linearVelocity is Zero and angularVelocity is VSlowLeft", engine));
	ruleBlock->addRule(fl::Rule::parse("if angleError is PFar then linearVelocity is Zero and angularVelocity is SlowLeft", engine));
	ruleBlock->addRule(fl::Rule::parse("if angleError is PVeryFar then linearVelocity is Zero and angularVelocity is FastLeft", engine));
	ruleBlock->addRule(fl::Rule::parse("if angleError is Zero and distanceError is NVeryFar then linearVelocity is FastBackwards and angularVelocity is Zero", engine));
	ruleBlock->addRule(fl::Rule::parse("if angleError is Zero and distanceError is NFar then linearVelocity is Backwards and angularVelocity is Zero", engine));
	ruleBlock->addRule(fl::Rule::parse("if angleError is Zero and distanceError is NNear then linearVelocity is MediumBackwards and angularVelocity is Zero", engine));
	ruleBlock->addRule(fl::Rule::parse("if angleError is Zero and distanceError is PNear then linearVelocity is MediumForward and angularVelocity is Zero", engine));
	ruleBlock->addRule(fl::Rule::parse("if angleError is Zero and distanceError is PFar then linearVelocity is Forward and angularVelocity is Zero", engine));
	ruleBlock->addRule(fl::Rule::parse("if angleError is Zero and distanceError is PVeryFar then linearVelocity is FastForward and angularVelocity is Zero", engine));
	ruleBlock->addRule(fl::Rule::parse("if angleError is Zero and distanceError is Zero then linearVelocity is Zero and angularVelocity is Zero", engine));
	engine->addRuleBlock(ruleBlock);
	fl::fuzzylite::setDebugging(false);
	RNUtils::printLn("%s has been instantiated...", name);
}

RNFuzzySpeedController::~RNFuzzySpeedController(){
	delete engine;
}	

void RNFuzzySpeedController::getTarget(double* distanceTarget, double* angleTarget){
	*distanceTarget = this->distanceTarget;
	*angleTarget = this->angleTarget;
}

void RNFuzzySpeedController::setTarget(const double& distanceTarget, const double& angleTarget){
	this->distanceTarget = distanceTarget;
	this->angleTarget = angleTarget;
}

void RNFuzzySpeedController::getSystemInput(const double& distance, const double& angle, double* linearVelocity, double* angularVelocity){

	this->distanceError->setValue(distance);
	this->angleError->setValue(angle);

	engine->process();

	*linearVelocity = this->linearVelocity->getValue();
	*angularVelocity = this->angularVelocity->getValue();
}
#include "RNPIDController.h"

RNPIDController::RNPIDController(const char* name, double setPoint, double samplingTime, double kp, double ti, double td){
	this->name = new char[strlen(name)];
	strcpy(this->name, name);
	this->samplingTime = samplingTime;

	this->kp = kp;
	this->ti = ti;
	this->td = td;
	firstIteration = true;
	this->setPoint = setPoint;

	this->lastInput = std::numeric_limits<double>::infinity();
	this->lastError = 0;
	this->pastLastError = 0;
	this->currentError = 0;
}

RNPIDController::~RNPIDController(){
	
}

double RNPIDController::getSystemInput(int measure){
	double uk = 0;
	double q0 = 0, q1 = 0, q2 = 0;
	if(firstIteration){
		this->lastInput = 0;
		firstIteration = false;
	}
	this->pastLastError = this->lastError;
	this->lastError = this->currentError;
	this->currentError = this->setPoint - measure;
	if(this->kp != 0 and this->ti == 0 and this->td == 0){
		//Proportional Controller
		uk = this->kp * this->currentError;
	} else if(this->kp != 0 and this->ti != 0 and this->td == 0){
		//Proportional-Integral Controller
		q0 = 1 + (this->samplingTime / this->ti);
		uk = this->lastInput + (this->kp * q0 * this->currentError) - (this->kp * this->lastError);

	} else if(this->kp != 0 and this->ti == 0 and this->td != 0){
		//Proportional-Derivative Controller
		uk = this->kp * this->currentError + this->kp * ((this->td / this->samplingTime) * (this->currentError - this->lastError));
		
	} else if(this->kp != 0 and this->ti != 0 and this->td != 0){
		//Proportional-Integral-Derivative Controller
		q0 = 1 + (this->samplingTime / this->ti) + (this->td / this->samplingTime);
		q1 = -1 - 2 * (this->td / this->samplingTime);
		q2 = this->td / this->samplingTime;

		uk = this->lastInput + (this->kp * q0 * this->currentError) + (this->kp * q1 * this->lastError) + (this->kp * q2 * this->pastLastError);
	}	
	this->lastInput = uk;
	return uk;
}

void RNPIDController::reset(void){
	firstIteration = true;
	this->lastInput = std::numeric_limits<double>::infinity();
	this->lastError = 0;
	this->pastLastError = 0;
	this->currentError = 0;
}

double RNPIDController::getProportionalGain(void){
	return this->kp;
}

void RNPIDController::setProportionalGain(double kp){
	this->kp = kp;
}

double RNPIDController::getIntegrationTime(void){
	return this->ti;
}

void RNPIDController::setIntegrationTime(double ti){
	this->ti = ti;
}

double RNPIDController::getDerivativeTime(void){
	return this->td;
}

void RNPIDController::setDerivativeTime(double td){
	this->td = td;
}

double RNPIDController::getSamplingTime(void){
	return this->samplingTime;
}

void RNPIDController::setSamplingTime(double samplingTime){
	this->samplingTime = samplingTime;
}

double RNPIDController::getTarget(void){
	return this->setPoint;
}

void RNPIDController::setTarget(double target){
	this->setPoint = target;
}

double RNPIDController::getLastInput(void){
	return this->lastInput;
}
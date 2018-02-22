#include "RNPIDController.h"

RNPIDController::RNPIDController(const char* name, float sp, float samplingTime, float kp, float ti, float td, float comp, float lowerSaturation, float upperSaturation){
	this->name = std::string(name);
	this->ts = samplingTime;

	this->kp = kp;
	this->ti = ti;
	this->td = td;

	this->comp = comp;
	this->upperSaturation = upperSaturation;
	this->lowerSaturation = lowerSaturation;

	firstIteration = true;
	this->sp = sp;
	this->iteration = RN_NONE;
	this->uk_1 = std::numeric_limits<float>::infinity();
	this->ek = std::numeric_limits<float>::infinity();
	this->ek_1 = 0;
	this->ek_2 = 0;
}

RNPIDController::~RNPIDController(){
	
}

int RNPIDController::getSystemInput(float measure, float* output){
	float uk = 0;
	float ki = 0;
	float kd = 0;

	if(firstIteration){
		this->uk_1 = 0;
		this->ek = 0;
		firstIteration = false;
	}
	this->ek_2 = this->ek_1;
	this->ek_1 = this->ek;
	this->ek = this->sp - measure;
	if(kp != 0.0){
		if(ti == 0.0 and td == 0.0){
			uk = kp * ek;
		} else if(ti != 0.0 and td == 0.0){
			ki = (kp / ti) * ts;
			uk = this->uk_1 + ((kp + ki) * this->ek) - (kp * this->ek_1);
		} else if(ti == 0.0 and td != 0.0){
			kd = (kp * td) / ts;
			uk = ((kp + kd) * this->ek) - (kd * this->ek_1);
		} else if(ti != 0.0 and td != 0.0){
			ki = (kp / ti) * ts;
			kd = (kp * td) / ts;
			uk = this->uk_1 + ((kp + ki + kd) * this->ek) - ((kp + 2.0 * kd) * this->ek_1) + ((kd) * this->ek_2);
		}
	}
	if(uk > upperSaturation){
		uk = upperSaturation;
	}
	if(uk < lowerSaturation){
		uk = lowerSaturation;
	}
	this->uk_1 = uk;
	*output = uk;
	this->iteration++;
	return this->iteration;
}

void RNPIDController::reset(void){
	firstIteration = true;
	this->uk_1 = std::numeric_limits<float>::infinity();
	this->ek_1 = 0;
	this->ek_2 = 0;
	this->ek = std::numeric_limits<float>::infinity();
	this->iteration = RN_NONE;
}

float RNPIDController::getProportionalGain(void){
	return this->kp;
}

void RNPIDController::setProportionalGain(float kp){
	this->kp = kp;
}

float RNPIDController::getIntegrationTime(void){
	return this->ti;
}

void RNPIDController::setIntegrationTime(float ti){
	this->ti = ti;
}

float RNPIDController::getDerivativeTime(void){
	return this->td;
}

void RNPIDController::setDerivativeTime(float td){
	this->td = td;
}

float RNPIDController::getSamplingTime(void){
	return this->ts;
}

void RNPIDController::setSamplingTime(float ts){
	this->ts = ts;
}

float RNPIDController::getDerivativeFilter(void){
	return this->comp;
}

void RNPIDController::setDerivativeFilter(float comp){
	this->comp = comp;
}

float RNPIDController::getLowerSaturationLimit(void){
	return this->lowerSaturation;
}

void RNPIDController::setLowerSaturationLimit(float saturation){
	this->lowerSaturation = saturation;
}

float RNPIDController::getUpperSaturationLimit(void){
	return upperSaturation;
}

void RNPIDController::setUpperSaturationLimit(float saturation){
	this->upperSaturation = saturation;
}

float RNPIDController::getTarget(void){
	return this->sp;
}

float RNPIDController::getError(void){
	return this->ek;
}

void RNPIDController::setTarget(float target){
	this->sp = target;
}

float RNPIDController::getLastInput(void){
	return this->uk_1;
}
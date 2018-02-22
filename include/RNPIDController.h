#ifndef RN_PID_CONTROLLER_H
#define RN_PID_CONTROLLER_H

#include "RNUtils.h"

class RNPIDController{
public:
	RNPIDController(const char* name = "", float sp = 0, float samplingTime = 100, float kp = 1, float ti = 0, float td = 0, float comp = 1, float lowerSaturation = -std::numeric_limits<float>::infinity(), float upperSaturation = std::numeric_limits<float>::infinity());
	~RNPIDController();

	int getSystemInput(float measure, float* output);

	float getProportionalGain(void);
	void setProportionalGain(float kp);

	float getIntegrationTime(void);
	void setIntegrationTime(float ti);

	float getDerivativeTime(void);
	void setDerivativeTime(float td);

	float getSamplingTime(void);
	void setSamplingTime(float time);

	float getDerivativeFilter(void);
	void setDerivativeFilter(float comp);

	float getLowerSaturationLimit(void);
	void setLowerSaturationLimit(float saturation);

	float getUpperSaturationLimit(void);
	void setUpperSaturationLimit(float saturation);

	float getError(void);

	float getTarget(void);
	void setTarget(float target);

	float getLastInput(void);
	void reset(void);

private:
	std::string name;
	int iteration;
	float kp;
	float ti;
	float td;
	float comp;

	float lowerSaturation;
	float upperSaturation;

	bool firstIteration;

	float sp;

	float ts;
	float uk_1;
	float ek_1;
	float ek_2;
	float ek;

};

#endif
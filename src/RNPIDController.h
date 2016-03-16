#ifndef RN_PID_CONTROLLER_H
#define RN_PID_CONTROLLER_H

#include <string.h>

class RNPIDController{
public:
	RNPIDController(const char* name = "", double setPoint = 0, double samplingTime = 100, double kp = 1, double ti = 0, double td = 0);
	~RNPIDController();

	double getSystemInput(int measure);

	double getProportionalGain(void);
	void setProportionalGain(double kp);

	double getIntegrationTime(void);
	void setIntegrationTime(double ti);

	double getDerivativeTime(void);
	void setDerivativeTime(double td);

	double getSamplingTime(void);
	void setSamplingTime(double time);

	double getTarget(void);
	void setTarget(double target);

private:
	char* name;
	double kp;
	double ti;
	double td;

	double setPoint;

	double samplingTime;
	double lastInput;
	double lastError;
	double pastLastError;
	double currentError;
};

#endif
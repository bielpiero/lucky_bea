#ifndef RN_DISTANCE_TIMER_TASK_H
#define RN_DISTANCE_TIMER_TASK_H

#include "RNRecurrentTask.h"

#define DEFAULT_SECURITY_DISTANCE_WARNING_TIME 60
#define DEFAULT_SECURITY_DISTANCE_STOP_TIME 90

class RNDistanceTimerTask : public RNRecurrentTask{
public:
	RNDistanceTimerTask(const RobotNode* gn, const char* name = "Distance Timer Task", const char* description = "");
	~RNDistanceTimerTask();
protected:
	virtual void task();
	virtual void onKilled();

private:
	unsigned int timerSecs;
	static const unsigned int SECURITY_DISTANCE_WARNING_TIME;
    static const unsigned int SECURITY_DISTANCE_STOP_TIME;
	//std::vector<Trio<int, int, int> > *history;

};

#endif
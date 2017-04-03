#ifndef RN_GESTURES_TASK_H
#define RN_GESTURES_TASK_H

#include "RNRecurrentTask.h"

/*struct s_motor{
	std::string idMotor;
    std::string cardId;
	std::string pos;
	std::string speed;
	std::string acceleration;
};*/

class RNGesturesTask : public RNRecurrentTask{
public:
	RNGesturesTask(const char* name = "Gestures Task", const char* description = "Doris Faces");
	~RNGesturesTask();
	virtual void task();
	virtual void onKilled();
};

#endif
#ifndef RN_RECURRENT_TASK_H
#define RN_RECURRENT_TASK_H

#include "RNAsyncTask.h"
#include "RobotNode.h"

class RNRecurrentTask : public RNAsyncTask{
public:
	RNRecurrentTask(const RobotNode* rn = NULL, const char* name = "", const char* description = "");
	virtual ~RNRecurrentTask();
	virtual void task() = 0;

	virtual void onKilled() = 0;
	
    void go();
    int done();
    void reset();
    void kill();
    
    void* runThread(void* object);
	void setController(RobotNode* rn);
	std::string getTaskName();
private:
	void waitUtilTaskFinished();
protected:
	RobotNode* rn;
private:
	std::string name;
	std::string description;
	bool running;
	bool goRequested;
	bool killed;
};

#endif
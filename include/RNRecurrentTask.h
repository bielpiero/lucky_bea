#ifndef RN_RECURRENT_TASK_H
#define RN_RECURRENT_TASK_H

#include "RNAsyncTask.h"
#include "GeneralController.h"

class RNRecurrentTask : public RNAsyncTask{
public:
	RNRecurrentTask(const char* name = "", const char* description = "");
	virtual ~RNRecurrentTask();
	virtual void task() = 0;

	virtual void onKilled() = 0;
	
    void go();
    int done();
    void reset();
    void kill();
    
    void* runThread(void* object);
	void setController(GeneralController* gn);
	std::string getTaskName();
private:
	void waitUtilTaskFinished();
protected:
	GeneralController* gn;
private:
	std::string name;
	std::string description;
	bool running;
	bool goRequested;
	bool killed;
};

#endif
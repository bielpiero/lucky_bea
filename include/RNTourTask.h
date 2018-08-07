#ifndef RN_TOUR_TASK_H
#define RN_TOUR_TASK_H

#include "RNRecurrentTask.h"
#include "GeneralController.h"
#include "RNGraph.h"
#include "RNTourDialog.h"

class RNTourTask : public RNRecurrentTask{

public:
	RNTourTask(const GeneralController* gn, const char* name = "Tour Task", const char* description = "Doris tour task");
	~RNTourTask();
	virtual void task();
	virtual void onKilled();
private:
	void init();
	int createCurrentMapGraph();
private:
	GeneralController* gn;
	RNGraph* currentMapGraph;
	RNTourDialog* tourSpeech;
	
	int lastSiteVisitedIndex;
	bool initialized;
	std::string xmlSectorsPath;
	std::vector<std::string> sequence;
};

#endif
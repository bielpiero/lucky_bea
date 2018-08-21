#ifndef RN_TOUR_TASK_H
#define RN_TOUR_TASK_H

#include "RNAsyncTask.h"
#include "GeneralController.h"
#include "RNGraph.h"

class RNTourThread : public RNAsyncTask{

public:
	RNTourThread(const GeneralController* gn, const char* name = "Tour Thread", const char* description = "Doris tour thread");
	~RNTourThread();
	int createCurrentMapGraph();
	void go();   
	void loadProgram(std::string filename);
private:
	void* runThread(void* object);
	void task();

	void lex();
	void parse();
	void parse(std::list<std::string> functionTokens, std::map<std::string, std::string> *functionSymbols);
	void printList(std::list<std::string> l);
	void printMap(std::map<std::string, std::string> m);
	void loadPredifinedSymbols();
	std::map<std::string, std::string> createOptionsMap(std::string opts);
	void processOptions(std::map<std::string, std::string> opts);
private:
	GeneralController* gn;
	std::string name;
	std::string description;
	bool executingTask;
	bool goRequested;
	bool killed;
	bool programLoaded;
	RNGraph* currentMapGraph;
	
	int lastSiteVisitedIndex;

	std::string xmlSectorsPath;
	std::vector<std::string> sequence;
	struct wcontent_t{
		std::map<std::string, std::string> symbols;
		std::list<std::string> tokens;
	};
	std::map<std::string, wcontent_t > functions;
	std::map<std::string, std::string> globalSymbols;

	std::ifstream file;
	DorisLipSync* lips;
};

#endif
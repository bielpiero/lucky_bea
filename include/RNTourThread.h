#ifndef RN_TOUR_TASK_H
#define RN_TOUR_TASK_H

#include "RNAsyncTask.h"
#include "GeneralController.h"
#include "RNGraph.h"

#define CMD_MOVE_BEGIN 0
#define CMD_MOVE_NEXT 1
#define CMD_MOVE_PREVIOUS 2
#define CMD_MOVE_END 3

class RNTourThread : public RNAsyncTask{

public:
	RNTourThread(const GeneralController* gn, const char* name = "Tour Thread", const char* description = "Doris tour thread");
	~RNTourThread();
	int createCurrentMapGraph();
	int createCurrentSectorGraph();
	void go();   
	void loadProgram(std::string filename);
private:
	GeneralController* gn;
	std::string name;
	std::string description;
	bool executingTask;
	bool goRequested;
	bool killed;
	bool programLoaded;
	RNGraph* currentMapGraph;
	RNGraph* currentSectorGraph;
	
	int lastSiteVisitedIndex;

	std::string xmlSectorsPath;
	std::vector<std::string> sequence;
	struct wcontent_t{
		std::map<std::string, std::string> symbols;
		std::list<std::string> tokens;
		std::string result;
	};
	std::map<std::string, wcontent_t > functions;
	std::map<std::string, std::string> globalSymbols;

	std::ifstream file;
	DorisLipSync* lips;
private:
	void* runThread(void* object);
	void task();

	void tripTo(int dst_sector, double dst_x, double dst_y);
	void shortTravel(int origin, int destiny);
	void longTravel(int origin, int destiny);
	void moveAround(int direction);
	int closestNodeTo(const ArPose& pose);
	void lex();
	void parse();
	void parse(std::string functionName, wcontent_t* content);
	void loadPredifinedSymbols();
	std::string evaluateExpression(std::string condition, std::map<std::string, std::string> symbols);
	std::map<std::string, std::string> createOptionsMap(std::string opts);
	void processOptions(std::map<std::string, std::string> opts);
	std::list<std::string> tokenizeExpCond(std::string expr_cond);
	void solveExpParenthesis(std::list<std::string>* tokens);
	void solveExp(std::list<std::string>* tokens);

};

#endif
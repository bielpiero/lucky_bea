#ifndef RN_TOUR_TASK_H
#define RN_TOUR_TASK_H

#include "definitions.h"
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
	int createCurrentSectorGraph(bool update = false);
	void go();
	virtual void kill();
	void loadProgram(std::string filename);
	void rfidTagsEvent(std::list<std::string> tags);
	void sectorUpdatedEvent(int sector);
private:
	GeneralController* gn;
	std::string name;
	std::string description;
	bool executingTask;
	bool goRequested;
	bool killed;
	bool programLoaded;
	MapSector* currentSector;
	RNGraph* currentMapGraph;
	RNGraph* currentSectorGraph;
	
	int lastSiteVisitedIndex;

	std::string xmlSectorsPath;
	std::vector<std::string> sequence;
	struct wcontent_t{
		std::map<std::string, std::string> symbols;
		std::list<std::string> arguments;
		std::list<std::string> tokens;
		int requiredArguments;
		std::string result;
	};
	struct wevent_t{
		int argCount;
		std::string eventFunction;
		wevent_t(int _argCount = 0){
			argCount = _argCount;
		}

	};
	std::map<std::string, wcontent_t > functions;
	std::map<std::string, std::string> globalSymbols;

	std::map<std::string, wevent_t> events;

	std::ifstream file;
	DorisLipSync* lips;
	RNRFIdentificationTask* rfid;
	RNEmotionsTask* emotions;

	std::list<int> currentSectorPathPlan;
	std::list<int> currentMapPathPlan;

	RNFunPointer1C<RNTourThread, std::list<std::string> >* rfidEvent;
	RNFunPointer1C<RNTourThread, int >* sectorChangedEvent;
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

	/** New
	  */
	void processGlobalVariables(std::list<std::string> tokens);
	std::string arrayValue(std::map<std::string, std::string> simbolos, std::string nombre, std::string position);
	std::string assignArray(std::map<std::string, std::string> simbolo, std::string array, std::string pos, std::string exprs);
	std::string assignSize(std::string word_size);

	void factor(std::list<std::string>* tokens);
	void term(std::list<std::string>* tokens);
	void simpExpr(std::list<std::string>* tokens);

	std::list<int> getSectorPathPlan();
	std::list<int> getMapPathPlan();

};

#endif
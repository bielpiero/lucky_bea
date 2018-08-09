#ifndef RN_TOUR_DIALOG_H
#define RN_TOUR_DIALOG_H

#include "RNUtils.h"
#include "DorisLipSync.h"
#include "GeneralController.h"

class RNTourDialog{
	public:
		RNTourDialog(const GeneralController* gn);
		virtual ~RNTourDialog();
		void lex();
		void parse();
		void parse(std::list<std::string> functionTokens, std::map<std::string, std::string> *functionSymbols);
	private:
		void printList(std::list<std::string> l);
		void printMap(std::map<std::string, std::string> m);
		void loadPredifinedSymbols();
		std::map<std::string, std::string> createOptionsMap(std::string opts);
		void processOptions(std::map<std::string, std::string> opts);
	private:
		struct wcontent_t{
			std::map<std::string, std::string> symbols;
			std::list<std::string> tokens;
		};
		std::map<std::string, wcontent_t > functions;
		std::map<std::string, std::string> globalSymbols;

		std::ifstream file;
		DorisLipSync* lips;
		GeneralController* gn;
};

#endif
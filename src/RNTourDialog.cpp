#include "RNTourDialog.h"

RNTourDialog::RNTourDialog(const GeneralController* gn){
	std::setlocale(LC_ALL, "es_ES");
	this->gn = (GeneralController*)gn;
	this->lips = this->gn->getTTS();
	file.open("tts/disam-01.text");
	if(!file){
		printf("could not load file\n");
	} else {
		loadPredifinedSymbols();	
		lex();
		parse();
	}
}

RNTourDialog::~RNTourDialog(){
	if(file){
		file.close();
	}
}

void RNTourDialog::loadPredifinedSymbols(){
	globalSymbols.emplace("FACE:happy", "ID:0");
	globalSymbols.emplace("FACE:surprise", "ID:1");
	globalSymbols.emplace("FACE:singing", "ID:2");
	globalSymbols.emplace("FACE:despair", "ID:3");
	globalSymbols.emplace("FACE:rejection", "ID:4");
	globalSymbols.emplace("FACE:slyly", "ID:5");
	globalSymbols.emplace("FACE:tired", "ID:6");
	globalSymbols.emplace("FACE:sleepy", "ID:7");
	globalSymbols.emplace("FACE:angry", "ID:8");
	globalSymbols.emplace("FACE:happy-dream", "ID:9");
	globalSymbols.emplace("FACE:shouting", "ID:10");
	globalSymbols.emplace("FACE:talking", "ID:11");
	globalSymbols.emplace("FACE:hypnotic", "ID:12");
	globalSymbols.emplace("FACE:evil", "ID:13");
	globalSymbols.emplace("FACE:afraid", "ID:14");
	globalSymbols.emplace("FACE:suspect", "ID:15");
	globalSymbols.emplace("FACE:neutral", "ID:16");
	globalSymbols.emplace("FACE:wink1", "ID:17");
	globalSymbols.emplace("FACE:wink2", "ID:18");
	globalSymbols.emplace("FACE:wink3", "ID:19");
	globalSymbols.emplace("FACE:shame", "ID:20");
	globalSymbols.emplace("FACE:thinker", "ID:21");
	globalSymbols.emplace("FACE:drunk", "ID:22");
	globalSymbols.emplace("FACE:worried", "ID:23");
	globalSymbols.emplace("FACE:pouts", "ID:24");
	globalSymbols.emplace("FACE:sidelook", "ID:25");
	globalSymbols.emplace("FACE:sympathy", "ID:26");
	globalSymbols.emplace("FACE:med-smile", "ID:27");
	globalSymbols.emplace("FACE:big-smile", "ID:28");
	globalSymbols.emplace("FACE:creepy-smile", "ID:29");
	globalSymbols.emplace("FACE:panic", "ID:30");
	globalSymbols.emplace("FACE:quiet", "ID:31");
	globalSymbols.emplace("FACE:sad", "ID:32");
	globalSymbols.emplace("ATTN:front", "ID:33");
	globalSymbols.emplace("ATTN:right", "ID:34");
	globalSymbols.emplace("ATTN:left", "ID:35");
}

void RNTourDialog::lex(){
	
	std::stringstream wss;
	wss << file.rdbuf();
	std::string buff = wss.str();
	std::list<char> fileContent(buff.begin(), buff.end());
	std::list<char>::iterator it;
	std::list<std::string> tokens;
	std::string tok = "";
	std::string str = "";
	std::string var = "";
	tokens.clear();
	int state = 0, isnumber = 0, varstarted = 0;
	int functionStarted = 0;
	int callfunction = 0;
	int equalfound = 0;
	std::string functionName = "";
	int functionNameStatus = 0;
	for(it = fileContent.begin(); it != fileContent.end(); it++){
		tok += *it;
		if(*it == ' '){
			if(functionStarted == 1 and functionNameStatus == 1 and str != ""){
				functionNameStatus = 0;
				functionName = str;
				functions.emplace(functionName, wcontent_t());
				str = "";
				tok = "";
			} else if(varstarted == 1 and var != ""){
				tokens.emplace_back("VAR:" + var);
				varstarted = 0;
				var = "";
				tok = "";
			} else if (callfunction == 1 and str != ""){
				tokens.emplace_back("FNC:" + str);
				callfunction = 0;
				str = "";
				tok = "";
			} else if(state == 2 and isnumber == 1){
				if(str != ""){
					tokens.emplace_back("NUM:" + str);
				}
				isnumber = 0;
				str = "";
				state = 0;
				tok = "";
			} else if(state == 0 or state == 1){
				tok = "";
			} else if (state == 2){
				tok = " ";
			}
		} else if(*it == '\n' or *it == '\t'){
			if(functionStarted == 1 and functionNameStatus == 1 and str != ""){
				functionName = str;
				functionNameStatus = 0;
				functions.emplace(functionName, wcontent_t());
				str = "";
			} else if(varstarted == 1 and var != ""){
				tokens.emplace_back("VAR:" + var);
				varstarted = 0;
				var = "";
				tok = "";
			} else if (callfunction == 1 and str != ""){
				tokens.emplace_back("FNC:" + str);
				callfunction = 0;
				str = "";
			} else if(state == 2 and isnumber == 1){
				if(str != ""){
					tokens.emplace_back("NUM:" + str);
				}
				isnumber = 0;
				str = "";
				state = 0;
				tok = "";
			}
			tok = "";
		} else if(tok == "function"){
			if(functionStarted == 0){
				functionStarted = 1;
				functionName = "";
				functionNameStatus = 1;
				tokens.clear();
			} else {
				functionStarted = -1;
			}
			tok = "";
		} else if(tok == "endfunction"){
			if(functionStarted == 1){
				functionStarted = 0;
				std::cout << functionName << std::endl;
				printList(tokens);
				wcontent_t currentContent = functions.at(functionName);
				currentContent.tokens = tokens;
				functions[functionName] = currentContent;
				tokens.clear();
				
			} else {
				functionStarted = -1;
			}
			tok = "";
		} else if(tok == "call"){
			tokens.emplace_back("CALL");
			callfunction = 1;
			tok = "";
		} else if(tok == "move"){
			tokens.emplace_back("MOVE");
			tok = "";
		} else if(tok == "goto"){
			tokens.emplace_back("GOTO");
			tok = "";
		} else if(tok == "say"){
			tokens.emplace_back("SAY");
			tok = "";
		} else if(tok == "turn"){
			tokens.emplace_back("TURN");
			tok = "";
		} else if(tok == "attention"){
			tokens.emplace_back("ATTENTION");
			tok = "";
		} else if(tok == "var"){
			varstarted = 1;
			var = "";
			tok = "";
		} else if (tok == "(" or tok == "["){
			state = 1;
			tok = "";
		} else if(tok == "=" and state == 0){
			state = 1;
			tok = "";
		} else if (tok == "\""){
			if(state == 1){
				state = 2;
				str += tok;
			} else if (state == 2){
				if(str != "" and str.substr(0, 1) == "\""){
					tokens.emplace_back("STR:" + str.substr(1, str.length() - 1));
				}
				str = "";
				state = 0;
			}
			tok = "";
		} else if(tok == "#"){ 
			if(state == 1){
				state = 2;
				isnumber = 1;
			}
			tok = "";
		} else if (tok == ","){
			if(state == 1){
				if(str != ""){
					tokens.emplace_back("VAR:" + str);
					str = "";
					tok = "";
				}
			} else if(state == 2){
				if(isnumber == 1){
					tokens.emplace_back("NUM:" + str);
					isnumber = 0;
					state = 1;
					str = "";
					tok = "";
				} else {
					str += tok;
					tok = "";
				}

			}
		} else if (tok == ")"){
			if(state == 1){
				if(str != ""){
					tokens.emplace_back("VAR:" + str);
				}
			} else if(state == 2){
				if(isnumber == 1){
					tokens.emplace_back("NUM:" + str);
					isnumber = 0;
				}
			}
			str = "";
			tok = "";
			state = 0;
			
		} else if(tok == "]"){
			if(state == 1){
				tokens.emplace_back("OPT:" + str);
				state = 0;
				tok = "";
				str = "";
			}
		} else if(functionStarted == 1 and functionNameStatus == 1){
			str += tok;
			tok = "";
		} else if(state > 0){
			if(tok == "#"){
				tok = "";
				if(isnumber == 0){
					isnumber = 1;
				}
			} else {
				str += tok;
				tok = "";
			}
		} else if (varstarted == 1){
			var += tok;
			tok = "";
		} else if(callfunction == 1){
			str += tok;
			tok = "";
		}
	}
}

void RNTourDialog::printList(std::list<std::string> l){
	std::list<std::string>::iterator it;
	std::cout << "[";
	for(it = l.begin(); it != l.end(); it++){
		std::cout << *it;
		if(std::next(it, 1) != l.end()){
			std::cout << ", ";
		}
	}
	std::cout << "]" << std::endl;
}

void RNTourDialog::printMap(std::map<std::string, std::string> m){
	std::map<std::string, std::string>::iterator it;
	std::cout << "{";
	for(it = m.begin(); it != m.end(); it++){
		std::cout << it->first << ": " << it->second; 
		if(std::next(it, 1) != m.end()){
			std::cout << ", ";
		}
	}
	std::cout << "}" << std::endl;
}

void RNTourDialog::parse(){

	std::map<std::string, wcontent_t>::iterator fit_main;
	fit_main = functions.find("main");
	if(fit_main != functions.end()){
		std::map<std::string, std::string> symbols;
		parse(fit_main->second.tokens, &symbols);
		fit_main->second.symbols = symbols;
	} else {
		printf("FUNCTION MAIN NOT FOUND\n");
	}
}

void RNTourDialog::parse(std::list<std::string> functionTokens, std::map<std::string, std::string>* functionSymbols){
	//printList(functionTokens);
	std::list<std::string>::iterator it = functionTokens.begin();
	while(it != functionTokens.end()){
		std::list<std::string>::iterator it2 = std::next(it, 1);
		std::list<std::string>::iterator it3 = std::next(it, 2);
		std::list<std::string>::iterator it4 = std::next(it, 3);
		if((*it) == "MOVE"){
			if(it2 != functionTokens.end()){
				if((*it2).substr(0, 3) == "STR"){
					printf("MOVE TO COMMAND\n");
					it = std::next(it, 2);
				} else if((*it2).substr(0, 3) == "VAR"){
					if(functionSymbols->find((*it2).substr(4)) != functionSymbols->end()){
						printf("MOVE VAR: %s\n", functionSymbols->at((*it2).substr(4)).c_str());
						//lips->textToViseme((*it2).substr(4));
					} else {
						fprintf(stderr, "Unidefined VAR: %s\n", (*it2).substr(4).c_str());
					}
					it = std::next(it, 2);
				}
			}
		} else if((*it) == "GOTO"){
			int sector;
			float px, py;
			if((*it2).substr(0, 3) == "NUM" and (*it3).substr(0, 3) == "NUM" and (*it4).substr(0, 3) == "NUM"){
				sector = std::stoi((*it2).substr(4));
				px = std::stof((*it3).substr(4));
				py = std::stof((*it4).substr(4));
			} else if((*it2).substr(0, 3) == "NUM" and (*it3).substr(0, 3) == "NUM" and (*it4).substr(0, 3) == "VAR"){
				sector = std::stoi((*it2).substr(4));
				px = std::stof((*it3).substr(4));
				if(functionSymbols->find((*it4).substr(4)) != functionSymbols->end()){
					py = std::stof(functionSymbols->at((*it4).substr(4)).substr(4));
				}
			} else if((*it2).substr(0, 3) == "NUM" and (*it3).substr(0, 3) == "VAR" and (*it4).substr(0, 3) == "NUM"){
				sector = std::stoi((*it2).substr(4));
				if(functionSymbols->find((*it3).substr(4)) != functionSymbols->end()){
					px = std::stof(functionSymbols->at((*it3).substr(4)).substr(4));
				}
				py = std::stof((*it4).substr(4));
			} else if((*it2).substr(0, 3) == "NUM" and (*it3).substr(0, 3) == "VAR" and (*it4).substr(0, 3) == "VAR"){
				sector = std::stoi((*it2).substr(4));
				if(functionSymbols->find((*it3).substr(4)) != functionSymbols->end()){
					px = std::stof(functionSymbols->at((*it3).substr(4)).substr(4));
				}
				if(functionSymbols->find((*it4).substr(4)) != functionSymbols->end()){
					py = std::stof(functionSymbols->at((*it4).substr(4)).substr(4));
				}
			} else if((*it2).substr(0, 3) == "VAR" and (*it3).substr(0, 3) == "NUM" and (*it4).substr(0, 3) == "NUM"){
				if(functionSymbols->find((*it2).substr(4)) != functionSymbols->end()){
					sector = std::stoi(functionSymbols->at((*it2).substr(4)).substr(4));
				}
				px = std::stof((*it3).substr(4));
				py = std::stof((*it4).substr(4));
			} else if((*it2).substr(0, 3) == "VAR" and (*it3).substr(0, 3) == "NUM" and (*it4).substr(0, 3) == "VAR"){
				if(functionSymbols->find((*it2).substr(4)) != functionSymbols->end()){
					sector = std::stoi(functionSymbols->at((*it2).substr(4)).substr(4));
				}
				px = std::stof((*it3).substr(4));
				if(functionSymbols->find((*it4).substr(4)) != functionSymbols->end()){
					py = std::stof(functionSymbols->at((*it4).substr(4)).substr(4));
				}
			} else if((*it2).substr(0, 3) == "VAR" and (*it3).substr(0, 3) == "VAR" and (*it4).substr(0, 3) == "NUM"){
				if(functionSymbols->find((*it2).substr(4)) != functionSymbols->end()){
					sector = std::stoi(functionSymbols->at((*it2).substr(4)).substr(4));
				}
				if(functionSymbols->find((*it3).substr(4)) != functionSymbols->end()){
					px = std::stof(functionSymbols->at((*it3).substr(4)).substr(4));
				}
				py = std::stof((*it4).substr(4));
			} else if((*it2).substr(0, 3) == "VAR" and (*it3).substr(0, 3) == "VAR" and (*it4).substr(0, 3) == "VAR"){
				if(functionSymbols->find((*it2).substr(4)) != functionSymbols->end()){
					sector = std::stoi(functionSymbols->at((*it2).substr(4)).substr(4));
				}
				if(functionSymbols->find((*it3).substr(4)) != functionSymbols->end()){
					px = std::stof(functionSymbols->at((*it3).substr(4)).substr(4));
				}
				if(functionSymbols->find((*it4).substr(4)) != functionSymbols->end()){
					py = std::stof(functionSymbols->at((*it4).substr(4)).substr(4));
				}
			}
			printf("Going to Sector: %d, {x: %f, y: %f}\n", sector, px, py);
			it = std::next(it, 4);
		} else if((*it) == "TURN"){
			if(it2 != functionTokens.end()){
				if((*it2).substr(0, 3) == "NUM"){
					printf("TURN %s DEGREES COMMAND\n", (*it2).substr(4).c_str());
					it = std::next(it, 2);
				}
			} 
		} else if((*it) == "CALL"){
			if(it2 != functionTokens.end() and (*it2).substr(0, 3) == "FNC"){
				std::string functionName = (*it2).substr(4);
				std::map<std::string, wcontent_t>::iterator fit_call;
				fit_call = functions.find(functionName);
				if(fit_call != functions.end()){
					std::map<std::string, std::string> symbols;
					parse(fit_call->second.tokens, &symbols);
					fit_call->second.symbols = symbols;
				} else {
					printf("FUNCTION %s NOT FOUND\n", functionName.c_str());
				}
				it = std::next(it, 2);
			}
			
		} else if((*it) == "SAY"){
			if(it2 != functionTokens.end()){
				if((*it2).substr(0, 3) == "STR"){
					//lips->textToViseme((*it2).substr(4));
					it = std::next(it, 2);
				} else if((*it2).substr(0, 3) == "VAR"){
					if(functionSymbols->find((*it2).substr(4)) != functionSymbols->end()){
						printf("SAY VAR: %s\n", functionSymbols->at((*it2).substr(4)).c_str());
						//lips->textToViseme((*it2).substr(4));
					} else {
						fprintf(stderr, "Unidefined VAR: %s\n", (*it2).substr(4).c_str());
					}
					it = std::next(it, 2);
				} else if((*it2).substr(0, 3) == "OPT"){
					if(it3 != functionTokens.end() and (*it3).substr(0, 3) == "STR"){
						printf("SAY STRING %s WITH OPTIONS: %s\n", (*it3).substr(4).c_str(), (*it2).substr(4).c_str());
						std::map<std::string, std::string> opts = createOptionsMap((*it2).substr(4));
						processOptions(opts);
						//lips->textToViseme((*it3).substr(4));
						it = std::next(it, 3);
					}
				}
			} 
		} else if((*it).substr(0, 3) == "VAR"){
			if(it2 != functionTokens.end()){
				if((*it2).substr(0, 3) == "STR" or (*it2).substr(0, 3) == "NUM"){
					if(functionSymbols->find((*it).substr(4)) == functionSymbols->end()){
						functionSymbols->emplace((*it).substr(4), (*it2));
					} else {
						functionSymbols->at((*it).substr(4)) = (*it2);
					}
					it = std::next(it, 2);
				} else if((*it2).substr(0, 3) == "VAR"){
					if(functionSymbols->find((*it2).substr(4)) != functionSymbols->end()){
						if(functionSymbols->find((*it).substr(4)) == functionSymbols->end()){
							functionSymbols->emplace((*it).substr(4), functionSymbols->at((*it2).substr(4)));
						} else {
							functionSymbols->at((*it).substr(4)) = functionSymbols->at((*it2).substr(4));
						}
					} else {
						fprintf(stderr, "Unidefined VAR: %s\n", (*it2).substr(4).c_str());
					}
					it = std::next(it, 2);
				} else {
					if(functionSymbols->find((*it).substr(4)) == functionSymbols->end()){
						functionSymbols->emplace((*it).substr(4), "UNK:nil");
					} else {
						fprintf(stderr, "VAR: %s, already exists\n", (*it).substr(4).c_str());
					}
					it++;
				}
			}
		} else {
			it++;
		}
	}
}

std::map<std::string, std::string> RNTourDialog::createOptionsMap(std::string opts){
	std::map<std::string, std::string> dic;
	std::vector<std::string> sopts = RNUtils::split(opts, ";");
	for(int i = 0; i < sopts.size(); i++){
		std::vector<std::string> opt_value = RNUtils::split(sopts.at(i), "=");
		if(opt_value.size() == 2){
			if(dic.find(opt_value.at(0)) == dic.end()){
				dic.emplace(opt_value.at(0), opt_value.at(1));
			} else {
				dic[opt_value.at(0)] = opt_value.at(1);
			}
		}
	}
	return dic;
}

void RNTourDialog::processOptions(std::map<std::string, std::string> opts){
	std::map<std::string, std::string>::iterator optsIt;
	std::map<std::string, std::string>::iterator op;
	for(optsIt = opts.begin(); optsIt != opts.end(); optsIt++){
		if(optsIt->first == "face"){
			op = globalSymbols.find("FACE:" + optsIt->second);
			if(op != opts.end()){
				//gn->setEmotionsResult("", op->second.substr(3));
			}
		} else if(optsIt->first == "attention"){
			op = globalSymbols.find("ATTN:" + optsIt->second);
			if(op != opts.end()){
				//gn->setEmotionsResult("", op->second.substr(3));
			}
		}
		RNUtils::sleep(100);
	}
}
#include "RNTourThread.h"

RNTourThread::RNTourThread(const GeneralController* gn, const char* name, const char* description) {
	std::setlocale(LC_ALL, "es_ES");
	this->name = "RNThread: " + std::string(name);
    this->description = std::string(description);
    setThreadName(this->name.c_str());

	this->gn = (GeneralController*)gn;
	this->lips = this->gn->getTTS();

	executingTask = goRequested = killed = false;

	xmlSectorsPath = RNUtils::getApplicationPath() + XML_FILE_SECTORS_PATH;
	currentMapGraph = NULL;
	currentSectorGraph = NULL;
	lastSiteVisitedIndex = RN_NONE;
	programLoaded = false;
	loadPredifinedSymbols();
}

RNTourThread::~RNTourThread(){
	if(file){
		file.close();
	}
}

void RNTourThread::go(){
    lock();
    goRequested = true;
    executingTask = true;
    killed = false;
    unlock();

    create();
}

void* RNTourThread::runThread(void* object){
    threadStarted();
    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
    if(RNUtils::ok() and isRunning){
        testCancel(); 
        bool doit;
        lock();
        doit = goRequested;
        unlock();
        if(doit){
            lock();
            executingTask = true;
            unlock();
            task();
            lock();
            executingTask = false;
            unlock();
        }
        RNUtils::sleep(20);
    }
    stop();
    threadFinished();
    return NULL;
}

void RNTourThread::task(){
	if(currentMapGraph == NULL){
		RNUtils::printLn("error: Sector not loaded...");
	} else if(not programLoaded){
		RNUtils::printLn("error: Program not loaded...");
	} else {
		parse();
	}
}

void RNTourThread::loadProgram(std::string filename){
	programLoaded = false;
	file.open(filename);
	if(!file){
		RNUtils::printLn("could not load file: %s", filename.c_str());
	} else {
		lex();
		programLoaded = true;
		RNUtils::printLn("Loaded program file: %s. waiting for user to start..", filename.c_str());
	}
}

int RNTourThread::createCurrentMapGraph(){
	int res = RN_NONE;
	if(gn->getCurrentSector()){
		if(currentMapGraph){
			delete currentMapGraph;
		}
		currentMapGraph = new RNGraph;
		std::string filename;

		gn->getMapFilename(gn->getCurrentSector()->getMapId(), filename);

		xml_document<> doc;
		xml_node<>* root_node;  

		std::string fullSectorPath = xmlSectorsPath + filename;
		std::ifstream the_file(fullSectorPath.c_str());
		std::vector<char> buffer = std::vector<char>((std::istreambuf_iterator<char>(the_file)), std::istreambuf_iterator<char>());
		buffer.push_back('\0');
		doc.parse<0>(&buffer[0]);
		root_node = doc.first_node(XML_ELEMENT_SECTORS_STR);
		if(root_node != NULL){
			for (xml_node<> * sector_node = root_node->first_node(XML_ELEMENT_SECTOR_STR); sector_node; sector_node = sector_node->next_sibling()){
				int xmlSectorId = std::atoi(sector_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value());
				currentMapGraph->addNode(xmlSectorId);
			}
			root_node = doc.first_node(XML_ELEMENT_SECTORS_STR);
			for (xml_node<> * sector_node = root_node->first_node(XML_ELEMENT_SECTOR_STR); sector_node; sector_node = sector_node->next_sibling()){
				int xmlSectorId = std::atoi(sector_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value());
				std::string ady = std::string(sector_node->first_attribute(XML_ATTRIBUTE_ADJACENCY_STR)->value());
				std::vector<std::string> adys = RNUtils::split(ady, ",");
				for(int i = 0; i < adys.size(); i++){
					int sectorAdy = std::atoi(adys.at(i).c_str());
					if(currentMapGraph->addEdge(xmlSectorId, sectorAdy) != RN_OK){
						RNUtils::printLn("Could not add edge between sector %d and sector %d", xmlSectorId, sectorAdy);
					}
					if(currentMapGraph->addEdge(sectorAdy, xmlSectorId) != RN_OK){
						RNUtils::printLn("Could not add edge between sector %d and sector %d", sectorAdy, xmlSectorId);
					}
				}
				
			}
		}
		the_file.close();
		res = RN_OK;
		RNUtils::printLn("Created Map Graph..");
	}
	return res;
}

int RNTourThread::createCurrentSectorGraph(){
	int res = RN_NONE;
	if(gn->getCurrentSector()){
		if(currentSectorGraph){
			delete currentSectorGraph;
		}
		currentSectorGraph = new RNGraph;
		
		for (int w = 0; w < gn->getCurrentSector()->waysSize(); w++){
			currentSectorGraph->addNode(gn->getCurrentSector()->wayAt(w)->st);
		}
		for (int w = 0; w < gn->getCurrentSector()->waysSize(); w++){
			int siteId = gn->getCurrentSector()->wayAt(w)->st;
			for(int i = 0; i < gn->getCurrentSector()->wayAt(w)->adjacencies.size(); i++){
				int siteAdy = gn->getCurrentSector()->wayAt(w)->adjacencies.at(i);
				if(currentSectorGraph->addEdge(siteId, siteAdy) != RN_OK){
					RNUtils::printLn("Could not add edge between site %d and site %d", siteId, siteAdy);
				}
				if(currentSectorGraph->addEdge(siteAdy, siteId) != RN_OK){
					RNUtils::printLn("Could not add edge between site %d and site %d", siteAdy, siteId);
				}
			}
		}
		res = RN_OK;
		RNUtils::printLn("Created Sector Graph..");
	}
	return res;
}

void RNTourThread::tripTo(int dst_sector, double dst_x, double dst_y){
	if(dst_sector != RN_NONE){
		longTravel(gn->getCurrentSector()->getId(), dst_sector);
	}
	ArPose* currPose = gn->getAltPose();
	ArPose dstPose(dst_x * 1e3, dst_y * 1e3, 0.0);
	int originSite = closestNodeTo(*currPose);
	printf("originSite: %d\n", originSite);
	int destinySite = closestNodeTo(dstPose);
	printf("destinySite: %d\n", destinySite);
	shortTravel(originSite, destinySite);
	RNUtils::printLn("Arrived Near, going to point desired");
	gn->moveRobotToPosition(dst_x, dst_y, 0.0);
    while((not gn->isGoalAchieved()) and (not gn->isGoalCanceled())) RNUtils::sleep(100);
}

void RNTourThread::moveAround(int direction){
	int originSite, destinySite;
	std::vector<std::string> sequence = RNUtils::split(gn->getCurrentSector()->getSequence(), ",");
	if(lastSiteVisitedIndex == RN_NONE){
		ArPose* currPose = gn->getAltPose();
		originSite = closestNodeTo(*currPose);
	} else {
		originSite = std::stoi(sequence.at(lastSiteVisitedIndex));
	}
	printf("originSite: %d\n", originSite);
	int actualSector = gn->getCurrentSector()->getId();
	if(direction == CMD_MOVE_BEGIN){
		destinySite = std::stoi(sequence.at(0));
	} else if(direction == CMD_MOVE_NEXT){
		destinySite = std::stoi(sequence.at(lastSiteVisitedIndex + 1));
	} else if(direction == CMD_MOVE_NEXT){
		destinySite = std::stoi(sequence.at(lastSiteVisitedIndex - 1));	
	} else if(direction == CMD_MOVE_NEXT){
		destinySite = std::stoi(sequence.at(sequence.size() - 1));
	}
	
	printf("destinySite: %d\n", destinySite);
	//shortTravel(originSite, destinySite);
	if(gn->isGoalAchieved() and actualSector == gn->getCurrentSector()->getId()){
		if(direction == CMD_MOVE_BEGIN){
			lastSiteVisitedIndex = std::stoi(sequence.at(0));
		} else if(direction == CMD_MOVE_NEXT){
			lastSiteVisitedIndex = std::stoi(sequence.at(lastSiteVisitedIndex + 1));
		} else if(direction == CMD_MOVE_NEXT){
			lastSiteVisitedIndex = std::stoi(sequence.at(lastSiteVisitedIndex - 1));	
		} else if(direction == CMD_MOVE_NEXT){
			lastSiteVisitedIndex = std::stoi(sequence.at(sequence.size() - 1));
		}
	}
}

void RNTourThread::longTravel(int origin, int destiny){
	if(currentMapGraph){
		std::list<int> path = currentMapGraph->branchAndBound(origin, destiny);
		printf("Long Travel in Map: %d\n", gn->getCurrentSector()->getMapId());
		RNUtils::printList<int>(path);
		std::list<int>::iterator pathIt;
		ArPose* currPose = NULL;
		for(pathIt = path.begin(); pathIt != path.end(); pathIt++){
			if(*pathIt != destiny){
				s_site* destinationSite = NULL;
				int originSite = RN_NONE, destinySite = RN_NONE;
				currPose = gn->getAltPose();
				originSite = closestNodeTo(*currPose);
				printf("originSite: %d\n", originSite);
				std::map<int, double> mds;
				for(int i = 0; i < gn->getCurrentSector()->sitesSize(); i++){
					s_site* node = gn->getCurrentSector()->siteAt(i);
					std::list<int>::iterator pathNextIt = std::next(pathIt, 1);
					if(pathNextIt != path.end()){
						if(node->linkedSectorId == *pathNextIt){
							destinySite = node->id;
						}
					}
				}
				shortTravel(originSite, destinySite);
				delete currPose;
				currPose = NULL;
				RNUtils::sleep(10);
			}
		}
	}
}

void RNTourThread::shortTravel(int origin, int destiny){
	if(currentSectorGraph){
		std::list<int> path = currentSectorGraph->branchAndBound(origin, destiny);
		printf("Short Travel in sector: %d\n", gn->getCurrentSector()->getId());
		RNUtils::printList<int>(path);
		std::list<int>::iterator pathIt;
		for(pathIt = path.begin(); pathIt != path.end(); pathIt++){
			s_site* destinationSite = gn->getCurrentSector()->findSiteById(*pathIt);
			RNUtils::sleep(100);
        	if(destinationSite != NULL){
        		bool changeSector = destinationSite->name == std::string(SEMANTIC_FEATURE_DOOR_STR);
        		gn->moveRobotToPosition(destinationSite->xpos, destinationSite->ypos, 0.0);
        		while((not gn->isGoalAchieved()) and (not gn->isGoalCanceled())) RNUtils::sleep(100);

        		if(changeSector){
        			
        			ArPose* currPose = gn->getAltPose();
        			gn->setPosition((currPose->getX() / 1e3) + destinationSite->xcoord, (currPose->getY() / 1e3) + destinationSite->ycoord, currPose->getThRad());
        			currPose = gn->getAltPose();
        			printf("{x: %f, y: %f, th: %f}\n", (currPose->getX() / 1e3), (currPose->getY() / 1e3), currPose->getThRad());
        			gn->loadSector(gn->getCurrentSector()->getMapId(), destinationSite->linkedSectorId);
        			lastSiteVisitedIndex = RN_NONE;
        		}

        	}
		}
	}
	
}

int RNTourThread::closestNodeTo(const ArPose& pose){
	int nodeId = RN_NONE;
	std::map<int, double> mds;
	printf("Pose: {x: %f, y: %f, th: %f}\n", (pose.getX() / 1e3), (pose.getY() / 1e3), pose.getThRad());
	for(int i = 0; i < gn->getCurrentSector()->sitesSize(); i++){
		s_site* node = gn->getCurrentSector()->siteAt(i);
		if(node->name != std::string(SEMANTIC_FEATURE_DOOR_STR)){
			mds.emplace(node->id, RNUtils::distanceTo(node->xpos, node->ypos, (pose.getX() / 1e3), (pose.getY() / 1e3)));
		}		
	}
	double mdist = std::numeric_limits<double>::max();
	std::map<int, double>::iterator mdsit;
	RNUtils::printMap<int, double>(mds);
	for(mdsit = mds.begin(); mdsit != mds.end(); mdsit++){
		if(mdist > mdsit->second){
			mdist = mdsit->second;
			nodeId = mdsit->first;
		}
	}
	return nodeId;
}

void RNTourThread::loadPredifinedSymbols(){
	globalSymbols.clear();
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
	globalSymbols.emplace("VAR:begin", "ID:0");
	globalSymbols.emplace("VAR:next", "ID:1");
	globalSymbols.emplace("VAR:previous", "ID:2");
	globalSymbols.emplace("VAR:end", "ID:3");
}

void RNTourThread::lex(){
	
	std::stringstream wss;
	wss << file.rdbuf();
	std::string buff = wss.str();
	std::list<char> fileContent(buff.begin(), buff.end());
	std::list<char>::iterator it;
	std::list<std::string> tokens;
	std::list<std::string>::iterator tpos = tokens.end();
	std::string tok = "";
	std::string str = "";
	std::string var = "";
	std::string cnd = "";
	std::string ret = "VOID";
	tokens.clear();
	unsigned long long ifcounter = 0, forcounter = 0, whilecounter = 0;
	int argCount = 0;
	int parcount = 0;
	int state = 0, isnumber = 0, varstarted = 0, retstarted = 0;
	int functionStarted = 0, functionarguments = 0, function_call = 0;
	int infunction = 0, ifstarted = 0, whilestarted = 0;
	int expr_started = 0, elsestate = 0, equal_started = 0;
	int equalfound = 0;
	std::string functionName = "";
	int functionNameStatus = 0;
	for(it = fileContent.begin(); it != fileContent.end(); it++){
		tok += *it;
		if(*it == ' '){
			tok = tok.substr(0, tok.size() - 1);
			if(functionStarted == 1 and functionNameStatus == 1 and str != ""){
				functionNameStatus = 0;
				functionName = str;
				functions.emplace(functionName, wcontent_t());
				str = "";
				tok = "";
			} else if(varstarted == 1 and var != ""){
				tokens.insert(tpos, "DVR:" + var);
				varstarted = 0;
				var = "";
				tok = "";
			} else if(state == 2 and isnumber == 1){
				if(str != ""){
					tokens.insert(tpos, "NUM:" + str);
				}
				isnumber = 0;
				str = "";
				state = 0;
				tok = "";
			} else if(tok != "" and expr_started == 0 and std::regex_match(tok, std::regex("[a-zA-Z_$][a-zA-Z_$0-9]*"))){
				if(functionarguments == 1){
					tokens.insert(tpos, "ARG:" + tok);
					argCount++;
				} else {
					tokens.insert(tpos, "VAR:" + tok);
				}
				
				tok = "";
			} else if(state == 1 and expr_started == 1){
				tok = " ";
			} else if(state == 0 or state == 1){
				tok = "";
			} else if (state == 2){
				tok = " ";
			} else if(retstarted == 1 and cnd == ""){
				tok = "";
			}
		} else if(*it == '\n' or *it == '\t'){
			tok = tok.substr(0, tok.size() - 1);
			if(functionStarted == 1 and functionNameStatus == 1 and str != ""){
				functionName = str;
				functionNameStatus = 0;
				functions.emplace(functionName, wcontent_t());
				str = "";
				tok = "";
			} else if(varstarted == 1 and var != ""){
				tokens.insert(tpos, "DVR:" + var);
				varstarted = 0;
				var = "";
				tok = "";
			} else if(state == 2 and isnumber == 1){
				if(str != ""){
					tokens.insert(tpos, "NUM:" + str);
				}
				isnumber = 0;
				str = "";
				state = 0;
				tok = "";
			} else if(tok != "" and expr_started == 0 and std::regex_match(tok, std::regex("[a-zA-Z_$][a-zA-Z_$0-9]*"))){
				if(functionarguments == 1){
					tokens.insert(tpos, "ARG:" + tok);
					argCount++;
				} else {
					tokens.insert(tpos, "VAR:" + tok);
				}
				tok = "";
			} else if(retstarted == 1 and expr_started == 1 and cnd != ""){
				tokens.insert(tpos, "EXP:" + cnd);
				expr_started = 0;
				retstarted = 0;
				state = 0;
				cnd = "";
				tok = "";
			} else if(equal_started == 1 and expr_started == 1 and cnd != ""){
				tokens.insert(tpos, "EXP:" + cnd);
				expr_started = 0;
				equal_started = 0;
				state = 0;
				cnd = "";
				tok = "";

			} else if(state == 1 and expr_started == 1){
				tok = " ";
			} else if(state == 0 or state == 1){
				tok = "";
			} else if (state == 2){
				tok = " ";
			} else if(retstarted == 1 and cnd == ""){
				tok = "";
			}
		} else if(tok == "function"){
			if(functionStarted == 0){
				functionStarted = 1;
				functionName = "";
				functionNameStatus = 1;
				functionarguments = 1;
				tokens.clear();
			} else {
				functionStarted = -1;
			}
			tok = "";
		} else if(tok == "endfunction"){
			if(functionStarted == 1){
				functionStarted = 0;
				std::cout << functionName << std::endl;
				RNUtils::printList<std::string>(tokens);
				wcontent_t currentContent = functions.at(functionName);
				currentContent.result = ret;
				currentContent.tokens = tokens;
				currentContent.requiredArguments = argCount;
				argCount = 0;
				functions[functionName] = currentContent;
				tokens.clear();
				ifcounter = 0;
				forcounter = 0; 
				whilecounter = 0;
			} else {
				functionStarted = -1;
			}
			tok = "";
		} else if(tok == "if"){
			tokens.insert(tpos, "IF");
			infunction = 1;
			expr_started = 1;
			ifstarted = 1;
			tok = "";
		} else if(tok == "else"){
			elsestate++;
			tpos = std::next(tpos, 1);
			tok = "";
		} else if(tok == "endif"){
			if(elsestate > 0){
				elsestate--;
				tpos = std::next(tpos, 1);
			} else {
				tpos = std::next(tpos, 2);
			}
			tok = "";
		} else if(tok == "while"){
			tokens.insert(tpos, "WHILE");
			infunction = 1;
			expr_started = 1;
			whilestarted = 1;
			tok = "";
		} else if(tok == "endw"){
			tpos = std::next(tpos, 1);
			tok = "";
		} else if(tok == "move"){
			tokens.insert(tpos, "MOVE");
			infunction = 1;
			tok = "";
		} else if(tok == "goto"){
			tokens.insert(tpos, "GOTO");
			infunction = 1;
			tok = "";
		} else if(tok == "say"){
			tokens.insert(tpos, "SAY");
			infunction = 1;
			tok = "";
		} else if(tok == "turn"){
			tokens.insert(tpos, "TURN");
			infunction = 1;
			tok = "";
		} else if(tok == "attention"){
			tokens.insert(tpos, "ATTENTION");
			infunction = 1;
			tok = "";
		} else if(tok == "var"){
			varstarted = 1;
			var = "";
			tok = "";
		} else if(tok == "return"){
			tokens.insert(tpos, "RETURN");
			retstarted = 1;
			expr_started = 1;
			infunction = 1;
			state = 1;
			tok = "";
		} else if(tok == "(" and state == 0 and functionStarted == 1 and functionNameStatus == 1 and str != ""){
			functionName = str;
			functionNameStatus = 0;
			functions.emplace(functionName, wcontent_t());
			state = 1;
			str = "";
			tok = "";
		} else if((*it) == '(' and state == 0 and tok != "" and infunction == 0){
			tok = tok.substr(0, tok.size() - 1);
			tokens.insert(tpos, "FNC:" + tok);
			tokens.insert(tpos, "EFC");
			tpos = std::prev(tpos, 1);
			function_call = 1;
			expr_started = 1;
			state = 1;
			str = "";
			tok = "";
		} else if (tok == "(" and state == 0 and infunction == 1){
			state = 1;
			tok = "";
		} else if (tok == "[" and state == 0 and infunction == 1){
			state = 1;
			tok = "";
		} else if(tok == "=" and state == 0){
			state = 1;
			expr_started = 1;
			equal_started = 1;
			tokens.insert(tpos, "EQ");
			tok = "";
		} else if (*it == '\"' and expr_started == 0){
			if(state == 1){
				state = 2;
				str += tok;
			} else if (state == 2){
				if(str != "" and str.substr(0, 1) == "\""){
					tokens.insert(tpos, "STR:" + str.substr(1, str.length() - 1));
				}
				str = "";
				state = 1;
			}
			tok = "";
		} else if(*it == '#' and expr_started == 0){ 
			if(state == 1){
				state = 2;
				isnumber = 1;
			}
			tok = "";
		} else if (tok == ","){
			if(function_call == 1 and expr_started == 1 and cnd != ""){
				printf("pillada expresion de llamada a funcion\n");
				tokens.insert(tpos, "EXP:" + cnd);
				cnd = "";
				tok = "";
			}
			if(state == 1){
				if(str != ""){
					if(functionarguments == 1){
						tokens.insert(tpos, "ARG:" + str);
						argCount++;
					} else {
						tokens.insert(tpos, "VAR:" + str);
					}
					str = "";
					tok = "";
				}
			} else if(state == 2){
				if(isnumber == 1){
					tokens.insert(tpos, "NUM:" + str);
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
			if(parcount > 0){
				parcount--;
				if(expr_started == 1){
					cnd += tok;
					tok = "";
				} else if(varstarted == 1){
					var += tok;
					tok = "";
				} else {
					str += tok;
					tok = "";
				}
			} else {
				if(state == 1){
					if(functionarguments == 1){
						if(str != ""){
							tokens.insert(tpos, "ARG:" + str);
							argCount++;
						}
						functionarguments = 0;
					} else if(ifstarted == 1){
						tokens.insert(tpos, "EXP:" + cnd);
						tokens.insert(tpos, "BIF:" + std::to_string(ifcounter));
						tokens.insert(tpos, "BEL:" + std::to_string(ifcounter));
						tokens.insert(tpos, "EIF:" + std::to_string(ifcounter++));
						tpos = std::prev(tpos, 2);
						ifstarted = 0;
						expr_started = 0;
						cnd = "";
					} else if(whilestarted == 1){
						tokens.insert(tpos, "EXP:" + cnd);
						tokens.insert(tpos, "BWH:" + std::to_string(whilecounter));
						tokens.insert(tpos, "EWH:" + std::to_string(whilecounter++));
						tpos = std::prev(tpos, 1);
						whilestarted = 0;
						expr_started = 0;
						cnd = "";
					} else if(str != ""){
						tokens.insert(tpos, "VAR:" + str);
					}
				} else if(state == 2){
					if(isnumber == 1){
						tokens.insert(tpos, "NUM:" + str);
						isnumber = 0;
					}
				}

				if(function_call == 1 and expr_started == 1 and cnd != ""){
					tokens.insert(tpos, "EXP:" + cnd);
					cnd = "";
					function_call = 0;
					expr_started = 0;
					tpos = std::next(tpos, 1);
				}
				infunction = 0;
				state = 0;
				str = "";
			}
			tok = "";
		} else if(tok == "]"){
			if(state == 1){
				tokens.insert(tpos, "OPT:" + str);
				state = 0;
				tok = "";
				str = "";
			}
		} else if(functionStarted == 1 and functionNameStatus == 1){
			str += tok;
			tok = "";
		} else if(state > 0){
			if(expr_started == 1){
				if(tok == "(" or tok == " ("){
					parcount++;
				}
				cnd += tok;
				tok = "";
			} else if(tok == "#"){
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
		}
	}
}

void RNTourThread::parse(){

	std::map<std::string, wcontent_t>::iterator fit_main;
	fit_main = functions.find("main");
	if(fit_main != functions.end()){
		wcontent_t content = fit_main->second;
		std::map<std::string, std::string> symbols;
		parse(fit_main->first, &content);
		fit_main->second = content;
	} else {
		printf("FUNCTION MAIN NOT FOUND\n");
	}
}

void RNTourThread::parse(std::string functionName, wcontent_t* content){
	std::list<std::string>::iterator argumentsIdx = content->arguments.begin();
	std::list<std::string> functionTokens = content->tokens;
	std::map<std::string, std::string>* functionSymbols = &content->symbols;
	std::list<std::string>::iterator it = functionTokens.begin();
	std::stack<std::string> ifIdsStack;
	std::stack<std::string> whileIdsStack;
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
					std::map<std::string, std::string>::iterator dir = globalSymbols.find((*it2));
					if(dir != globalSymbols.end()){
						int direction = std::stoi(globalSymbols.at((*it2)).substr(3));
						moveAround(direction);
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
			tripTo(sector, px, py);
			
			it = std::next(it, 4);
		} else if((*it) == "TURN"){
			if(it2 != functionTokens.end()){
				if((*it2).substr(0, 3) == "NUM"){
					printf("TURN %s DEGREES COMMAND\n", (*it2).substr(4).c_str());
					it = std::next(it, 2);
				}
			} 
		} else if((*it).substr(0, 3) == "FNC"){
			printf("CALL FUNCTION\n");
			std::string nextFunctionName = (*it).substr(4);
			std::map<std::string, wcontent_t>::iterator fit_call;
			fit_call = functions.find(nextFunctionName);
			if(fit_call != functions.end()){
				wcontent_t content = fit_call->second;
				std::list<std::string>::iterator itArgs = std::next(it, 1);
				while(*itArgs != "EFC"){
					content.arguments.emplace_back(evaluateExpression((*itArgs).substr(4), *functionSymbols));
					itArgs++;
				}
				
				if(content.requiredArguments != content.arguments.size()){
					fprintf(stderr, "error: no matching funtion to call to %s. expects %d arguments, provided %d.\n", (*it).substr(4).c_str(), content.requiredArguments, content.arguments.size());
				} else {
					parse(nextFunctionName, &content);
					it = itArgs;
				}
				fit_call->second = content;
			} else {
				printf("FUNCTION %s NOT FOUND\n", nextFunctionName.c_str());
			}
			it = std::next(it, 1);
		} else if((*it) == "SAY"){
			if(it2 != functionTokens.end()){
				if((*it2).substr(0, 3) == "STR"){
					lips->textToViseme((*it2).substr(4));
					it = std::next(it, 2);
				} else if((*it2).substr(0, 3) == "VAR"){
					if(functionSymbols->find((*it2).substr(4)) != functionSymbols->end()){
						printf("SAY VAR: %s\n", functionSymbols->at((*it2).substr(4)).c_str());
						//lips->textToViseme((*it2).substr(4));
					} else {
						fprintf(stderr, "Undefined VAR: %s\n", (*it2).substr(4).c_str());
					}
					it = std::next(it, 2);
				} else if((*it2).substr(0, 3) == "OPT"){
					if(it3 != functionTokens.end() and (*it3).substr(0, 3) == "STR"){
						printf("SAY STRING %s WITH OPTIONS: %s\n", (*it3).substr(4).c_str(), (*it2).substr(4).c_str());
						std::map<std::string, std::string> opts = createOptionsMap((*it2).substr(4));
						processOptions(opts);
						lips->textToViseme((*it3).substr(4));
						it = std::next(it, 3);
					}
				}
			} 
		} else if((*it).substr(0, 3) == "ARG"){
			if(argumentsIdx != content->arguments.end()){
				functionSymbols->emplace((*it).substr(4), *argumentsIdx);
				argumentsIdx++;
			} else {
				fprintf(stderr, "error: invalid number of arguments in function %s\n", functionName.c_str());
				exit(-1);
			}
			it++;
		} else if((*it).substr(0, 3) == "DVR"){
			if(it2 != functionTokens.end() and it3 != functionTokens.end()){
				if((*it2) == "EQ" and (*it3).substr(0, 3) == "EXP"){
					std::string r = evaluateExpression((*it3).substr(4), *functionSymbols);
					if(functionSymbols->find((*it).substr(4)) == functionSymbols->end()){
						functionSymbols->emplace((*it).substr(4), r);
					} else {
						functionSymbols->at((*it).substr(4)) = r;
					}
					it = std::next(it, 3);
				} else {
					printf("DECLARED VAR %s\n", (*it).substr(4).c_str());
					if(functionSymbols->find((*it).substr(4)) == functionSymbols->end()){
						functionSymbols->emplace((*it).substr(4), "UNK:nil");
					} else {
						fprintf(stderr, "VAR: %s, already exists\n", (*it).substr(4).c_str());
					}
					it++;
				}
			} else {
				printf("DECLARED VAR %s\n", (*it).substr(4).c_str());
				if(functionSymbols->find((*it).substr(4)) == functionSymbols->end()){
					functionSymbols->emplace((*it).substr(4), "UNK:nil");
				} else {
					fprintf(stderr, "VAR: %s, already exists\n", (*it).substr(4).c_str());
				}
				it++;
			}
		} else if((*it).substr(0, 3) == "VAR"){
			if(functionSymbols->find((*it).substr(4)) != functionSymbols->end()){
				if((*it2) == "EQ" and (*it3).substr(0, 3) == "EXP"){
					std::string r = evaluateExpression((*it3).substr(4), *functionSymbols);
					functionSymbols->at((*it).substr(4)) = r;
					it = std::next(it, 3);
				}
			} else {
				fprintf(stderr, "Undefined VAR: %s\n", (*it).substr(4).c_str());
				it++;
			}
		} else if((*it) == "RETURN"){
			if(it2 != functionTokens.end() and (*it2).substr(0, 3) == "EXP"){
				std::string r = evaluateExpression((*it2).substr(4), *functionSymbols);
				functions.at(functionName).result = r;
			}
			it = functionTokens.end();
		} else if((*it) == "IF"){
			std::string r = evaluateExpression((*it2).substr(4), *functionSymbols); //hacer recursive descent parser

			if(r == "NUM:1"){
				it = std::next(it, 3);
				ifIdsStack.push((*it3).substr(4));
			} else {
				std::string elsename = "BEL:" + (*it3).substr(4);
				while((*it) != elsename){
					it++;
				}
			}
		} else if((*it).substr(0, 3) == "BEL"){
			if(not ifIdsStack.empty()){
				if(ifIdsStack.top() == (*it).substr(4)){
					std::string elsename = "EIF:" + ifIdsStack.top();
					while((*it) != elsename){
						it++;
					}
					ifIdsStack.pop();
				}
			} else {
				it++;
			}
		} else if((*it).substr(0, 3) == "EIF"){
			if(not ifIdsStack.empty()){
				if(ifIdsStack.top() == (*it).substr(4)){
					ifIdsStack.pop();
				}
			}
			it++;
		} else if(*it == "WHILE"){
			std::string r = evaluateExpression((*it2).substr(4), *functionSymbols);
			if(r == "NUM:1"){
				it = std::next(it, 3);
				whileIdsStack.push((*it3).substr(4));
			} else {
				std::string endwname = "EWH:" + (*it3).substr(4);
				while((*it) != endwname){
					it++;
				}
			}
		} else if((*it).substr(0, 3) == "EWH"){
			if(not whileIdsStack.empty()){
				if(whileIdsStack.top() == (*it).substr(4)){
					whileIdsStack.pop();
					std::string bwhname = "BWH:" + (*it).substr(4);
					while((*it) != bwhname){
						it = std::prev(it);
					}
					it = std::prev(it, 2);
				}
			} else {
				it++;
			}
		} else {
			it++;
		}
	}
}
std::string RNTourThread::evaluateExpression(std::string expr, std::map<std::string, std::string> symbols){
	std::stack<std::string> exprStack;
	std::string result = "";
	std::list<std::string> tokens = tokenizeExpCond(expr);

	std::list<std::string>::iterator itokens;
	bool ok_sofar = true;
	for(itokens = tokens.begin(); itokens != tokens.end() and ok_sofar; itokens++){
		if((*itokens).substr(0, 3) == "VAR"){
			std::map<std::string, std::string>::iterator var = symbols.find((*itokens).substr(4));
			if(var != symbols.end()){
				*itokens = var->second;
			} else {
				fprintf(stderr, "error: Undefined variable %s in this scope\n", (*itokens).substr(4).c_str());
				ok_sofar = false;
			}
		} else if((*itokens).substr(0, 3) == "FNC"){
			std::map<std::string, wcontent_t>::iterator itFnc;
			itFnc = functions.find((*itokens).substr(4));
			if(itFnc != functions.end()){
				wcontent_t content = itFnc->second;

				std::list<std::string>::iterator itArgs = std::next(itokens, 1);
				while(*itArgs != "EFC"){
					content.arguments.emplace_back(evaluateExpression((*itArgs).substr(4), symbols));
					itArgs++;
				}
				
				if(content.requiredArguments != content.arguments.size()){
					fprintf(stderr, "error: no matching funtion to call to %s. expects %d arguments, provided %d.\n", (*itokens).substr(4).c_str(), content.requiredArguments, content.arguments.size());
				} else {
					parse(itFnc->first, &content);
				}

				if(functions.at(itFnc->first).result != "VOID"){
					tokens.insert(itokens, functions.at(itFnc->first).result);
					tokens.erase(itokens, std::next(itArgs, 1));
					itokens = tokens.begin();
				} else {
					fprintf(stderr, "error: Function %s does not return value\n", (*itokens).substr(4).c_str());
				}
				
			} else {
				fprintf(stderr, "error: Undefined function %s in this scope\n", (*itokens).substr(4).c_str());
				ok_sofar = false;
			}
		}
	}

	solveExpParenthesis(&tokens);
	if(tokens.size() > 1){
		solveExp(&tokens);
	}
	result = tokens.front();
	return result;
}

void RNTourThread::solveExpParenthesis(std::list<std::string>* tokens){
	std::list<std::string> mini_tokens;
	std::list<std::string>::reverse_iterator itoks_s = std::find(tokens->rbegin(), tokens->rend(), "(");
	int idx_start = RN_NONE, idx_end = RN_NONE;
	if(itoks_s != tokens->rend()){
		idx_start = (tokens->size() - 1) - std::distance(tokens->rbegin(), itoks_s);
	}
	std::list<std::string>::iterator itoks_e = std::find(std::next(tokens->begin(), idx_start), tokens->end(), ")");
	if(itoks_e != tokens->end()){
		idx_end = std::distance(tokens->begin(), itoks_e);
	}

	if(idx_start != RN_NONE and idx_end != RN_NONE){
		
		mini_tokens.insert(mini_tokens.end(), std::next(tokens->begin(), idx_start + 1), std::next(tokens->begin(), idx_end));
		solveExp(&mini_tokens);
		tokens->erase(std::next(tokens->begin(), idx_start), std::next(tokens->begin(), idx_end + 1));
		tokens->insert(std::next(tokens->begin(), idx_start), mini_tokens.front());
		
		solveExpParenthesis(tokens);
	}
}

void RNTourThread::solveExp(std::list<std::string>* tokens){
	factor(tokens);
	term(tokens);
	simpExpr(tokens);
}

void RNTourThread::factor(std::list<std::string>* tokens){
	bool fac = true;
	while(fac){
		fac = false;
		std::list<std::string>::iterator itTokens = tokens->begin();
		int idx_start = RN_NONE, idx_end = RN_NONE, idx = 0;
		std::string op1 = "", op2 = "", oper = "", result = "";
		while(itTokens != tokens->end()){
			if(*itTokens == "*" or *itTokens == "/" or *itTokens == "%" or *itTokens == "AND"){
				fac = true;
				op1 = *std::prev(itTokens);
				op2 = *std::next(itTokens, 1);
				oper = *itTokens;
				idx_start = idx - 1;
				idx_end = idx + 1;
				itTokens = tokens->end();
			} else {
				itTokens++;
				idx++;
			}
		}
		if(fac){
			if(op1 != "" and op2 != "" and oper != ""){
				if(op1.substr(0, 3) == op2.substr(0, 3)){
					if(op1.substr(0, 3) == "NUM"){
						float res;
						if(oper == "*"){
							res = std::stof(op1.substr(4)) * std::stof(op2.substr(4));
						} else if(oper == "/"){
							res = std::stof(op1.substr(4)) / std::stof(op2.substr(4));
						} else if(oper == "%"){
							res = (float)(static_cast<int>(std::stof(op1.substr(4))) % static_cast<int>(std::stof(op2.substr(4))));
						} else if(oper == "AND"){
							res = std::stof(op1.substr(4)) and std::stof(op2.substr(4));
						}
						result = "NUM:" + std::to_string(res);
					} else {
						fprintf(stderr, "error: invalid type operation\n");
					}
				} else {
					fprintf(stderr, "error: invalid type operation\n");
				}
			}

			if(idx_start != RN_NONE and idx_end != RN_NONE){
				tokens->erase(std::next(tokens->begin(), idx_start), std::next(tokens->begin(), idx_end + 1));
				tokens->insert(std::next(tokens->begin(), idx_start), result);
				
			}
		}
		
	}
}

void RNTourThread::term(std::list<std::string>* tokens){
	bool trm = true;
	while(trm){
		trm = false;
		std::list<std::string>::iterator itTokens = tokens->begin();
		int idx_start = RN_NONE, idx_end = RN_NONE, idx = 0;
		std::string op1 = "", op2 = "", oper = "", result = "";
		while(itTokens != tokens->end()){
			if(*itTokens == "+" or *itTokens == "-" or *itTokens == "OR"){
				trm = true;
				op1 = *std::prev(itTokens);
				op2 = *std::next(itTokens, 1);
				oper = *itTokens;
				idx_start = idx - 1;
				idx_end = idx + 1;
				itTokens = tokens->end();
			} else {
				itTokens++;
				idx++;
			}
		}
		if(trm){
			if(op1 != "" and op2 != "" and oper != ""){
				if(op1.substr(0, 3) == op2.substr(0, 3)){
					if(op1.substr(0, 3) == "NUM"){
						float res;
						if(oper == "+"){
							res = std::stof(op1.substr(4)) + std::stof(op2.substr(4));
						} else if(oper == "-"){
							res = std::stof(op1.substr(4)) - std::stof(op2.substr(4));
						} else if(oper == "OR"){
							res = std::stof(op1.substr(4)) or std::stof(op2.substr(4));
						}
						result = "NUM:" + std::to_string(res);
					} else {
						fprintf(stderr, "error: invalid type operation\n");
					}
				} else {
					fprintf(stderr, "error: invalid type operation\n");
				}
			}

			if(idx_start != RN_NONE and idx_end != RN_NONE){
				tokens->erase(std::next(tokens->begin(), idx_start), std::next(tokens->begin(), idx_end + 1));
				tokens->insert(std::next(tokens->begin(), idx_start), result);
			}
		}
	}
}

void RNTourThread::simpExpr(std::list<std::string>* tokens){
	bool se = true;

	while(se){
		se = false;
		std::list<std::string>::iterator itTokens = tokens->begin();
		int idx_start = RN_NONE, idx_end = RN_NONE, idx = 0;
		std::string op1 = "", op2 = "", oper = "", result = "";
		while(itTokens != tokens->end()){
			if(*itTokens == "EQU" or *itTokens == "NEQ" or *itTokens == "LT" or *itTokens == "LEQ" or *itTokens == "GT" or *itTokens == "GEQ"){
				se = true;
				op1 = *std::prev(itTokens);
				op2 = *std::next(itTokens, 1);
				oper = *itTokens;
				idx_start = idx - 1;
				idx_end = idx + 1;
				itTokens = tokens->end();
			} else {
				itTokens++;
				idx++;
			}
		}
		if(se){
			if(op1 != "" and op2 != "" and oper != ""){
				if(op1.substr(0, 3) == op2.substr(0, 3)){
					int res;
					if(oper == "EQU"){
						if(op1.substr(0, 3) == "NUM"){
							res = std::stof(op1.substr(4)) == std::stof(op2.substr(4));
						} else {
							res = op1.substr(4) == op2.substr(4);
						}
					} else if(oper == "NEQ"){
						if(op1.substr(0, 3) == "NUM"){
							res = std::stof(op1.substr(4)) != std::stof(op2.substr(4));
						} else {
							res = op1.substr(4) != op2.substr(4);
						}
					} else if(oper == "LT"){
						if(op1.substr(0, 3) == "NUM"){
							res = std::stof(op1.substr(4)) < std::stof(op2.substr(4));
						} else {
							res = op1.substr(4) < op2.substr(4);
						}
					} else if(oper == "LEQ"){
						if(op1.substr(0, 3) == "NUM"){
							res = std::stof(op1.substr(4)) <= std::stof(op2.substr(4));
						} else {
							res = op1.substr(4) <= op2.substr(4);
						}
					} else if(oper == "GT"){
						if(op1.substr(0, 3) == "NUM"){
							res = std::stof(op1.substr(4)) > std::stof(op2.substr(4));
						} else {
							res = op1.substr(4) > op2.substr(4);
						}
					} else if(oper == "GEQ"){
						if(op1.substr(0, 3) == "NUM"){
							res = std::stof(op1.substr(4)) >= std::stof(op2.substr(4));
						} else {
							res = op1.substr(4) >= op2.substr(4);
						}
					}
					result = "NUM:" + std::to_string(res);
					
				} else {
					fprintf(stderr, "error: invalid type operation\n");
				}
			}

			if(idx_start != RN_NONE and idx_end != RN_NONE){
				tokens->erase(std::next(tokens->begin(), idx_start), std::next(tokens->begin(), idx_end + 1));
				tokens->insert(std::next(tokens->begin(), idx_start), result);
			}
		}
	}
}

std::list<std::string> RNTourThread::tokenizeExpCond(std::string expr_cond){
	std::string::iterator it = expr_cond.begin();
	std::list<std::string> tokens;
	std::string tok = "";
	std::string str = "";
	std::string cnd = "";
	int state = 0;
	int evalfunc = 0;
	int expr_started = 0;
	while(it != expr_cond.end()){
		if(tok == "and" and (*it == '(' or *it == ')' or *it == '\"' or *it == '#')){
			tokens.emplace_back("AND");
			tok = "";
		} else if(tok == "or" and (*it == '(' or *it == ')' or *it == '\"' or *it == '#')){
			tokens.emplace_back("OR");
			tok = "";
		} else if(tok == "not" and (*it == '(' or *it == ')' or *it == '\"' or *it == '#')){
			tokens.emplace_back("NOT");
			tok = "";
		} 

		if(expr_started == 1){
			tok += *it;
			if(*it == ' '){
				tok = " ";
			} else if(*it == ','){
				tokens.emplace_back("EXP:" + cnd);
				cnd = "";
			} else if(*it == ')'){
				tokens.emplace_back("EXP:" + cnd);
				expr_started = 0;
				if(evalfunc == 1){
					tokens.emplace_back("EFC");
					evalfunc = 0;
				}
			} else {
				cnd += tok;
			}
		} else {
			if(*it == ' '){
				if(state == 2){
					if(str != ""){
						tokens.emplace_back("NUM:" + str);
					}
					str = "";
					state = 0;
					tok = "";
				} else if(tok != "" and (tok != "and" and tok != "or" and tok != "not") and std::regex_match(tok, std::regex("[a-zA-Z_$][a-zA-Z_$0-9]*"))){
					tokens.emplace_back("VAR:" + tok);
				} else if(tok == "and"){
					tokens.emplace_back("AND");
				} else if(tok == "or"){
					tokens.emplace_back("OR");
				} else if(tok == "not"){
					tokens.emplace_back("NOT");
				}
				tok = "";
			} else if(*it == '#'){ 
				if(state == 0){
					state = 2;
				}
				tok = "";
			} else if (*it == '\"'){
				if(state == 0){
					state = 1;
				} else if (state == 1){
					if(str != ""){
						tokens.emplace_back("STR:" + str);
					}
					str = "";
					state = 0;
				}
				tok = "";
			} else if(*it == '('){
				if(tok != ""){
					tokens.emplace_back("FNC:" + tok);
					expr_started = 1;
					evalfunc = 1;
				} else {
					tokens.emplace_back("(");
				}
				tok = "";
			
			} else if(*it == ')'){
				if(evalfunc == 1){
					tokens.emplace_back("EFC");
					evalfunc = 0;
				} else {
					if(tok != "" and std::regex_match(tok, std::regex("[a-zA-Z_$][a-zA-Z_$0-9]*"))){
						tokens.emplace_back("VAR:" + tok);
					} else if(state == 2){
						if(str != ""){
							tokens.emplace_back("NUM:" + str);
						}
						str = "";
						state = 0;
					}
					tokens.emplace_back(")");
				}
				tok = "";
			} else if(*it == '+' or *it == '-' or *it == '*' or *it == '/' or *it == '%'){
				if(tok != "" or str != ""){
					if(state == 2){
						if(str != ""){
							tokens.emplace_back("NUM:" + str);
						}
						str = "";
						state = 0;
					} else if(tok != "" and std::regex_match(tok, std::regex("[a-zA-Z_$][a-zA-Z_$0-9]*"))){
						tokens.emplace_back("VAR:" + tok);
					}
					tok = "";
				}
				std::string oper = "";
				oper += *it;
				tokens.emplace_back(oper);
				tok = "";
			} else if(*it == '=' or *it == '!' or *it == '<'  or *it == '>'){
				if(tok != "" or str != ""){
					if(state == 2){
						if(str != ""){
							tokens.emplace_back("NUM:" + str);
						}
						str = "";
						state = 0;
					} else if(tok != "" and std::regex_match(tok, std::regex("[a-zA-Z_$][a-zA-Z_$0-9]*"))){
						tokens.emplace_back("VAR:" + tok);
					}
					tok = "";
				}
				std::string::iterator it2 = std::next(it, 1);
				if(*it2 == '='){
					if(*it == '='){
						tokens.emplace_back("EQU");
						tok = "";
					} else if(*it == '!'){
						tokens.emplace_back("NEQ");
						tok = "";
					} else if(*it == '<'){
						tokens.emplace_back("LEQ");
						tok = "";
					} else if(*it == '>'){
						tokens.emplace_back("GEQ");
						tok = "";
					}
					it++;
				} else {
					if(*it == '='){
						
					} else if(*it == '!'){
						
					} else if(*it == '<'){
						tokens.emplace_back("LT");
						tok = "";
					} else if(*it == '>'){
						tokens.emplace_back("GT");
						tok = "";
					}
				}
			} else if(state > 0){
				str += *it;
				tok = "";
			} else if(state == 0){
				tok += *it;
			}
		}
		it++;
	}	
	if(tok != "" or str != ""){
		if(tok != "" and std::regex_match(tok, std::regex("[a-zA-Z_$][a-zA-Z_$0-9]*"))){
			tokens.emplace_back("VAR:" + tok);
		} else if(state == 2){
			if(str != ""){
				tokens.emplace_back("NUM:" + str);
			}
			str = "";
			state = 0;
		}
	}
	return tokens;
}

std::map<std::string, std::string> RNTourThread::createOptionsMap(std::string opts){
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

void RNTourThread::processOptions(std::map<std::string, std::string> opts){
	std::map<std::string, std::string>::iterator optsIt;
	std::map<std::string, std::string>::iterator op;
	for(optsIt = opts.begin(); optsIt != opts.end(); optsIt++){
		if(optsIt->first == "face"){
			op = globalSymbols.find("FACE:" + optsIt->second);
			if(op != opts.end()){
				gn->setEmotionsResult("", op->second.substr(3));
			}
		} else if(optsIt->first == "attention"){
			op = globalSymbols.find("ATTN:" + optsIt->second);
			if(op != opts.end()){
				gn->setEmotionsResult("", op->second.substr(3));
			}
		}
		RNUtils::sleep(100);
	}
}
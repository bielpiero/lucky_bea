#include "RNTourTask.h"

RNTourTask::RNTourTask(const GeneralController* gn, const char* name, const char* description) : RNRecurrentTask(gn, name, description){
	this->gn = (GeneralController*)gn;
	xmlSectorsPath = RNUtils::getApplicationPath() + XML_FILE_SECTORS_PATH;
	currentMapGraph = NULL;
	initialized = false;
	lastSiteVisitedIndex = RN_NONE;
}

RNTourTask::~RNTourTask(){

}

void RNTourTask::task(){
	if(initialized){

	} else {
		init();
	}
}

void RNTourTask::onKilled(){
	initialized = false;
	lastSiteVisitedIndex = RN_NONE;
}

int RNTourTask::createCurrentMapGraph(){
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
			if(not currentMapGraph->empty()){
				RNUtils::printLn("%s", currentMapGraph->toString().c_str());
			}
			root_node = doc.first_node(XML_ELEMENT_SECTORS_STR);
			for (xml_node<> * sector_node = root_node->first_node(XML_ELEMENT_SECTOR_STR); sector_node; sector_node = sector_node->next_sibling()){
				int xmlSectorId = std::atoi(sector_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value());
				std::string ady = std::string(sector_node->first_attribute(XML_ATTRIBUTE_ADYACENCY_STR)->value());
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
	}
	return res;
}

void RNTourTask::init(){
	if(gn != NULL){
		if(createCurrentMapGraph() == RN_OK){
			sequence = RNUtils::split(gn->getCurrentSector()->getSequence(), ",");
			initialized = true;
		}
		
	}
}
#include "MapSector.h"

MapSector::MapSector(){
	sitesCyclic = false;
	this->polygonDefinition = "";
	landmarks = new std::list<s_landmark*>();
	features = new std::list<s_feature*>();
	sites = new std::list<s_site*>();
	ways = new std::list<s_way*>();
	tags = new std::list<s_tag*>();
	polygon = new std::vector<PointXY*>();
}

MapSector::~MapSector(){
	deleteAllLandmarks();
    delete landmarks;

    deleteAllFeatures();
    delete features;

    deleteAllSites();
    delete sites;

    deleteAllWays();
    delete ways;

    deleteAllTags();
    delete tags;

    deletePolygon();
    delete polygon;
}

MapSector* MapSector::clone(){
	MapSector* copy = new MapSector();
	copy->id = this->id;
	copy->mapId = this->mapId;
	copy->width = this->width;
	copy->height = this->height;
	copy->name = this->name;
	copy->sitesCyclic = this->sitesCyclic;
	copy->polygonDefinition = this->polygonDefinition;

	std::list<s_landmark*>::iterator lit = landmarks->begin();
	while(lit != landmarks->end()){
		copy->landmarks->emplace_back(new s_landmark(**lit));
		lit++;
	}
	std::list<s_feature*>::iterator fit = features->begin();
	while(fit != features->end()){
		copy->features->emplace_back(new s_feature(**fit));
		fit++;
	}
	std::list<s_site*>::iterator sit = sites->begin();
	while(sit != sites->end()){
		copy->sites->emplace_back(new s_site(**sit));
		sit++;
	} 
	std::list<s_way*>::iterator wit = ways->begin();
	while(wit != ways->end()){
		copy->ways->emplace_back(new s_way(**wit));
		wit++;
	}
	std::list<s_tag*>::iterator tit = tags->begin();
	while(tit != tags->end()){
		copy->tags->emplace_back(new s_tag(**tit));
		tit++;
	} 
	std::vector<PointXY*>::iterator pit = polygon->begin();
	while(pit != polygon->end()){
		copy->polygon->push_back(new PointXY(**pit));
		pit++;
	}
	/*copy->landmarks->resize(this->landmarks->size());
	copy->features->resize(this->features->size());
	copy->sites->resize(this->sites->size());
	copy->ways->resize(this->ways->size());
	copy->tags->resize(this->tags->size());
	copy->polygon->reserve(this->polygon->size());


	std::copy(this->landmarks->begin(), this->landmarks->end(), copy->landmarks->begin());
	std::copy(this->features->begin(), this->features->end(), copy->features->begin());
	std::copy(this->sites->begin(), this->sites->end(), copy->sites->begin());
	std::copy(this->ways->begin(), this->ways->end(), copy->ways->begin());
	std::copy(this->tags->begin(), this->tags->end(), copy->tags->begin());
	std::copy(this->polygon->begin(), this->polygon->end(), copy->polygon->begin());*/
	return copy;
}

void MapSector::setId(int id){ 
	this->id = id; 
}

int MapSector::getId(){
	return this->id;
}

void MapSector::setMapId(int id){
	this->mapId = id;
}

int MapSector::getMapId(){
	return this->mapId;
}

bool MapSector::isHallway(void){
	return RNUtils::toLowercase(this->name).find(std::string(SEMANTIC_HALLWAY_STR)) != std::string::npos;
}

void MapSector::setName(std::string name){ 
	this->name = name; 
}

std::string MapSector::getName(){
	return this->name; 
}

void MapSector::setWidth(double width){ 
	this->width = width; 
}

double MapSector::getWidth(){ 
	return this->width; 
}

void MapSector::setHeight(double height){ 
	this->height = height; 
}

double MapSector::getHeight(){ 
	return this->height; 
}

void MapSector::setSequence(std::string sequence){ 
	this->sequence = sequence; 
}

std::string MapSector::getSequence(){ 
	return this->sequence; 
}

void MapSector::addSite(s_site* site){ 
	sites->emplace_back(site); 
}

void MapSector::addWay(s_way* way){ 
	ways->emplace_back(way); 
}

void MapSector::addFeature(s_feature* feature){ 
	features->emplace_back(feature); 
}

void MapSector::addLandmark(s_landmark* landmark){ 
	landmarks->emplace_back(landmark); 
}

void MapSector::addTag(s_tag* tag){ 
	tags->emplace_back(tag); 
}

s_site* MapSector::siteAt(int index) {
	return *(std::next(sites->begin(), index));
}

s_way* MapSector::wayAt(int index) {
	return *(std::next(ways->begin(), index));
}

s_feature* MapSector::featureAt(int index) {
	return *(std::next(features->begin(), index));
}

s_landmark* MapSector::landmarkAt(int index) {
	return *(std::next(landmarks->begin(), index));
}

s_tag* MapSector::tagAt(int index) {
	return *(std::next(tags->begin(), index));
}

s_site* MapSector::findSiteById(int id) { 
	bool found = false;
	s_site* s = NULL;
	std::list<s_site*>::iterator it;
	for (it = sites->begin(); it != sites->end() and not found; it++){
		if((*it)->id == id){
			found = true;
			s = *it;
		}
	}

	return s;
}

s_feature* MapSector::findFeatureById(int id) {
	bool found = false;
	s_feature* f = NULL;
	std::list<s_feature*>::iterator it;
	for (it = features->begin(); it != features->end() and not found; it++){
		if((*it)->id == id){
			found = true;
			f = *it;
		}
	}

	return f;
}

s_tag* MapSector::findTagById(std::string id) {
	bool found = false;
	s_tag* t = NULL;
	std::list<s_tag*>::iterator it;
	for (it = tags->begin(); it != tags->end() and not found; it++){
		if((*it)->id == id){
			found = true;
			t = *it;
		}
	}

	return t;
}

std::list<s_site*> MapSector::findSitesByName(std::string name){
	bool found = false;
	std::list<s_site*> values; 
	std::list<s_site*>::iterator it;
	for (it = sites->begin(); it != sites->end() and not found; it++){
		if((*it)->name == name){
			values.emplace_back(*it);
		}
	}
	
	return values;
}

std::list<s_feature*> MapSector::findFeaturesByName(std::string name){
	bool found = false;
	std::list<s_feature*> values;
	std::list<s_feature*>::iterator it;
	for (it = features->begin(); it != features->end() and not found; it++){
		if((*it)->name == name){
			values.emplace_back(*it);
		}
	}
	return values;
}

void MapSector::deleteSite(s_site* obj) { 
	sites->remove(obj); 
}

void MapSector::deleteWay(s_way* obj) { 
	ways->remove(obj); 
}

void MapSector::deleteFeature(s_feature* obj) { 
	features->remove(obj); 
}

void MapSector::deleteLandmark(s_landmark* obj) { 
	landmarks->remove(obj); 
}

void MapSector::deleteTag(s_tag* obj) { 
	tags->remove(obj); 
}

void MapSector::deleteSiteAt(int index) { 
	sites->erase(std::next(sites->begin(), index)); 
}

void MapSector::deleteWayAt(int index) { 
	ways->erase(std::next(ways->begin(), index)); 
}

void MapSector::deleteFeatureAt(int index) { 
	features->erase(std::next(features->begin(), index));
}

void MapSector::deleteLandmarkAt(int index) { 
	landmarks->erase(std::next(landmarks->begin(), index)); 
}

void MapSector::deleteTagAt(int index) { 
	tags->erase(std::next(tags->begin(), index)); 
}

size_t MapSector::sitesSize() { 
	return sites->size(); 
}

size_t MapSector::waysSize() { 
	return ways->size(); 
}

size_t MapSector::featuresSize() { 
	return features->size(); 
}

size_t MapSector::landmarksSize() { 
	return landmarks->size(); 
}

size_t MapSector::tagsSize() { 
	return tags->size(); 
}

size_t MapSector::landmarksSizeByType(std::string type){
	size_t count = 0;
	std::list<s_landmark*>::iterator it;
	for(it = landmarks->begin(); it != landmarks->end(); it++){
		if((*it)->type == type){
			count++;
		}
	}

	return count;
}

s_landmark* MapSector::landmarkByTypeAndId(std::string type, int id){
	bool found = false;
	s_landmark* l = NULL;
	std::list<s_landmark*>::iterator it;
	for(it = landmarks->begin(); it != landmarks->end() and not found; it++){
		if((*it)->type == type){
			if(id == (*it)->id){
				l = *it;
				found = true;
			}
		}
	}

	return l;
}

void MapSector::deleteAllSites() { 
	while(!sites->empty()) {
        delete sites->front();
        sites->pop_front();
    }
}

void MapSector::deleteAllWays() { 
	while(!ways->empty()) {
        delete ways->front();
        ways->pop_front();
    }
}

void MapSector::deleteAllFeatures(){
	while(!features->empty()) {
        delete features->front();
        features->pop_front();
    }
}

void MapSector::deleteAllLandmarks(){
	while(!landmarks->empty()) {
        delete landmarks->front();
        landmarks->pop_front();
    }
}

void MapSector::deleteAllTags(){
	while(!tags->empty()) {
        delete tags->front();
        tags->pop_front();
    }
}

void MapSector::deletePolygon(){
	for (int i = 0; i < polygon->size(); i++) {
        delete polygon->at(i);
    }
    polygon->clear();
}

void MapSector::setPolygon(std::string polygon){
	this->polygonDefinition = polygon;
	getPolygonFromString();
}
std::string MapSector::getPolygonDefinition(){
	return this->polygonDefinition;
}

void MapSector::getPolygonFromString(){
	polygon->clear();
	std::vector<std::string> values = RNUtils::split((char*)this->polygonDefinition.c_str(), " ");
	double px = 0, py = 0;
	PointXY mPoint;
	for(unsigned int idx = 0; idx < values.size(); ){

		if(values.at(idx) == "m" or values.at(idx) == "M"){
			px = std::atof(values.at(idx + 1).c_str()) / 100.0;
			py = std::atof(values.at(idx + 2).c_str()) / 100.0;
			if(px < 1.0e-7){
				px = 0.0f;
			}
			if(py < 1.0e-7){
				py = 0.0f;
			}
			polygon->push_back(new PointXY(px, py));
			mPoint = PointXY(px, py);
			idx += 3;
		} else if(values.at(idx) == "l" or values.at(idx) == "L"){
			px = polygon->at(polygon->size() - 1)->getX() + (std::atof(values.at(idx + 1).c_str()) / 100.0);
			py = polygon->at(polygon->size() - 1)->getY() + (std::atof(values.at(idx + 2).c_str()) / 100.0);
			if(px < 1.0e-7){
				px = 0.0f;
			}
			if(py < 1.0e-7){
				py = 0.0f;
			}
			polygon->push_back(new PointXY(px, py));
			idx += 3;
		} else if(values.at(idx) == "q" or values.at(idx) == "Q"){
			std::vector<PointXY> bezierPointXYs;
			bezierPointXYs.push_back(*polygon->at(polygon->size() - 1));

			px = polygon->at(polygon->size() - 1)->getX() + (std::atof(values.at(idx + 1).c_str()) / 100.0);
			py = polygon->at(polygon->size() - 1)->getY() + (std::atof(values.at(idx + 2).c_str()) / 100.0);
			if(px < 1.0e-7){
				px = 0.0f;
			}
			if(py < 1.0e-7){
				py = 0.0f;
			}
			bezierPointXYs.push_back(PointXY(px, py));

			px = polygon->at(polygon->size() - 1)->getX() + (std::atof(values.at(idx + 3).c_str()) / 100.0);
			py = polygon->at(polygon->size() - 1)->getY() + (std::atof(values.at(idx + 4).c_str()) / 100.0);
			if(px < 1.0e-7){
				px = 0.0f;
			}
			if(py < 1.0e-7){
				py = 0.0f;
			}
			bezierPointXYs.push_back(PointXY(px, py));

			std::vector<PointXY> bezierCurve;
			
			RNUtils::getBezierCurve(bezierPointXYs, bezierCurve);
			for(unsigned int jdx = 1; jdx < bezierCurve.size(); jdx++){
				polygon->push_back(new PointXY(bezierCurve.at(jdx)));
			}
			idx += 5;
		} else if(values.at(idx) == "z" or values.at(idx) == "Z"){
			polygon->push_back(new PointXY(mPoint));
			idx++;
		}
	}
	std::sort(polygon->begin(), polygon->end(), polygonSorter);
}

bool MapSector::polygonSorter(PointXY* a, PointXY* b){
	return (a->getY() < b->getY()) and (a->getX() < b->getX());
}

double MapSector::getAngle(PointXY a, PointXY b){
	double result = 0;
	result = std::atan2(b.getY(), b.getX()) - std::atan2(a.getY(), a.getX());
	while(result > M_PI){
		result = result - 2 * M_PI;
	} 

	while(result < -M_PI) {
		result = result + 2 * M_PI;
	}

	return result;
}

bool MapSector::checkPointXYInPolygon(PointXY g){
	PointXY a;
	PointXY b;
	bool result = false;

	for(int i=0, j = polygon->size() - 1; i < polygon->size(); i++){
    	if (((polygon->at(i)->getY() > g.getY()) != (polygon->at(j)->getY() > g.getY())) and 
    		(g.getX() < ((polygon->at(j)->getX() - polygon->at(i)->getX()) * (g.getY() - polygon->at(i)->getY()) / (polygon->at(j)->getY() - polygon->at(i)->getY()) + polygon->at(i)->getX()))){
	       result = !result;
    	}
    	j = i;
    }
	return result;
}
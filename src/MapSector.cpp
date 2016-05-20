#include "MapSector.h"

MapSector::MapSector(){
	sitesCyclic = false;
	this->polygonDefinition = "";
	landmarks = new std::vector<s_landmark*>();
	features = new std::vector<s_feature*>();
	sites = new std::vector<s_site*>();
	polygon = new std::vector<PointXY*>();
}

MapSector::~MapSector(){
	deleteAllLandmarks();
    delete landmarks;

    deleteAllFeatures();
    delete features;

    deleteAllSites();
    delete sites;

    deletePolygon();
    delete polygon;
}

void MapSector::setId(int id){ 
	this->id = id; 
}

float MapSector::getId(){
	return this->id; 
}

void MapSector::setName(std::string name){ 
	this->name = name; 
}

std::string MapSector::getName(){
	return this->name; 
}

void MapSector::setWidth(float width){ 
	this->width = width; 
}

float MapSector::getWidth(){ 
	return this->width; 
}

void MapSector::setHeight(float height){ 
	this->height = height; 
}

float MapSector::getHeight(){ 
	return this->height; 
}

void MapSector::setIfSitesCyclic(bool sitesCyclic){ 
	this->sitesCyclic = sitesCyclic; 
}

float MapSector::isSitesCyclic(){ 
	return this->sitesCyclic; 
}

void MapSector::setSequence(std::string sequence){ 
	this->sequence = sequence; 
}

std::string MapSector::getSequence(){ 
	return this->sequence; 
}

void MapSector::addSite(s_site* site){ 
	sites->push_back(site); 
}

void MapSector::addFeature(s_feature* feature){ 
	features->push_back(feature); 
}

void MapSector::addLandmark(s_landmark* landmark){ 
	landmarks->push_back(landmark); 
}

s_site* MapSector::siteAt(int index) { 
	return sites->at(index); 
}

s_feature* MapSector::featureAt(int index) { 
	return features->at(index); 
}

s_landmark* MapSector::landmarkAt(int index) { 
	return landmarks->at(index); 
}

s_site* MapSector::findSiteById(int id) { 
	bool found = false;
	int index = NONE;
	for (int i = 0; i < sites->size() and not found; i++){
		if(sites->at(i)->id == id){
			found = true;
			index = i;
		}
	}
	if(index != NONE){
		return sites->at(index);
	} else {
		return NULL;
	}
}

s_feature* MapSector::findFeatureById(int id) { 
	bool found = false;
	int index = NONE;
	for (int i = 0; i < features->size() and not found; i++){
		if(features->at(i)->id == id){
			found = true;
			index = i;
		}
	}
	if(index != NONE){
		return features->at(index);
	} else {
		return NULL;
	}
}

std::vector<s_site*> MapSector::findSitesByName(std::string name){
	int index = NONE;
	std::vector<s_site*> values; 
	for (int i = 0; i < sites->size(); i++){
		if(sites->at(i)->name == name){
			values.push_back(sites->at(i));
		}
	}
	return values;
}

std::vector<s_feature*> MapSector::findFeaturesByName(std::string name){
	int index = NONE;
	std::vector<s_feature*> values;
	for (int i = 0; i < features->size(); i++){
		if(features->at(i)->name == name){
			values.push_back(features->at(i));
		}
	}
	return values;
}

void MapSector::deleteSite(s_site* obj) { 
	sites->erase(std::remove(sites->begin(), sites->end(), obj), sites->end()); 
}

void MapSector::deleteFeature(s_feature* obj) { 
	features->erase(std::remove(features->begin(), features->end(), obj), features->end()); 
}

void MapSector::deleteLandmark(s_landmark* obj) { 
	landmarks->erase(std::remove(landmarks->begin(), landmarks->end(), obj), landmarks->end()); 
}

void MapSector::deleteSiteAt(int index) { 
	sites->erase(sites->begin() + index); 
}

void MapSector::deleteFeatureAt(int index) { 
	features->erase(features->begin() + index); 
}

void MapSector::deleteLandmarkAt(int index) { 
	landmarks->erase(landmarks->begin() + index); 
}	

size_t MapSector::sitesSize() { 
	return sites->size(); 
}

size_t MapSector::featuresSize() { 
	return features->size(); 
}

size_t MapSector::landmarksSize() { 
	return landmarks->size(); 
}

void MapSector::deleteAllSites() {  
	for (int i = 0; i < sites->size(); i++) {
        delete sites->at(i);
    }
    sites->clear();
    RNUtils::printLn("Deleted all sites..");
}

void MapSector::deleteAllFeatures(){
	for (int i = 0; i < features->size(); i++) {
        delete features->at(i);
    }
    features->clear();
    RNUtils::printLn("Deleted all features..");
}

void MapSector::deleteAllLandmarks(){
	for (int i = 0; i < landmarks->size(); i++) {
        delete landmarks->at(i);
    }
    landmarks->clear();
    RNUtils::printLn("Deleted all landmarks..");
}

void MapSector::deletePolygon(){
	for (int i = 0; i < polygon->size(); i++) {
        delete polygon->at(i);
    }
    polygon->clear();
    RNUtils::printLn("Deleted all polygon points..");
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
	float px = 0, py = 0;
	PointXY mPoint;
	for(unsigned int idx = 0; idx < values.size(); ){

		if(values.at(idx) == "m" or values.at(idx) == "M"){
			px = std::atof(values.at(idx + 1).c_str()) / 100.0;
			py = std::atof(values.at(idx + 2).c_str()) / 100.0;
			polygon->push_back(new PointXY(px, py));
			mPoint = PointXY(px, py);
			idx += 3;
		} else if(values.at(idx) == "l" or values.at(idx) == "L"){
			px = polygon->at(polygon->size() - 1)->getX() + (std::atof(values.at(idx + 1).c_str()) / 100.0);
			py = polygon->at(polygon->size() - 1)->getY() + (std::atof(values.at(idx + 2).c_str()) / 100.0);
			polygon->push_back(new PointXY(px, py));
			idx += 3;
		} else if(values.at(idx) == "q" or values.at(idx) == "Q"){
			std::vector<PointXY> bezierPointXYs;
			bezierPointXYs.push_back(*polygon->at(polygon->size() - 1));

			px = polygon->at(polygon->size() - 1)->getX() + (std::atof(values.at(idx + 1).c_str()) / 100.0);
			py = polygon->at(polygon->size() - 1)->getY() + (std::atof(values.at(idx + 2).c_str()) / 100.0);
			bezierPointXYs.push_back(PointXY(px, py));

			px = polygon->at(polygon->size() - 1)->getX() + (std::atof(values.at(idx + 3).c_str()) / 100.0);
			py = polygon->at(polygon->size() - 1)->getY() + (std::atof(values.at(idx + 4).c_str()) / 100.0);
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
}

float MapSector::getAngle(PointXY a, PointXY b){
	float result = 0;
	result = std::atan2(b.getY(), b.getX()) - std::atan2(a.getY(), a.getX());
	while(result > M_PI){
		result = result - 2 * M_PI;
	} 

	while(result < -M_PI) {
		result = result + 2 * M_PI;
	}

	return result;
}

bool MapSector::checkPointXYInPolygon(PointXY g, float &angle){
	PointXY a;
	PointXY b;
	bool result = false;
	for(unsigned int i = 0; i < polygon->size() - 1; i++){
		a.setX(polygon->at(i)->getX() - g.getX());
		a.setY(polygon->at(i)->getY() - g.getY());
		b.setX(polygon->at(i + 1)->getX() - g.getX());
		b.setY(polygon->at(i + 1)->getY() - g.getY());

		angle += getAngle(a, b);
	}

	if((std::abs(angle) <= (2 * M_PI)) and (std::abs(angle) > M_PI)){
		result = true;
	}
	return result;
}
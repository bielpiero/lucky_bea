#ifndef MAP_SECTOR_H
#define MAP_SECTOR_H

#include "RNUtils.h"
#include "semdefs.h"

struct s_landmark{
public:
	int id;
	std::string type;
	double varMinX;
	double varMaxX;
	double varMinY;
	double varMaxY;
	double varMinZ;
	double varMaxZ;
	double xpos;
	double ypos;
	double zpos;
	s_landmark(){
		id = RN_NONE;
		type = "";
		varMinX = 0;
		varMaxX = 0;
		varMinY = 0;
		varMaxY = 0;
		varMinZ = 0;
		varMaxZ = 0;
		xpos = 0;
		ypos = 0;
		zpos = 0;
	}
	s_landmark(const s_landmark &lmrk){
		id = lmrk.id;
		type = lmrk.type;
		varMinX = lmrk.varMinX;
		varMaxX = lmrk.varMaxX;
		varMinY = lmrk.varMinY;
		varMaxY = lmrk.varMaxY;
		varMinZ = lmrk.varMinZ;
		varMaxZ = lmrk.varMaxZ;
		xpos = lmrk.xpos;
		ypos = lmrk.ypos;
		zpos = lmrk.zpos;
	}
};

struct s_feature{
public:
	int id;
	std::string name;
	double width;
	double height;
	double xpos;
	double ypos;
	s_feature(){
		id = RN_NONE;
		name = "";
		width = 0.0;
		height = 0.0;
		xpos = 0.0;
		ypos = 0.0;
	}

	s_feature(const s_feature& ftr){
		id = ftr.id;
		name = ftr.name;
		width = ftr.width;
		height = ftr.height;
		xpos = ftr.xpos;
		ypos = ftr.ypos;
	}
};

struct s_way{
public:
	int st;
	std::vector<int> adjacencies;
	s_way(){
		st = RN_NONE;
		adjacencies = std::vector<int>();
	}

	s_way(const s_way& w){
		st = w.st;
		std::vector<int>::const_iterator ait = w.adjacencies.begin();
		while(ait != w.adjacencies.end()){
			adjacencies.push_back(*ait);
			ait++;
		}
	}
};

struct s_site{
public:
	int id;
	std::string name;
	double xpos;
	double ypos;
	int linkedSectorId;
    double xcoord;
    double ycoord;

    s_site(){
    	id = RN_NONE;
    	name = "";
    	xpos = 0.0;
    	ypos = 0.0;
    	linkedSectorId = RN_NONE;
    	xcoord = 0.0;
    	ycoord = 0.0;
    }

    s_site(const s_site& st){
    	id = st.id;
    	name = st.name;
    	xpos = st.xpos;
    	ypos = st.ypos;
    	linkedSectorId = st.linkedSectorId;
    	xcoord = st.xcoord;
    	ycoord = st.ycoord;
    }
};

struct s_tag{
	std::string id;
	int antenna;
	std::string name;
	int linkedSiteId;
	s_tag(){
		id = "";
		antenna = RN_NONE;
		linkedSiteId = RN_NONE;
		name = "";
	}

	s_tag(const s_tag& tg){
		id = tg.id;
		antenna = tg.antenna;
		linkedSiteId = tg.linkedSiteId;
		name = tg.name;
	}
};

class MapSector{
private:
	int mapId;
	int id;
	std::string name;
	double width;
	double height;
	std::string polygonDefinition;
	std::vector<PointXY*> *polygon;
	bool sitesCyclic;
	std::string sequence;
	std::list<s_landmark*> *landmarks;
	std::list<s_feature*> *features;
	std::list<s_site*> *sites;
	std::list<s_way*> *ways;
	std::list<s_tag*> *tags;
private:
	void getPolygonFromString();
	double getAngle(PointXY a, PointXY b);

	static bool polygonSorter(PointXY* a, PointXY* b);
public:
	MapSector();
	~MapSector();

	MapSector* clone();

	void setId(int id);
	int getId();

	void setMapId(int id);
	int getMapId();

	bool isHallway(void);

	void setName(std::string name);
	std::string getName();

	void setPolygon(std::string polygon);
	std::string getPolygonDefinition();

	void setWidth(double width);
	double getWidth();

	void setHeight(double height);
	double getHeight();

	void setIfSitesCyclic(bool sitesCyclic);
	double isSitesCyclic();

	void setSequence(std::string sequence);
	std::string getSequence();

	void addSite(s_site* site);
	void addWay(s_way* way);
	void addFeature(s_feature* feature);
	void addLandmark(s_landmark* landmark);
	void addTag(s_tag* tag);

	s_site* siteAt(int index);
	s_way* wayAt(int index);
	s_feature* featureAt(int index);
	s_landmark* landmarkAt(int index);
	s_tag* tagAt(int index);

	s_landmark* landmarkByTypeAndId(std::string type, int id);

	s_site* findSiteById(int id);
	s_feature* findFeatureById(int id);
	s_tag* findTagById(std::string id);

	std::list<s_site*> findSitesByName(std::string name);
	std::list<s_feature*> findFeaturesByName(std::string name);

	void deleteSite(s_site* obj);
	void deleteWay(s_way* obj);
	void deleteFeature(s_feature* obj);
	void deleteLandmark(s_landmark* obj);
	void deleteTag(s_tag* obj);

	void deleteSiteAt(int index);
	void deleteWayAt(int index);
	void deleteFeatureAt(int index);
	void deleteLandmarkAt(int index);
	void deleteTagAt(int index);

	size_t sitesSize();
	size_t waysSize();
	size_t featuresSize();
	size_t landmarksSize();
	size_t tagsSize();

	size_t landmarksSizeByType(std::string type);
	
	void deleteAllSites();
	void deleteAllWays();
    void deleteAllFeatures();
    void deleteAllLandmarks();
    void deleteAllTags();
    void deletePolygon();

    bool checkPointXYInPolygon(PointXY g);

};

#endif
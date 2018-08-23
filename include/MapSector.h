#ifndef MAP_SECTOR_H
#define MAP_SECTOR_H

#include "RNUtils.h"
#include "semdefs.h"

struct s_landmark{
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
};

struct s_feature{
	int id;
	std::string name;
	double width;
	double height;
	double xpos;
	double ypos;
};

struct s_way{
	int st;
	std::vector<int> adjacencies;
	s_way(){
		st = RN_NONE;
		adjacencies = std::vector<int>();
	}
};

struct s_site{
	int id;
	std::string name;
	double tsec;
	double radius;
	double xpos;
	double ypos;
	int linkedSectorId;
    double xcoord;
    double ycoord;
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
	std::vector<s_landmark*> *landmarks;
	std::vector<s_feature*> *features;
	std::vector<s_site*> *sites;
	std::vector<s_way*> *ways;
private:
	void getPolygonFromString();
	double getAngle(PointXY a, PointXY b);

	static bool polygonSorter(PointXY* a, PointXY* b);
public:
	MapSector();
	~MapSector();

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

	s_site* siteAt(int index);
	s_way* wayAt(int index);
	s_feature* featureAt(int index);
	s_landmark* landmarkAt(int index);

	s_landmark* landmarkByTypeAndId(std::string type, int id);

	s_site* findSiteById(int id);
	s_feature* findFeatureById(int id);

	std::vector<s_site*> findSitesByName(std::string name);
	std::vector<s_feature*> findFeaturesByName(std::string name);

	void deleteSite(s_site* obj);
	void deleteWay(s_way* obj);
	void deleteFeature(s_feature* obj);
	void deleteLandmark(s_landmark* obj);

	void deleteSiteAt(int index);
	void deleteWayAt(int index);
	void deleteFeatureAt(int index);
	void deleteLandmarkAt(int index);

	size_t sitesSize();
	size_t waysSize();
	size_t featuresSize();
	size_t landmarksSize();

	size_t landmarksSizeByType(std::string type);
	
	void deleteAllSites();
	void deleteAllWays();
    void deleteAllFeatures();
    void deleteAllLandmarks();
    void deletePolygon();

    bool checkPointXYInPolygon(PointXY g);

};

#endif
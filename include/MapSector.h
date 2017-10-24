#ifndef MAP_SECTOR_H
#define MAP_SECTOR_H

#include "RNUtils.h"

struct s_landmark{
	int id;
	std::string type;
	float varMinX;
	float varMaxX;
	float varMinY;
	float varMaxY;
	float varMinZ;
	float varMaxZ;
	float xpos;
	float ypos;
	float zpos;
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
	float width;
	float height;
	float xpos;
	float ypos;
	int linkedSectorId;
    float xcoord;
    float ycoord;
};

struct s_site{
	int id;
	std::string name;
	float tsec;
	float radius;
	float xpos;
	float ypos;
	int linkedFeatureId;
};

class MapSector{
private:
	int id;
	std::string name;
	float width;
	float height;
	std::string polygonDefinition;
	std::vector<PointXY*> *polygon;
	bool sitesCyclic;
	std::string sequence;
	std::vector<s_landmark*> *landmarks;
	std::vector<s_feature*> *features;
	std::vector<s_site*> *sites;
private:
	void getPolygonFromString();
	float getAngle(PointXY a, PointXY b);
public:
	MapSector();
	~MapSector();

	void setId(int id);
	float getId();

	void setName(std::string name);
	std::string getName();

	void setPolygon(std::string polygon);
	std::string getPolygonDefinition();

	void setWidth(float width);
	float getWidth();

	void setHeight(float height);
	float getHeight();

	void setIfSitesCyclic(bool sitesCyclic);
	float isSitesCyclic();

	void setSequence(std::string sequence);
	std::string getSequence();

	void addSite(s_site* site);
	void addFeature(s_feature* feature);
	void addLandmark(s_landmark* landmark);

	s_site* siteAt(int index);
	s_feature* featureAt(int index);
	s_landmark* landmarkAt(int index);

	s_site* findSiteById(int id);
	s_feature* findFeatureById(int id);

	std::vector<s_site*> findSitesByName(std::string name);
	std::vector<s_feature*> findFeaturesByName(std::string name);

	void deleteSite(s_site* obj);
	void deleteFeature(s_feature* obj);
	void deleteLandmark(s_landmark* obj);

	void deleteSiteAt(int index);
	void deleteFeatureAt(int index);
	void deleteLandmarkAt(int index);

	size_t sitesSize();
	size_t featuresSize();
	size_t landmarksSize();

	size_t landmarksSizeByType(std::string type);
	
	void deleteAllSites();
    void deleteAllFeatures();
    void deleteAllLandmarks();
    void deletePolygon();

    bool checkPointXYInPolygon(PointXY g, float &angle);

};

#endif
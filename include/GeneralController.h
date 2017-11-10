#ifndef GENERAL_CONTROLLER_H
#define GENERAL_CONTROLLER_H

#include "RobotNode.h"
#include "RNLandmarkList.h"

#include "SocketNode2.h"
#include "SerialPort.h"
#include "UDPClient.h"
#include "RNSocketClient.h"
#include "xml/rapidxml_print.hpp"
#include "xml/rapidxml.hpp"

#include "semdefs.h"

#include "RNVirtualFace.h"
#include "RobotDataStreamer.h"
#include "DorisLipSync.h"
#include "MapSector.h"

#include "Matrix.h"
#include "stats.h"
#include "fl/Headers.h"
#include "fl/fuzzyStats.h"

// Task headers

#define CONVERTER_BUFFER_SIZE 20
#define SERVOS_COUNT 16

#define PACKAGE_NAME "lucky_bea"

#define XML_FILE_PATH "conf/BeaConSuerte.xml"
#define XML_FILE_MAPS_PATH "conf/BeaMaps.xml"
#define XML_FILE_SECTORS_PATH "maps/"
#define XML_FILE_ROBOT_CONFIG_PATH "conf/BeaRobotConfig.xml"

#define PERMISSION_ACCEPTED 0
#define PERMISSION_REQUESTED 1
#define PERMISSION_REJECTED 2

#define X_INDEX 0
#define V_INDEX 3
#define W_INDEX 5

#define TRAP_VERTEX 4

#define ADD_SITE_VARIABLE_LENGTH 7
#define MODIFY_SITE_VARIABLE_LENGTH 8
#define DELETE_SITE_VARIABLE_LENGTH 3

#define SITE_SEQUENCE_VARIABLE_LENGTH 2

#define ADD_FEATURE_VARIABLE_LENGTH 7
#define MODIFY_FEATURE_VARIABLE_LENGTH 8
#define DELETE_FEATURE_VARIABLE_LENGTH 3

using namespace rapidxml;

class GeneralController;
class RNLaserTask;
class RNGlobalLocalizationTask;
class RNLocalizationTask;
class RNKalmanLocalizationTask;
class RNPFLocalizationTask;
class RNOmnicameraTask;
class RNCameraTask;
class RNRecurrentTaskMap;
class RNRFIdentificationTask;
class RNGesturesTask;
class RNDialogsTask;
class RNEmotionsTask;

struct s_motor{
	std::string idMotor;
    std::string cardId;
	std::string pos;
	std::string speed;
	std::string acceleration;
};

struct s_secuence{
	std::string idSecuence;
	std::string tsec;
	std::vector<s_motor> motors;
};

struct s_movement{
	std::string idGesture;
	std::string ts;
	std::vector<s_secuence> secuences;
	
};

struct dynamic_face_info{
	std::string id_gesto;
	GeneralController* object;
};


struct s_trapezoid{
	float x1;
	float x2;
	float x3;
	float x4;
	s_trapezoid(){
		x1 = 0;
		x2 = 0;
		x3 = 0;
		x4 = 0;
	}
};

struct s_position{
	s_trapezoid* xZone;
	s_trapezoid* yZone;
	s_trapezoid* thZone;
	s_position(){
		xZone = new s_trapezoid();
		yZone = new s_trapezoid();
		thZone = new s_trapezoid();		
	}
	~s_position(){
		delete xZone;
		delete yZone;
		delete thZone;
	}
};

struct s_obs_dth{
	s_trapezoid* dZone;
	s_trapezoid* thZone;

	s_obs_dth(){
		dZone = new s_trapezoid();
		thZone = new s_trapezoid();		
	}
	~s_obs_dth(){
		delete dZone;
		delete thZone;
	}
};

struct s_sensor{
	std::string type;
	bool activated;
	s_obs_dth* observationNoise;
	s_sensor(){
		activated = false;
		type = "";
		observationNoise = new s_obs_dth();
	}

	~s_sensor(){
		delete observationNoise;
	}
};

struct s_navigation_params{
	float alpha;
	s_position* initialPosition;
	s_obs_dth* processNoise;
	std::vector<s_sensor*>* sensors;
	s_navigation_params(){
		initialPosition = new s_position();
		processNoise = new s_obs_dth();
		sensors = new std::vector<s_sensor*>();
	}

	~s_navigation_params(){
		for(int i = 0; i < sensors->size(); i++){
			delete sensors->at(i);
		}
		sensors->clear();
		delete sensors;
		delete initialPosition;
		delete processNoise;
	}
};

struct s_robot{
	float height;
	std::string localization;
	s_navigation_params* navParams;
	s_robot(){
		height = 0.0;
		localization = "kalman";
		navParams = new s_navigation_params();
	}

	~s_robot(){
		delete navParams;
	}
};

class GeneralController : public CSocketNode, public RobotNode // la clase GeneralController hereda de la clase CSocketNode
{
private: //variables emotions
	SerialPort* maestroControllers;
	DorisLipSync* ttsLipSync;

	bool continue_dynamic_thread;
	bool pendingTransferControl;
	unsigned int clientsConnected;
	
	std::string xmlFaceFullPath;
	std::string xmlMapsFullPath;
	std::string xmlSectorsPath;
	std::string xmlRobotConfigFullPath;
	
	std::ostringstream emotionsTimestamp;
	std::ostringstream mappingSectorTimestamp;
	std::ostringstream mappingLandmarksTimestamp;
	std::ostringstream mappingFeaturesTimestamp;
	std::ostringstream mappingSitesTimestamp;
	std::FILE* file;
protected: // functions
	
public: // emotions functions
	
	GeneralController(const char* port);
	~GeneralController(void);
	
	virtual void onConnection(int socketIndex);//callback for client and server
	virtual void onMsg(int socketIndex, char* cad, unsigned long long int length);//callback for client and server
	virtual const char* getClassName() const;
	void stopDynamicGesture();
private:

	bool isPermissionNeeded(char function);
	void requestRobotControl(int socketIndex);
	void releaseRobotControl(int socketIndex);

	void acceptTransferRobotControl(int socketIndex, char* acceptance);

	void getPololuInstruction(char* cad, unsigned char& card_id, unsigned char& servo_id, int& value);
	void getGestures(std::string type, std::string& gestures);
	void setGesture(std::string id, std::string& servo_positions);
	void saveGesture(std::string token, int gesture_type);
	void saveStaticGesture(std::string name, s_motor servos[]);
	void saveDynamicGesture(std::string name, s_motor servos[]);
	
	void modifyGesture(std::string token, int gesture_type);
	void modifyStaticGesture(std::string face_id, std::string name, s_motor servos[]);
	
	void removeGesture(std::string id);
	
	void setServoPosition(unsigned char card_id, unsigned char servo_id, int position);
	void setServoSpeed(unsigned char card_id, unsigned char servo_id, int speed);
	void setServoAcceleration(unsigned char card_id, unsigned char servo_id, int speed);
	
	static void* dynamicFaceThread(void*);

public:
	void onBumpersUpdate(std::vector<bool> front, std::vector<bool> rear);
	void onPositionUpdate(double x, double y, double theta, double transSpeed, double rotSpeed);
	void onRawPositionUpdate(double x, double y, double theta, double deltaDistance, double deltaDegrees);
	void onBatteryChargeStateChanged(char battery);
	void onSonarsDataUpdate(std::vector<PointXY*>* data);
	void onSecurityDistanceWarningSignal();
    void onSecurityDistanceStopSignal();
    void onSensorsScanCompleted();
	
	int initializeKalmanVariables();
	void loadSector(int mapId, int sectorId);
	
	void stopCurrentTour();

	int getCurrenMapId();
	int getCurrentSectorId();
	int getNextSectorId();
	int getLastVisitedNode();

	PointXY getNextSectorCoord();

	void setLastVisitedNode(int id);
	void setNextSectorId(int id);

	int lockLaserLandmarks();
	int unlockLaserLandmarks();

	int lockRFIDLandmarks();
	int unlockRFIDLandmarks();

	int lockVisualLandmarks();
	int unlockVisualLandmarks();

	Matrix getP();
	Matrix getQ();
	Matrix getR();

	float getLaserDistanceVariance();
	float getLaserAngleVariance();
	float getCameraDistanceVariance();
	float getCameraAngleVariance();

	float getRobotHeight();

	bool isLaserSensorActivated();
	bool isCameraSensorActivated();
	bool isRfidSensorActivated();

	Matrix getRawEncoderPosition();
	Matrix getRawDeltaPosition();

	MapSector* getCurrentSector();
	RNLandmarkList* getLaserLandmarks();
	RNLandmarkList* getVisualLandmarks();
	RNLandmarkList* getRFIDLandmarks();

	void setLaserLandmarks(RNLandmarkList* landmarks);
	void setVisualLandmarks(RNLandmarkList* landmarks);
	void setRFIDLandmarks(RNLandmarkList* landmarks);
private:
	

	UDPClient* spdUDPClient;
	RobotDataStreamer* spdWSServer;
	// Tasks Objects
	RNLaserTask* laserTask;
	RNEmotionsTask* emotions;
	RNGesturesTask* gestures;
	RNDialogsTask* dialogs;
	RNRecurrentTaskMap* tasks;
	RNLocalizationTask* localization;
	RNGlobalLocalizationTask* globalLocalization;
	RNOmnicameraTask* omnidirectionalTask;
	RNRFIdentificationTask* rfidTask; //future
	RNCameraTask* eyesCameras;

	RNVirtualFace* virtualFace;

	//possibilistic navigation
	Matrix robotVelocity;
	Matrix robotEncoderPosition;

	Matrix robotRawDeltaPosition;
	Matrix robotRawEncoderPosition;

	std::vector<fl::Trapezoid*>* kalmanFuzzy;
	Matrix robotState;
	Matrix P;
	Matrix Q;
	Matrix R;

	float laserDistanceVariance;
	float laserAngleVariance;
	float cameraDistanceVariance;
	float cameraAngleVariance;

	int tokenRequester;
    int lastSiteVisitedIndex;
	
	RNLandmarkList* laserLandmarks;
	RNLandmarkList* visualLandmarks;
	RNLandmarkList* rfidLandmarks;
	
	bool setChargerPosition;
	bool hasAchievedGoal;
	bool frontBumpersOk;
	bool rearBumpersOk;
	int spdUDPPort;
	unsigned char streamingActive;
	
	unsigned char keepRobotTracking;
	unsigned char keepTourAlive;

	int currentMapId;
	int nextSectorId;
	bool hallwayDetected;
	PointXY nextSectorCoord;
	MapSector* currentSector;
	s_robot* robotConfig;

	bool laserSensorActivated;
	bool cameraSensorActivated;
	bool rfidSensorActivated;

	bool printed;

	pthread_mutex_t laserLandmarksLocker;
	pthread_mutex_t rfidLandmarksLocker;
	pthread_mutex_t visualLandmarksLocker;

	void getMapId(char* cad, int& mapId);
	
	void loadRobotConfig();
	
	void initializeSPDPort(int socketIndex, char* cad);
	
	void getVelocities(char* cad, float& lin_vel, float& angular_vel);
	void moveRobot(float lin_vel, float angular_vel);

	void setRobotPosition(Matrix pose);
	void setRobotPosition(float x, float y, float theta);
	void moveRobotToPosition(float x, float y, float th);
	void getPositions(char* cad, float& x, float& y, float& theta);

	void getMapSectorId(char* cad, int& mapId, int& sectorId);
	void getMapsAvailable(std::string& mapsAvailable);
	void getMapFilename(int mapId, std::string& filename);
	void getMapConnection(int mapId, std::string& connections);
	void getSectorsAvailable(int mapId, std::string& sectorsAvailable);
	void getSectorInformationLandmarks(int mapId, int sectorId, std::string& sectorInformation);
	void getSectorInformationFeatures(int mapId, int sectorId, std::string& sectorInformation);
	void getSectorInformationSites(int mapId, int sectorId, std::string& sectorInformation);
	void getSectorInformationSitesSequence(int mapId, int sectorId, std::string& sectorInformation);

	//sites functions
	void addSectorInformationSite(char* cad, int& indexAssigned);
	void modifySectorInformationSite(char* cad);
	void deleteSectorInformationSite(char* cad);
	void setSitesExecutionSequence(char* cad);

	//features functions
	void addSectorInformationFeatures(char* cad, int& indexAssigned);
	void modifySectorInformationFeatures(char* cad);
	void deleteSectorInformationFeatures(char* cad);
	
	void startSitesTour();

	bool isFirstQuadrant(float angle);
	bool isSecondQuadrant(float angle);
	bool isThirdQuadrant(float angle);
	bool isFouthQuadrant(float angle);
	
	static void* sitesTourThread(void*);
};

#endif


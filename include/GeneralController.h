#ifndef GENERAL_CONTROLLER_H
#define GENERAL_CONTROLLER_H

#include "RobotNode.h"
#include "RNLandmarkList.h"

#include "DorisLipSync.h"

#include "SocketNode2.h"
#include "SerialPort.h"
#include "UDPClient.h"
#include "RNSocketClient.h"

#include "xmldefs.h"

#include "xml/rapidxml_print.hpp"
#include "xml/rapidxml.hpp"

#include "RNVirtualFace.h"
#include "RobotDataStreamer.h"
#include "MapSector.h"
#include "RNTimer.h"

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

#define PROGRAMS_DIR_PATH "tts/"
#define PROGRAM_EXTENSION "doris"

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
class RNLocalizationTask;
class RNKalmanLocalizationTask;
class RNPKalmanLocalizationTask;
class RNOmnicameraTask;
class RNCameraTask;
class RNRecurrentTaskMap;
class RNRFIdentificationTask;
class RNGesturesTask;
class RNArmTask;
class RNDialogsTask;
class RNEmotionsTask;
class RNTourThread;

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
	double alpha;
	double x1;
	double x2;
	double x3;
	double x4;
	s_trapezoid(){
		alpha = 0;
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
	double height;
	std::string localization;
	int faceId;
	int neckId;
	s_navigation_params* navParams;
	s_robot(){
		faceId = RN_NONE;
		neckId = RN_NONE;
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
	DorisLipSync* tts;

	std::string textInputId;
	std::string emotionState;
	std::string faceIdFromEmotions;

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

	SerialPort* getMaestroController();
	void setTextInputIdToEmotion(std::string textInputId);
	void setEmotionsResult(std::string emotionState, std::string faceIdFromEmotions);
	DorisLipSync* getTTS();
private:

	bool isPermissionNeeded(char function);
	void requestRobotControl(int socketIndex);
	void releaseRobotControl(int socketIndex);

	void acceptTransferRobotControl(int socketIndex, char* acceptance);

	void getPololuInstruction(char* cad, unsigned char& card_id, unsigned char& servo_id, int& value);

	void saveGesture(std::string token, int gesture_type);
	void saveStaticGesture(std::string name, s_motor servos[]);
	void saveDynamicGesture(std::string name, s_motor servos[]);
	
	void modifyGesture(std::string token, int gesture_type);
	void modifyStaticGesture(std::string face_id, std::string name, s_motor servos[]);
	
	void removeGesture(std::string id);
	
	void setServoPosition(unsigned char card_id, unsigned char servo_id, int position);
	void setServoSpeed(unsigned char card_id, unsigned char servo_id, int speed);
	void setServoAcceleration(unsigned char card_id, unsigned char servo_id, int speed);

public:
	void onBumpersUpdate(std::vector<bool> front, std::vector<bool> rear);
	void onPositionUpdate(double x, double y, double theta, double transSpeed, double rotSpeed);
	void onBatteryChargeStateChanged(char battery);
	void onSonarsDataUpdate(std::vector<PointXY*>* data);
	void onSecurityDistanceWarningSignal();
    void onSecurityDistanceStopSignal();
    void onSensorsScanCompleted();
    void onLaserScanCompleted(LaserScan* data);
	
	int initializeKalmanVariables();
	void loadSector(int mapId, int sectorId);
	void getMapFilename(int mapId, std::string& filename);

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

	int lockRawDeltaEncoders();
	int unlockRawDeltaEncoders();

	Matrix getP();
	Matrix getQ();


	double getLaserDistanceAlpha();
	double getLaserAngleAlpha();
	double getCameraDistanceAlpha();
	double getCameraAngleAlpha();

	double getLaserDistanceVariance();
	double getLaserAngleVariance();
	double getCameraDistanceVariance();
	double getCameraAngleVariance();

	double getRobotHeight();
	int getFaceId();
	int getNeckId();

	bool isLaserSensorActivated();
	bool isCameraSensorActivated();
	bool isRfidSensorActivated();

	MapSector* getCurrentSector();
	RNLandmarkList* getLaserLandmarks();
	RNLandmarkList* getVisualLandmarks();
	RNLandmarkList* getRFIDLandmarks();

	void setLaserLandmarks(RNLandmarkList* landmarks);
	void setVisualLandmarks(RNLandmarkList* landmarks);
	void setRFIDLandmarks(RNLandmarkList* landmarks);

	void moveRobotToPosition(double x, double y, double th);
	void setRobotPosition(Matrix pose);
	void setRobotPosition(double x, double y, double theta);
	void moveRobot(double lin_vel, double angular_vel);

	RNRFIdentificationTask* getRfidTask();
	RNEmotionsTask* getEmotionsTask();
	RNLaserTask* getLaserTask();
private:
	
	RobotDataStreamer* spdWSServer;
	// Tasks Objects
	RNLaserTask* laserTask;
	RNEmotionsTask* emotions;
	RNGesturesTask* gestures;
	RNArmTask* armGestures;
	RNDialogsTask* dialogs;
	RNRecurrentTaskMap* tasks;
	RNLocalizationTask* localization;
	RNOmnicameraTask* omnidirectionalTask;
	RNRFIdentificationTask* rfidTask; //future
	RNCameraTask* eyesCameras;
	RNTourThread* tourThread;
	RNVirtualFace* virtualFace;

	//possibilistic navigation
	Matrix robotVelocity;
	Matrix robotEncoderPosition;

	Matrix robotRawDeltaPosition;
	Matrix robotRawEncoderPosition;

	std::vector<fl::Trapezoid*>* kalmanFuzzy;
	
	Matrix P;
	Matrix Q;


	double laserDistanceAlpha;
	double laserAngleAlpha;
	double cameraDistanceAlpha;
	double cameraAngleAlpha;

	double laserDistanceVariance;
	double laserAngleVariance;
	double cameraDistanceVariance;
	double cameraAngleVariance;

	int tokenRequester;
    int lastSiteVisitedIndex;
	
	RNLandmarkList* laserLandmarks;
	RNLandmarkList* visualLandmarks;
	RNLandmarkList* rfidLandmarks;
	
	bool setChargerPosition;
	bool hasAchievedGoal;
	bool frontBumpersOk;
	bool rearBumpersOk;

	unsigned char streamingActive;
	
	unsigned char keepRobotTracking;
	unsigned char keepTourAlive;

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
	pthread_mutex_t rawPositionLocker;
	pthread_mutex_t currentSectorLocker;

	void getMapId(char* cad, int& mapId);
	
	void loadRobotConfig();
	void getInitialLocation();
	
	void initializeSPDPort(int socketIndex, char* cad);
	
	void getVelocities(char* cad, double& lin_vel, double& angular_vel);
	
	void getPositions(char* cad, double& x, double& y, double& theta);

	void getMapSectorId(char* cad, int& mapId, int& sectorId);
	void getArmSingleMotorInfo(char* cad, int& id, int& angle);
	void getArmAllMotorsInfo(char* cad, std::vector<uint16_t>& motors);
	void getMapsAvailable(std::string& mapsAvailable);

	void getAvailablePrograms(std::string& availablePrograms);
	
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
	

	bool isFirstQuadrant(double angle);
	bool isSecondQuadrant(double angle);
	bool isThirdQuadrant(double angle);
	bool isFouthQuadrant(double angle);
	
};

#endif


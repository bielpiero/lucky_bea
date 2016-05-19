#ifndef GENERAL_CONTROLLER_H
#define GENERAL_CONTROLLER_H

#include <opencv2/opencv.hpp>

#include "RobotNode.h"
#include "SocketNode2.h"
#include "SerialPort.h"
#include "UDPClient.h"
#include "xml/rapidxml_print.hpp"
#include "xml/rapidxml.hpp"
#include "xmldefs.h"
#include "semdefs.h"

#include "RobotDataStreamer.h"
#include "DorisLipSync.h"
#include "MapSector.h"

#include "Matrix.h"
#include "stats.h"
#include "fl/fuzzy.h"

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

#define STATE_VARIABLES 3

#define MAX_LANDMAKS 20;

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

struct s_video_streamer_data{
	int socketIndex;
	int port;
	GeneralController* object;	
};

struct s_trapezoid{
	float x1;
	float x2;
	float x3;
	float x4;
};

struct s_position{
	s_trapezoid* xZone;
	s_trapezoid* yZone;
	s_trapezoid* thZone;
};

struct s_obs_dth{
	s_trapezoid* dZone;
	s_trapezoid* thZone;
};

struct s_navigation_params{
	float alpha;
	s_position* initialPosition;
	s_obs_dth* processNoise;
	s_obs_dth* observationNoise;
};

struct s_robot{
	s_navigation_params* navParams;
};

class RNLandmark{
private:
	std::vector<PointXY*>* data;
	float meanX;
	float meanY;
public:
	RNLandmark(){ 
		data = new std::vector<PointXY*>(); 
		meanX = std::numeric_limits<float>::infinity();
		meanY = std::numeric_limits<float>::infinity();
	}

	~RNLandmark() {
		for (int i = 0; i < data->size(); i++) {
            delete data->at(i);
        }
		data->clear();
		delete data; 
	}
	void addPoint(float x, float y){
		meanX = std::numeric_limits<float>::infinity();
		meanY = std::numeric_limits<float>::infinity();
		data->push_back(new PointXY(x, y));
	}
	PointXY* getPointAt(int index){
		return data->at(index);
	}
	float getPointsXMean(){
		if(meanX == std::numeric_limits<float>::infinity()){
			float sum = 0;
			for (int i = 0; i < data->size(); i++) {
	            sum = sum + data->at(i)->getX();
	        }
	        meanX = (sum / (float)data->size());
	    }
	    return meanX;
	}

	float getPointsYMean(){
		if(meanY == std::numeric_limits<float>::infinity()){
			float sum = 0;
			for (int i = 0; i < data->size(); i++) {
	            sum = sum + data->at(i)->getY();
	        }
	        meanY = (sum / (float)data->size());
	    }
	    return meanY;
	}

	void setPointsXMean(float mean){
		this->meanX = mean;
	}
	void setPointsYMean(float mean){
		this->meanY = mean;
	}

	size_t size(){
		return data->size();
	}
};


class GeneralController : public CSocketNode, public RobotNode // la clase GeneralController hereda de la clase CSocketNode
{
private:
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
	std::ostringstream mappingEnvironmentTimestamp;
	std::ostringstream mappingLandmarksTimestamp;
	std::ostringstream mappingFeaturesTimestamp;
	std::ostringstream mappingSitesTimestamp;
		
public:
	//GeneralController(ros::NodeHandle nh_, const char* port);
	GeneralController(const char* port);
	~GeneralController(void);
	
	virtual void onConnection(int socketIndex);//callback for client and server
	virtual void onMsg(int socketIndex, char* cad, unsigned long long int length);//callback for client and server
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
	
	/// ROS Functions
public:

	void onBumpersUpdate(std::vector<bool> front, std::vector<bool> rear);
	void onPositionUpdate(double x, double y, double theta, double transSpeed, double rotSpeed);
	void onRawPositionUpdate(double x, double y, double theta, double deltaDistance, double deltaDegrees);
	void onBatteryChargeStateChanged(char battery);
	void onSonarsDataUpdate(std::vector<PointXY*>* data);
	void onLaserScanCompleted(LaserScan* laser);
	void onSecurityDistanceWarningSignal();
    void onSecurityDistanceStopSignal();
    void onSensorsScanCompleted();
	
	void initializeKalmanVariables();
	
	void stopVideoStreaming();
	void stopRobotTracking();
	void stopCurrentTour();
	void trackRobot();
private:
	static const float LASER_MAX_RANGE;
	static const float LANDMARK_RADIUS;

	UDPClient* spdUDPClient;
	RobotDataStreamer* spdWSServer;
	

	//OpenCV
	cv::VideoCapture vc;
	cv::VideoCapture vcSecond;
	//possibilistic navigation
	Matrix robotVelocity;
	Matrix robotEncoderPosition;

	Matrix robotRawDeltaPosition;
	Matrix robotRawEncoderPosition;

	std::vector<fuzzy::trapezoid*>* kalmanFuzzy;
	Matrix robotState;
	Matrix P;
	Matrix Q;
	Matrix R;

	int tokenRequester;
    int lastSiteVisitedIndex;
	
	std::vector<RNLandmark*>* landmarks;
	
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
    float nXCoord;
    float nYCoord;
	MapSector* currentSector;
	s_robot* robotConfig;

	pthread_t trackThread;

	pthread_mutex_t mutexLandmarkLocker;

	void getMapId(char* cad, int& mapId);

	void loadSector(int mapId, int sectorId);
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
	void landmarkObservation(Matrix Xk, s_landmark* landmark, float& distance, float& angle);
	void getObservationsTrapezoids(std::vector<fuzzy::trapezoid*> &obsWithNoise, std::vector<fuzzy::trapezoid*> &obsWONoise);
	void getObservations(Matrix &observations);
	Matrix normalizeAngles(Matrix trap);
	Matrix denormalizeAngles(Matrix trap, int mode = 0);
	Matrix multTrapMatrix(Matrix mat, Matrix trap);
	bool isFirstQuadrant(float angle);
	bool isSecondQuadrant(float angle);
	bool isThirdQuadrant(float angle);
	bool isFouthQuadrant(float angle);

	void getCameraDevicePort(char* cad, int& device, int& port);
	void beginVideoStreaming(int socketIndex, int videoDevice, int port);
	
	static void* streamingThread(void*);
	static void* trackRobotThread(void*);
	static void* trackRobotProbabilisticThread(void*);
	static void* sitesTourThread(void*);
};

#endif


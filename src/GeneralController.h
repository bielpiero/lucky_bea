#ifndef GENERAL_CONTROLLER_H
#define GENERAL_CONTROLLER_H

#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <cmath>
#include <string>
#include <cstring>
#include <ctime>
#include "SocketNode2.h"
#include "SerialPort.h"
#include "stdxros.hpp"
#include "UDPClient.h"
#include "xml/rapidxml_print.hpp"
#include "xml/rapidxml.hpp"
#include "xmldefs.h"
#include "semdefs.h"
#include "RobotNode.h"
#include "RobotDataStreamer.h"
#include "DorisLipSync.h"

#include "Matrix.h"
#include "stats.h"
#include "fl/fuzzy.h"

#define CONVERTER_BUFFER_SIZE 20
#define SERVOS_COUNT 16

#define PACKAGE_NAME "lucky_bea"

#define XML_FILE_PATH "/src/conf/BeaConSuerte.xml"
#define XML_FILE_SECTORS_PATH "/src/conf/BeaSectors.xml"
#define XML_FILE_ROBOT_CONFIG_PATH "/src/conf/BeaRobotConfig.xml"

#define NONE -1

#define PERMISSION_ACCEPTED 0
#define PERMISSION_REQUESTED 1
#define PERMISSION_REJECTED 2


#define YES 1
#define NO 0
#define MAYBE 2

#define STATE_VARIABLES 3

#define MAX_LANDMAKS 20;

#define X_INDEX 0
#define V_INDEX 3
#define W_INDEX 5

#define TRAP_VERTEX 4

#define ADD_SITE_VARIABLE_LENGTH 6
#define MODIFY_SITE_VARIABLE_LENGTH 7
#define DELETE_SITE_VARIABLE_LENGTH 2

#define SITE_SEQUENCE_VARIABLE_LENGTH 2

#define ADD_FEATURE_VARIABLE_LENGTH 6
#define MODIFY_FEATURE_VARIABLE_LENGTH 7
#define DELETE_FEATURE_VARIABLE_LENGTH 2

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

struct s_landmark{
	int id;
	float varMinX;
	float varMaxX;
	float varMinY;
	float varMaxY;
	float xpos;
	float ypos;
};

struct s_feature{
	int id;
	std::string name;
	float width;
	float height;
	float xpos;
	float ypos;
};

struct s_site{
	int id;
	std::string name;
	float tsec;
	float radius;
	float xpos;
	float ypos;
};

struct s_sector{
	int id;
	std::string name;
	float width;
	float height;
	bool sitesCyclic;
	std::string sequence;
	std::vector<s_landmark*> *landmarks;
	std::vector<s_feature*> *features;
	std::vector<s_site*> *sites;
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
	std::string xmlSectorsFullPath;
	std::string xmlRobotConfigFullPath;
	
	std::ostringstream emotionsTimestamp;
	std::ostringstream mappingEnvironmentTimestamp;
	std::ostringstream mappingLandmarksTimestamp;
	std::ostringstream mappingFeaturesTimestamp;
	std::ostringstream mappingSitesTimestamp;
		
public:
	GeneralController(ros::NodeHandle nh_, const char* port);
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
	
	void initializeKalmanVariables();
	
	void stopVideoStreaming();
	void stopRobotTracking();
	void stopCurrentTour();
	void trackRobot();
private:
	static const float LASER_MAX_RANGE;

	UDPClient* spdUDPClient;
	RobotDataStreamer* spdWSServer;
	ros::NodeHandle nh;

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
	
	std::vector<Matrix> landmarks;
	
	bool setChargerPosition;
	bool hasAchievedGoal;
	bool frontBumpersOk;
	bool rearBumpersOk;
	int spdUDPPort;
	unsigned char streamingActive;
	
	unsigned char keepRobotTracking;
	unsigned char keepTourAlive;
	std::vector<s_sector*>* navSectors;
	s_sector* currentSector;
	s_robot* robotConfig;

	pthread_t trackThread;

	pthread_mutex_t mutexLandmarkLocker;

	void loadSectors();
	void loadSector(int sectorId);
	void loadRobotConfig();
	
	void initializeSPDPort(int socketIndex, char* cad);
	
	void getVelocities(char* cad, float& lin_vel, float& angular_vel);
	void moveRobot(float lin_vel, float angular_vel);

	void setRobotPosition(Matrix pose);
	void setRobotPosition(float x, float y, float theta);
	void moveRobotToPosition(float x, float y, float th);
	void getPositions(char* cad, float& x, float& y, float& theta);

	void getMapId(char* cad, int& mapId);
	void getMapsAvailable(std::string& mapsAvailable);
	void getMapInformationLandmarks(int mapId, std::string& mapInformation);
	void getMapInformationFeatures(int mapId, std::string& mapInformation);
	void getMapInformationSites(int mapId, std::string& mapInformation);
	void getMapInformationSitesSequence(int mapId, std::string& mapInformation);

	//sites functions
	void addMapInformationSite(char* cad, int& indexAssigned);
	void modifyMapInformationSite(char* cad);
	void deleteMapInformationSite(char* cad);
	void setSitesExecutionSequence(char* cad);

	//features functions
	void addMapInformationFeatures(char* cad, int& indexAssigned);
	void modifyMapInformationFeatures(char* cad);
	void deleteMapInformationFeatures(char* cad);
	
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

	void getTimestamp(std::ostringstream& timestamp);
	
	static void* streamingThread(void*);
	static void* trackRobotThread(void*);
	static void* trackRobotProbabilisticThread(void*);
	static void* sitesTourThread(void*);

	static void* serverStatusThread(void*);
	
};

#endif


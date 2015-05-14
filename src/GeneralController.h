#ifndef GENERAL_CONTROLLER_H
#define GENERAL_CONTROLLER_H

#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <cmath>
#include <string>
#include <cstring>
#include "SocketNode2.h"
#include "SerialPort.h"
#include "stdxros.hpp"
#include "UDPClient.h"
#include "xml/rapidxml_print.hpp"
#include "xml/rapidxml.hpp"
#include "xmldefs.h"
#include "semdefs.h"

#include "Matrix.h"
#include "stats.h"
#include "fl/fuzzy.h"

#define CONVERTER_BUFFER_SIZE 20
#define SERVOS_COUNT 16

#define PACKAGE_NAME "lucky_bea"

#define XML_FILE_PATH "/src/conf/BeaConSuerte.xml"
#define XML_FILE_SECTORS_PATH "/src/conf/BeaSectors.xml"
#define XML_FILE_ROBOT_CONFIG_PATH "/src/conf/BeaRobotConfig.xml"

#define YES 1
#define NO 0
#define MAYBE 2

#define STATE_VARIABLES 3

#define MAX_LANDMAKS 20;

#define X_INDEX 0
#define V_INDEX 3
#define W_INDEX 6

#define TRAP_VERTEX 4

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
	s_position* processNoise;
	s_obs_dth* observationNoise;
};

struct s_robot{
	s_navigation_params* navParams;
};

struct s_landmark{
	int id;
	float var;
	float xpos;
	float ypos;
};

struct s_feature{
	int id;
	std::string name;
	float var;
	float xpos;
	float ypos;
};

struct s_site{
	int id;
	std::string name;
	float tsec;
	float var;
	float xpos;
	float ypos;
};

struct s_sector{
	int id;
	std::string name;
	float width;
	float height;
	bool sitesCyclic;
	std::vector<s_landmark*> *landmarks;
	std::vector<s_feature*> *features;
	std::vector<s_site*> *sites;
};


class GeneralController : public CSocketNode // la clase GeneralController hereda de la clase CSocketNode
{
private:
	SerialPort* maestroControllers;
	bool continue_dynamic_thread;
	
	std::string xmlFaceFullPath;
	std::string xmlSectorsFullPath;
	std::string xmlRobotConfigFullPath;
		
public:
	GeneralController(ros::NodeHandle nh_);
	~GeneralController(void);
	
	virtual void OnConnection();//callback for client and server
	virtual void OnMsg(char* cad,int length);//callback for client and server
	void stopDynamicGesture();
private:
	void getPololuInstruction(char* cad, unsigned char& card_id, unsigned char& servo_id, int& value);
	void getGestures(std::string type, std::string& gestures);
	void setGesture(std::string id);
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
	//void batteryStateCallback(const std_msgs::Float32::ConstPtr& battery); // when available
	void bumperStateCallback(const rosaria::BumperState::ConstPtr& bumpers);
	void poseStateCallback(const nav_msgs::Odometry::ConstPtr& pose);
	void batteryVoltageCallback(const std_msgs::Float64::ConstPtr& battery);
	void batteryRechargeStateCallback(const std_msgs::Int8::ConstPtr& battery);
	
	void sonarStateCallback(const sensor_msgs::PointCloud::ConstPtr& sonar);
	void sonarPointCloud2StateCallback(const sensor_msgs::PointCloud2::ConstPtr& sonar);
	
	void laserScanStateCallback(const sensor_msgs::LaserScan::ConstPtr& laser);
	void laserPointCloudStateCallback(const sensor_msgs::PointCloud::ConstPtr& laser);

	void goalAchievementStateCallback(const std_msgs::Int8::ConstPtr& hasAchieved);
	
	void initializeKalmanVariables();
	
	void stopVideoStreaming();
	void stopRobotTracking();
	void stopCurrentTour();
	void trackRobot();
private:
	static const float LASER_MAX_RANGE;
	static const float MIN_RAND;
	static const float MAX_RAND;
	UDPClient* spdUDPClient;
	ros::NodeHandle nh;
	ros::Publisher cmd_vel_pub;
	ros::Publisher cmd_goto_pub;
	ros::Publisher pose2d_pub;
	//OpenCV
	cv::VideoCapture vc;
	//possibilistic navigation
	Matrix robotVelocity;
	Matrix robotEncoderPosition;
	std::vector<fuzzy::trapezoid*>* kalmanFuzzy;
	Matrix robotState;
	Matrix P;
	Matrix Q;
	Matrix R;
	
	std::vector<Matrix> landmarks;
	
	bool setChargerPosition;
	bool hasAchievedGoal;
	bool keepSpinning;
	bool frontBumpersOk;
	bool rearBumpersOk;
	int udpPort;
	int spdUDPPort;
	unsigned char streamingActive;
	
	unsigned char keepRobotTracking;
	unsigned char keepTourAlive;
	s_sector* navSector;
	s_robot* robotConfig;

	pthread_mutex_t mutexLandmarkLocker;

	void loadSector(int sectorId);
	void loadRobotConfig();
	
	void initializeSPDPort(char* cad);
	
	void getVelocities(char* cad, float& lin_vel, float& angular_vel);
	void moveRobot(float lin_vel, float angular_vel);

	void setRobotPosition(Matrix pose);
	void setRobotPosition(float x, float y, float theta);
	void goToPosition(float x, float y, float th);
	void getPositions(char* cad, float& x, float& y, float& theta);
	
	void startSitesTour();
	void landmarkObservation(Matrix Xk, s_landmark* landmark, float& distance, float& angle);
	std::vector<fuzzy::trapezoid*> getObservationsTrapezoids();
	Matrix normalizeAngles(Matrix trap);
	Matrix denormalizeAngles(Matrix trap, int mode = 0);
	Matrix sortVariation(Matrix variation);

	void getNumberOfCamerasAvailable(int& count);
	void getCameraDevicePort(char* cad, int& device, int& port);
	void beginVideoStreaming(int videoDevice);
	
	static void* streamingThread(void*);
	static void* trackRobotThread(void*);
	static void* sitesTourThread(void*);
	
};

#endif


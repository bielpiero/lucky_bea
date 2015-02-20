#ifndef GENERAL_CONTROLLER_H
#define GENERAL_CONTROLLER_H

#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <cmath>
#include "SocketNode2.h"
#include "SerialPort.h"
#include "stdxros.hpp"
#include "UDPClient.h"
#include "xml/rapidxml_print.hpp"
#include "xml/rapidxml.hpp"

#include "Matrix.h"
#include "stats.h"
#include "fl/fuzzy.h"

#define CONVERTER_BUFFER_SIZE 20
#define SERVOS_COUNT 16

#define PACKAGE_NAME "lucky_bea"

#define XML_FILE_PATH "/src/conf/BeaConSuerte.xml"

#define XML_STATIC_GESTURES_STR "GestosEstaticos"
#define XML_DYNAMIC_GESTURES_STR "GestosDinamicos"

#define XML_ATTRIBUTE_ID_STR "id"
#define XML_ATTRIBUTE_CARD_ID_STR "cardId"
#define XML_ATTRIBUTE_POSITION_STR "pos"
#define XML_ATTRIBUTE_SPEED_STR "speed"
#define XML_ATTRIBUTE_ACCELERATION_STR "acceleration"
#define XML_ATTRIBUTE_TYPE_STR "tipo"
#define XML_ATTRIBUTE_NAME_STR "nombre"

#define XML_ELEMENT_GESTURE_STR "Gesto"
#define XML_ELEMENT_MOTOR_STR "Motor"

#define YES 1
#define NO 0
#define MAYBE 2

#define MAX_LANDMAKS 20;
#define LASER_MAX_RANGE 11.6;

#define STATE_RANGE_X 0
#define STATE_RANGE_Y 10

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


class GeneralController : public CSocketNode // la clase GeneralController hereda de la clase CSocketNode
{
private:
	SerialPort* maestroControllers;
	bool continue_dynamic_thread;
	
	std::string xmlFaceFullPath;
	
		
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
	
	void sonarStateCallback(const sensor_msgs::PointCloud::ConstPtr& sonar);
	void sonarPointCloud2StateCallback(const sensor_msgs::PointCloud2::ConstPtr& sonar);
	
	void laserScanStateCallback(const sensor_msgs::LaserScan::ConstPtr& laser);
	void laserPointCloudStateCallback(const sensor_msgs::PointCloud::ConstPtr& laser);
	
	void initializeKalmanVariables();
	
	void stopVideoStreaming();
	void stopRobotTracking();
private:
	UDPClient* spdUDPClient;
	ros::NodeHandle nh;
	ros::Publisher cmd_vel_pub;
	ros::Publisher pose2d_pub;
	cv::VideoCapture videoCapture;
	
	//possibilistic navigation
	Matrix robotVelocity;
	Matrix robotEncoderPosition;
	std::vector<fuzzy::variable*>* kalmanFuzzy;
	Matrix robotState;
	Matrix P;
	Matrix Q;
	Matrix R;
	
	std::vector<Matrix> landmarks;
	
	bool keepSpinning;
	bool bumpersOk;
	int udpPort;
	int spdUDPPort;
	unsigned char streamingActive;
	
	unsigned char keepRobotTracking;
	
	void initializeSPDPort(char* cad);
	
	void getVelocities(char* cad, double& lin_vel, double& angular_vel);
	void moveRobot(double lin_vel, double angular_vel);
	void moveRobotTo(Matrix pose);
	void trackRobot();
	
	void getNumberOfCamerasAvailable(int& count);
	void getCameraDevicePort(char* cad, int& device, int& port);
	void beginVideoStreaming(int videoDevice);
	
	static void* streamingThread(void*);
	static void* trackRobotThread(void*);
	
};

#endif


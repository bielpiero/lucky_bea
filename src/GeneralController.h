
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include "SocketNode2.h"
#include "SerialPort.h"
#include "stdxros.hpp"
#include "xml/rapidxml_print.hpp"
#include "xml/rapidxml.hpp"

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
	
	ros::NodeHandle nh;
	ros::Publisher cmd_vel_pub;
	//cv::VideoCapture videoCapture;
public:
	GeneralController(ros::NodeHandle nh_);
	~GeneralController(void);
	
	virtual void OnConnection();//callback for client and server
	virtual void OnMsg(char* cad,int length);//callback for client and server
	void Define(void *_Dlg);
private:
	void GetPololuInstruction(char* cad, unsigned char& card_id, unsigned char& servo_id, int& value);
	void GetGestures(std::string type, std::string& gestures);
	void SetGesture(std::string id);
	void SaveGesture(std::string token, int gesture_type);
	void SaveStaticGesture(std::string name, s_motor servos[]);
	void SaveDynamicGesture(std::string name, s_motor servos[]);
	
	void ModifyGesture(std::string token, int gesture_type);
	void ModifyStaticGesture(std::string face_id, std::string name, s_motor servos[]);
	
	void RemoveGesture(std::string id);
	
	void SetServoPosition(unsigned char card_id, unsigned char servo_id, int position);
	void SetServoSpeed(unsigned char card_id, unsigned char servo_id, int speed);
	void SetServoAcceleration(unsigned char card_id, unsigned char servo_id, int speed);

	void StopDynamicGesture();
	
	static void* DynamicFaceThread(void*);
	
	/// ROS Functions
public:
	void batteryStateCallback(const std_msgs::Float32::ConstPtr& battery);
	void bumperStateCallback(const rosaria::BumperState::ConstPtr& bumpers);
	void poseStateCallback(const nav_msgs::Odometry::ConstPtr& pose);
	void sonarStateCallback(const sensor_msgs::PointCloud::ConstPtr& sonar);
	//void laserStateCallback(const sensor_msgs::PointCloud &laser);
	void batteryVoltageCallback(const std_msgs::Float64::ConstPtr& battery);
	
		
private:
	bool keepSpinning;
	bool bumpersOk;
	unsigned char streamingActive;
	std::string xmlFaceFullPath;
	
	void GetVelocities(char* cad, double& lin_vel, double& angular_vel);
	void moveRobot(double lin_vel, double angular_vel);
	
	void GetNumberOfCamerasAvailable(int& count);
	void beginVideoStreaming(int videoDevice);
	void stopVideoStreaming();
	static void* streamingThread(void*);
	
};


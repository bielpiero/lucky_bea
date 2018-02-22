#ifndef RN_ARM_TASK_H
#define RN_ARM_TASK_H

#include "RNRecurrentTask.h"
#include "GeneralController.h"
#include "xmldefs.h"

#include "arm/Arm.h"

#define DEVICENAME          "/dev/ttyUSB1"
#define BAUDRATE            1000000
#define PROTOCOL_VERSION    1.0 

#define XML_ARM_GESTURE_FILE_PATH "conf/BeaArm.xml"

class RNArmTask : public RNRecurrentTask{
private:
    GeneralController* gn;
    std::vector<ArmGesture*> *gestures;
    std::string gestureId;
    SerialPort* maestroController;

    Arm* brazo;
public:
	RNArmTask(const GeneralController* gn, SerialPort* maestroController, const char* name = "Arm Gestures Task", const char* description = "Doris Arm");
	~RNArmTask();
	virtual void task();
	virtual void onKilled();

	void setGesture(std::string gestureId);
    void getGestures(int id, std::string& jsonResponse);
    void moveSingleMotor(int id, int angle);
    void moveAllMotors(std::vector<uint16_t> motors);
};

#endif
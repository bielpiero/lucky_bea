#ifndef RN_ARM_TASK_H
#define RN_ARM_TASK_H

#include "RNRecurrentTask.h"
#include "GeneralController.h"
#include "xmldefs.h"

#define XML_ARM_GESTURE_FILE_PATH "conf/BeaArm.xml"

class ArmMotor{
private:
    std::string id;
    std::string device;
    std::string degrees;
public:
    ArmMotor(){
        this->id = "";
        this->device = "";
        this->degrees = "";
    }
    ~ArmMotor(){}

    std::string getId() { return id; }
    void setId(std::string id) { this->id = id; }

    std::string getDegrees() { return degrees; }
    void setDegrees(std::string degrees) { this->degrees = degrees; }

    std::string getDevice() { return device; }
    void setDevice(std::string device) { this->device = device; }
};

class ArmGesture{
private:
    std::string name;
    std::string id;
    std::string type;
    std::vector<ArmMotor*>* motors;

public:
    ArmGesture(){
        this->name = "";
        this->id = "";
        this->type = "";
        motors = new std::vector<ArmMotor*>();       
    }

    virtual ~ArmGesture(){
        clearMotors();
        delete motors;
    }
    

    std::string getName() { return name; }
    void setName(std::string name) { this->name = name; }
    std::string getId() { return id; }
    void setId(std::string id) { this->id = id; }   
    std::string getType() { return type; }
    void setType(std::string type) { this->type = type; }

    void addMotor(ArmMotor* motor){
        if(motor != NULL){
            motors->push_back(motor);
        }
    }   

    ArmMotor* motorAt(int index){
        ArmMotor* result = NULL;
        if(index > RN_NONE and index < motors->size()){
            result = motors->at(index);
        }
        return result;
    }

    size_t motorsSize(){
        return motors->size();
    }

    void clearMotors(){
        for (int i = 0; i < motors->size(); i++){
            delete motors->at(i);
        }
        motors->clear();
    }

};

class RNArmTask : public RNRecurrentTask{
private:
    GeneralController* gn;
    std::vector<ArmGesture*> *gestures;
    std::string gestureId;
    SerialPort* maestroController;
public:
	RNArmTask(const GeneralController* gn, SerialPort* maestroController, const char* name = "Arm Gestures Task", const char* description = "Doris Arm");
	~RNArmTask();
	virtual void task();
	virtual void onKilled();

	void setGesture(std::string gestureId);
    void getGestures(int id, std::string& jsonResponse);
};

#endif
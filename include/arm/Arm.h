#ifndef RN_ARM_H
#define RN_ARM_H


#include "DynamixelMotor.h"
#include "Hand.h"


#define ENABLE              1
#define DISABLE             0
#define MOVING_THRESHOLD    1
#define ESC_ASCII_VALUE     0x1b
#define umbralTorqueW		200
#define umbralTorqueStop	600
#define vecesLectTorque 	20
#define waitingTime			2.0

#define SHOULDER_ELBOW_DISTANCE  26
#define ELBOW_HAND_DISTANCE 	27
#define FINGER_LENGTH		7

//posicion zero de cada DynamixelMotor --FOLDER SINDEDOS

//la mano tiene que tener el pulgar hacia fuera en posic cero
//velocidad de cada DynamixelMotor
#define speed1 70
#define speed2 60
#define speed3 75
#define speed4 75
#define speed5 40 	// a ojo
#define speed6 40 	// a ojo

//steps del recorrido total de cada DynamixelMotor
#define arc1_grados 60	//682 pasos
#define arc2_grados 50	//antes era 30
#define arc3_grados 120	//1535 pasos
#define arc4_grados 130//1250 pasos //antes era 130
#define arc5_grados 300 //2045 pasos
#define arc6_grados 300 //2045 pasos
#define dedoClosed	0
#define dedoOpened 90

#define arcArtic1 1760
#define arcArtic2 660
#define arcArtic3 4095
#define arcArtic4 3410
#define arcArtic5 1023
#define arcArtic6 1023


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

class ArmFrame{
private:
	std::string id;
	std::vector<ArmMotor*>* motors;
public:

	ArmFrame(){
        this->id = "";
        motors = new std::vector<ArmMotor*>();       
    }

    virtual ~ArmFrame(){
        clearMotors();
        delete motors;
    }

    std::string getId() { return id; }
    void setId(std::string id) { this->id = id; }  

    void addMotor(ArmMotor* DynamixelMotor){
        if(DynamixelMotor != NULL){
            motors->push_back(DynamixelMotor);
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

class ArmGesture{
private:
    std::string name;
    std::string id;
    std::string type;
    std::vector<ArmFrame*>* gestureStates;

public:
    ArmGesture(){
        this->name = "";
        this->id = "";
        this->type = "";
        gestureStates = new std::vector<ArmFrame*>();       
    }

    virtual ~ArmGesture(){
        clearStates();
        delete gestureStates;
    }

    std::string getName() { return name; }
    void setName(std::string name) { this->name = name; }
    std::string getId() { return id; }
    void setId(std::string id) { this->id = id; }   
    std::string getType() { return type; }
    void setType(std::string type) { this->type = type; }

    void addState(ArmFrame* state){
        if(state != NULL){
            gestureStates->push_back(state);
        }
    }

    ArmFrame* stateAt(int index){
        ArmFrame* result = NULL;
        if(index > RN_NONE and index < gestureStates->size()){
            result = gestureStates->at(index);
        }
        return result;
    }

    size_t statesSize(){
        return gestureStates->size();
    }

    void clearStates(){
        for (int i = 0; i < gestureStates->size(); i++){
            delete gestureStates->at(i);
        }
        gestureStates->clear();
    }

};

class Arm{
private:
	dynamixel::PortHandler *portHandler;
	dynamixel::PacketHandler *packetHandler;
	SerialPort* handController;

    std::vector<DynamixelMotor*>* vect_brazo;
    Hand* hand;
    int* goalPositionSteps1;
    int* goalPositionSteps2;
    int* goalPositionAngle1;
    int* goalPositionAngle2;

    int* handGoalPositAngle1;
    int* handGoalPositAngle2;
    int* handGoalPositSteps1;
    int* handGoalPositSteps2;

    int* speedArray;
    bool toInitialPosit;
	
private:
	void setGoalPositionKeyboard(std::vector<uint16_t> positions);
	bool setGoalPosition(ArmGesture* gesture);
	void setBasicSpeedArray();
	void setTorque(int );
	void setArmToZero();
	void setToZero();
	void setSpeedSameTime(int* value);
	void waitFinishMove();
	void setArmLimites();
	bool checkNoChoque(int, int);
	//bool checkNoChoque(int *);
	void armEmergencyStop();
	void emergencyStop();
	
	void armRead1Byte(uint16_t address); 
	void armRead2Bytes(uint16_t address); 
	void armWrite1Byte(uint16_t address,uint8_t value); 
	void armWrite2Bytes(uint16_t address,uint16_t value); 

	bool armSyncWrite1Byte(uint16_t address, int value); 
	bool armSyncWrite2Bytes(uint16_t address, int* value); 
	bool armBulkRead(uint16_t address, uint8_t lenght);

public:
    Arm(SerialPort* handController, std::string deviceName, float protocolVersion);
    virtual ~Arm();
    
    bool openPort(); 
    bool setPortBaudrate(const unsigned int baud); 
    void closePort(); 

    void init(); 
    void shutdown(); 
	void easyMovPredef(int);
	void predifinedMovement(ArmGesture* gesture);
	void bulkControl(std::vector<uint16_t> positions);
	void singleMotor(int id, uint16_t angle);
	void moveHand(int*);
	void PPT();
};
#endif
#ifndef RN_GESTURES_TASK_H
#define RN_GESTURES_TASK_H

#include "RNRecurrentTask.h"
#include "xmldefs.h"

#define XML_GESTURE_FILE_PATH "conf/BeaConSuerte.xml"

class Motor{
private:
    std::string id;
    std::string cardId;
    std::string pos;
    std::string speed;
    std::string acceleration;
public:
    Motor(){
        this->id = "";
        this->cardId = "";
        this->pos = "";
        this->speed = "";
        this->acceleration = "";
    }
    ~Motor(){}

    std::string getCardId() { return cardId; }
    void setCardId(std::string cardId) { this->cardId = cardId; }
    std::string getId() { return id; }
    void setId(std::string id) { this->id = id; }
    std::string getPos() { return pos ; }
    void setPos(std::string pos) { this->pos = pos; }
    std::string getSpeed() { return speed ; }
    void setSpeed(std::string speed) { this->speed = speed; }
    std::string getAcceleration() { return acceleration ; }
    void setAcceleration(std::string acceleration) { this->acceleration = acceleration; }
};

class Gesture{
private:
    std::string name;
    std::string id;
    std::string type;
    std::vector<Motor*>* motors;

public:
    Gesture(){
        this->name = "";
        this->id = "";
        this->type = "";
        motors = new std::vector<Motor*>();       
    }

    virtual ~Gesture(){
        clearMotors();
        delete motors;
    }
    

    std::string getName() { return name; }
    void setName(std::string name) { this->name = name; }
    std::string getId() { return id; }
    void setId(std::string id) { this->id = id; }   
    std::string getType() { return type; }
    void setType(std::string type) { this->type = type; }

    void addMotor(Motor* motor){
        if(motor != NULL){
            motors->push_back(motor);
        }
    }   

    Motor* motorAt(int index){
        Motor* result = NULL;
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

class RNGesturesTask : public RNRecurrentTask{
private:
    std::vector<Gesture*> *gestures;
    std::string gestureId;
    SerialPort* maestroController;
public:
	RNGesturesTask(SerialPort* maestroController, const char* name = "Gestures Task", const char* description = "Doris Faces");
	~RNGesturesTask();
	virtual void task();
	virtual void onKilled();

	void setGesture(std::string gestureId);
};

#endif
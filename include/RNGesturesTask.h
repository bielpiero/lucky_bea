#ifndef RN_GESTURES_TASK_H
#define RN_GESTURES_TASK_H

#include "RNRecurrentTask.h"
#include "GeneralController.h"
#include "xmldefs.h"

#define XML_GESTURE_FILE_PATH "conf/BeaConSuerte.xml"

class FaceMotor{
private:
    std::string id;
    std::string cardId;
    std::string pos;
    std::string speed;
    std::string acceleration;
public:
    FaceMotor(){
        this->id = "";
        this->cardId = "";
        this->pos = "";
        this->speed = "";
        this->acceleration = "";
    }
    ~FaceMotor(){}

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

class FaceFrame{
private:
    std::string id;
    std::vector<FaceMotor*>* motors;
public:

    FaceFrame(){
        this->id = "";
        motors = new std::vector<FaceMotor*>();       
    }

    virtual ~FaceFrame(){
        clearMotors();
        delete motors;
    }

    std::string getId() { return id; }
    void setId(std::string id) { this->id = id; }  

    void addMotor(FaceMotor* faceMotor){
        if(faceMotor != NULL){
            motors->push_back(faceMotor);
        }
    }   

    FaceMotor* motorAt(int index){
        FaceMotor* result = NULL;
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

class FaceGesture{
private:
    std::string name;
    std::string id;
    std::string type;
    std::vector<FaceFrame*>* frames;

public:
    FaceGesture(){
        this->name = "";
        this->id = "";
        this->type = "";
        frames = new std::vector<FaceFrame*>();       
    }

    virtual ~FaceGesture(){
        clearFrames();
        delete frames;
    }
    

    std::string getName() { return name; }
    void setName(std::string name) { this->name = name; }
    std::string getId() { return id; }
    void setId(std::string id) { this->id = id; }   
    std::string getType() { return type; }
    void setType(std::string type) { this->type = type; }

    void addFrame(FaceFrame* frame){
        if(frame != NULL){
            frames->push_back(frame);
        }
    }   

    FaceFrame* frameAt(int index){
        FaceFrame* result = NULL;
        if(index > RN_NONE and index < frames->size()){
            result = frames->at(index);
        }
        return result;
    }

    size_t framesSize(){
        return frames->size();
    }

    void clearFrames(){
        for (int i = 0; i < frames->size(); i++){
            delete frames->at(i);
        }
        frames->clear();
    }

};

class RNGesturesTask : public RNRecurrentTask{
private:
    GeneralController* gn;
    std::vector<FaceGesture*> *gestures;
    std::string gestureId;
    SerialPort* maestroController;
public:
	RNGesturesTask(const GeneralController* gn, SerialPort* maestroController, const char* name = "Gestures Task", const char* description = "Doris Faces");
	~RNGesturesTask();
	virtual void task();
	virtual void onKilled();

	void setGesture(std::string gestureId);
    void getGestures(std::string& jsonResponse);
    void saveGesture(std::string token);
    void modifyGesture(std::string token);
    void modifyStaticGesture(std::string gesture_id, std::string name, s_motor servos[]);
    void saveStaticGesture(std::string name, s_motor servos[]);
    void removeGesture(std::string face_id);
};

#endif
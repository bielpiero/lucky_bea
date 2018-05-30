#ifndef FACE_GESTURE_H
#define FACE_GESTURE_H

#include "RNUtils.h"

class FaceMotor{
private:
    int id;
    int cardId;
    int pos;
    int speed;
    int acceleration;
public:
    FaceMotor(){
        this->id = RN_NONE;
        this->cardId = RN_NONE;
        this->pos = RN_NONE;
        this->speed = RN_NONE;
        this->acceleration = RN_NONE;
    }
    ~FaceMotor(){}

    int getCardId() { return cardId; }
    void setCardId(int cardId) { this->cardId = cardId; }

    int getId() { return id; }
    void setId(int id) { this->id = id; }

    int getPos() { return pos ; }
    void setPos(int pos) { this->pos = pos; }

    int getSpeed() { return speed ; }
    void setSpeed(int speed) { this->speed = speed; }

    int getAcceleration() { return acceleration ; }
    void setAcceleration(int acceleration) { this->acceleration = acceleration; }
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

#endif
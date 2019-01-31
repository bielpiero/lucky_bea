#ifndef FACE_RANGES_MOTORS_H
#define FACE_RANGES_MOTORS_H

#include "RNUtils.h"

class FRMotor{
private:
    int id;
    int cardId;
    int low;
    int high;
    std::string description;
public:
    FRMotor(){
        this->id = RN_NONE;
        this->cardId = RN_NONE;
        this->low = RN_NONE;
        this->high = RN_NONE;
        this->description = "";
    }
    ~FRMotor(){}

    int getCardId() { return cardId; }
    void setCardId(int cardId) { this->cardId = cardId; }

    int getId() { return id; }
    void setId(int id) { this->id = id; }

    int getLow() { return low; }
    void setLow(int low) { this->low = low; }

    int getHigh() { return high; }
    void setHigh(int high) { this->high = high; }

    std::string getDescription() { return description; }
    void setDescription(std::string description) { this->description = description; }
};

class FRMotorsList{
private:
    std::list<FRMotor*>* mouth;
    std::list<FRMotor*>* eyelids;
    std::list<FRMotor*>* eyebrows;
    std::list<FRMotor*>* eyes;
    std::list<FRMotor*>* neck;
public:
    FRMotorsList(){
        mouth = new std::list<FRMotor*>();
        eyelids = new std::list<FRMotor*>();
        eyebrows = new std::list<FRMotor*>();
        eyes = new std::list<FRMotor*>();
        neck = new std::list<FRMotor*>();
    }

    virtual ~FRMotorsList(){
        clearMouthMotors();
        clearEyelidsMotors();
        clearEyebrowsMotors();
        clearEyesMotors();
        clearNeckMotors();
    }

    void addMouthMotor(FRMotor* motor){
        auto m = std::find_if(mouth->begin(), mouth->end(), [motor](FRMotor* mtr){ return motor->getId() == mtr->getId(); });
        if(m == mouth->end()){
            mouth->emplace_back(motor);
        }
    }

    void addEyelidsMotor(FRMotor* motor){
        auto m = std::find_if(eyelids->begin(), eyelids->end(), [motor](FRMotor* mtr){ return motor->getId() == mtr->getId(); });
        if(m == eyelids->end()){
            eyelids->emplace_back(motor);
        }
    }

    void addEyebrowsMotor(FRMotor* motor){
        auto m = std::find_if(eyebrows->begin(), eyebrows->end(), [motor](FRMotor* mtr){ return motor->getId() == mtr->getId(); });
        if(m == eyebrows->end()){
            eyebrows->emplace_back(motor);
        }
    }

    void addEyesMotor(FRMotor* motor){
        auto m = std::find_if(eyes->begin(), eyes->end(), [motor](FRMotor* mtr){ return motor->getId() == mtr->getId(); });
        if(m == eyes->end()){
            eyes->emplace_back(motor);
        }
    }

    void addNeckMotor(FRMotor* motor){
        auto m = std::find_if(neck->begin(), neck->end(), [motor](FRMotor* mtr){ return motor->getId() == mtr->getId(); });
        if(m == neck->end()){
            neck->emplace_back(motor);
        }
    }

    FRMotor* mouthMotorAt(int index){
        return *(std::next(mouth->begin(), index));
    }

    FRMotor* eyelidsMotorAt(int index){
        return *(std::next(eyelids->begin(), index));
    }

    FRMotor* eyebrowsMotorAt(int index){
        return *(std::next(eyebrows->begin(), index));
    }

    FRMotor* eyesMotorAt(int index){
        return *(std::next(eyes->begin(), index));
    }

    FRMotor* neckMotorAt(int index){
        return *(std::next(neck->begin(), index));
    }

    size_t mouthMotorsSize(){
        return mouth->size();
    }

    size_t eyelidsMotorsSize(){
        return eyelids->size();
    }

    size_t eyebrowsMotorsSize(){
        return eyebrows->size();
    }

    size_t eyesMotorsSize(){
        return eyes->size();
    }

    size_t neckMotorsSize(){
        return neck->size();
    }

    void deleteMouthMotorAt(int index) { 
        mouth->erase(std::next(mouth->begin(), index)); 
    }

    void deleteEyelidsMotorAt(int index) { 
        eyelids->erase(std::next(eyelids->begin(), index)); 
    }

    void deleteEyebrowsMotorAt(int index) { 
        eyebrows->erase(std::next(eyebrows->begin(), index)); 
    }

    void deleteEyesMotorAt(int index) { 
        eyes->erase(std::next(eyes->begin(), index)); 
    }

    void deleteNeckMotorAt(int index) { 
        neck->erase(std::next(neck->begin(), index)); 
    }

    void clearMouthMotors(){
        while(!mouth->empty()) {
            delete mouth->front();
            mouth->pop_front();
        }
    }

    void clearEyelidsMotors(){
        while(!eyelids->empty()) {
            delete eyelids->front();
            eyelids->pop_front();
        }
    }

    void clearEyebrowsMotors(){
        while(!eyebrows->empty()) {
            delete eyebrows->front();
            eyebrows->pop_front();
        }
    }

    void clearEyesMotors(){
        while(!eyes->empty()) {
            delete eyes->front();
            eyes->pop_front();
        }
    }

    void clearNeckMotors(){
        while(!neck->empty()) {
            delete neck->front();
            neck->pop_front();
        }
    }
};

#endif
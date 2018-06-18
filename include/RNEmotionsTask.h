#ifndef RN_EMOTIONS_TASK_H
#define RN_EMOTIONS_TASK_H

#include "RNRecurrentTask.h"
#include "GeneralController.h"
#include "Headers.h"

#define XML_IMPULSES_FILE_PATH "conf/BeaImpulses.xml"
#define XML_ELEMENT_IMPULSES_STR "impulses"
#define XML_ELEMENT_IMPULSE_STR "impulse"

#define XML_ATTRIBUTE_ANGRY_STR "angry"
#define XML_ATTRIBUTE_HAPPY_STR "happy"
#define XML_ATTRIBUTE_CALM_STR "calm"
#define XML_ATTRIBUTE_SAD_STR "sad"
#define XML_ATTRIBUTE_AFRAID_STR "afraid"


class Impulse{
private:
    int id;
    int angry;
    int happy;
    int calm;
    int sad;
    int afraid;

public:
    Impulse(){
        this->id = RN_NONE;
        this->angry = RN_NONE;
        this->happy = RN_NONE;
        this->calm = RN_NONE;
        this->sad = RN_NONE;
        this->afraid = RN_NONE;
    }

    Impulse(int angry, int happy, int calm, int sad, int afraid, int id = RN_NONE){
        this->id = RN_NONE;
        this->angry = angry;
        this->happy = happy;
        this->calm = calm;
        this->sad = sad;
        this->afraid = afraid;
    }
    virtual ~Impulse(){}
    

    int getId() { return id; }
    void setId(int id) { this->id = id; }
    int getAngry() { return angry; }
    void setAngry(int angry) { this->angry = angry; }   
    int getHappy() { return happy; }
    void setHappy(int happy) { this->happy = happy; }
    int getCalm() { return calm; }
    void setCalm(int calm) { this->calm = calm; }
    int getSad() { return sad; }
    void setSad(int sad) { this->sad = sad; }
    int getAfraid() { return afraid; }
    void setAfraid(int afraid) { this->afraid = afraid; }

};

class RNEmotionsTask : public RNRecurrentTask{
public:
	RNEmotionsTask(const GeneralController* gn, const char* name = "Emotions Task", const char* description = "Doris Feelings");
	~RNEmotionsTask();
	virtual void task();

    void setSpokenImpulseId(int id);

private:
    void initializeFuzzyEmotionSystem();
    void getSystemInput(double* emotion);
private:
	GeneralController* gn;
	std::vector<Impulse*>* impulses;
	Impulse* currentState;
    int spokenId;

    fl::Engine* emotionEngine;
    fl::InputVariable* angryFS;
    fl::InputVariable* happyFS;
    fl::InputVariable* calmFS;
    fl::InputVariable* sadFS;
    fl::InputVariable* afraidFS;
    fl::OutputVariable* emotionFS;
    fl::RuleBlock* ruleBlock;
};

#endif
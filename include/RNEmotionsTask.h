#ifndef RN_EMOTIONS_TASK_H
#define RN_EMOTIONS_TASK_H

#include "RNRecurrentTask.h"
#include "GeneralController.h"
#include "FRMotorsList.h"
#include "Headers.h"

#define XML_IMPULSES_FILE_PATH "conf/BeaImpulses.xml"
#define XML_FACE_MOTORS_RANGES_FILE_PATH "conf/BeaFaceMotorRanges.xml"

#define XML_ELEMENT_FACE_STR "face"
#define XML_ELEMENT_MOUTH_STR "mouth"
#define XML_ELEMENT_EYELIDS_STR "eyelids"
#define XML_ELEMENT_EYEBROWS_STR "eyebrows"
#define XML_ELEMENT_EYES_STR "eyes"
#define XML_ELEMENT_NECK_STR "neck"

#define XML_ATTRIBUTE_LOW_STR "low"
#define XML_ATTRIBUTE_HIGH_STR "high"
#define XML_ATTRIBUTE_DESCRIPTION_STR "description"

#define XML_ELEMENT_IMPULSES_STR "impulses"
#define XML_ELEMENT_IMPULSE_STR "impulse"

#define XML_ATTRIBUTE_JOY_STR "joy"
#define XML_ATTRIBUTE_FEAR_STR "fear"
#define XML_ATTRIBUTE_APPROVAL_STR "approval"

#define XML_FACE_STR "{face}"
#define XML_NECK_STR "{neck}"


class Impulse{
private:
    int id;
    double joy;
    double fear;
    double approval;

public:
    Impulse(){
        this->id = RN_NONE;
        this->joy = RN_NONE;
        this->fear = RN_NONE;
        this->approval = RN_NONE;
    }

    Impulse(double joy, double fear, double approval, int id = RN_NONE){
        this->id = id;
        this->joy = joy;
        this->fear = fear;
        this->approval = approval;
    }
    virtual ~Impulse(){}
    

    double getId() { return id; }
    void setId(int id) { this->id = id; }
    double getJoy() { return joy; }
    void setJoy(double joy) { this->joy = joy; }   
    double getFear() { return fear; }
    void setFear(double fear) { this->fear = fear; }
    double getApproval() { return approval; }
    void setApproval(double approval) { this->approval = approval; }

    const std::string toString() const{
        char buffer[1024];
        sprintf(buffer, "id: %d {Joy: %lf, Fear: %lf, Approval: %lf}", id, joy, fear, approval);
        return std::string(buffer);
    }

};

class RNEmotionsTask : public RNRecurrentTask{
public:
	RNEmotionsTask(const GeneralController* gn, const char* name = "Emotions Task", const char* description = "Doris Feelings");
	~RNEmotionsTask();
	virtual void task();

    void setSpokenImpulseId(int id);

private:
    void initializeFuzzyEmotionSystem();
    void getSystemInput(double* eyelids, double* eyebrows, double* mouth, double* responseMode, double* voiceRate);
    void setFace(double eyelids, double eyebrows, double mouth);
    void setDialogState(double responseMode, double voiceRate);
private:
	GeneralController* gn;
	std::vector<Impulse*>* impulses;
	Impulse* currentState;

    SerialPort* maestroController;

    FRMotorsList* faceMotors;

    int spokenId;

    fl::Engine* emotionEngine;
    fl::InputVariable* joyFS;
    fl::InputVariable* fearFS;
    fl::InputVariable* approvalFS;
    fl::OutputVariable* eyelidsFS;
    fl::OutputVariable* eyebrowsFS;
    fl::OutputVariable* mouthFS;
    fl::OutputVariable* voiceRateFS;
    fl::OutputVariable* responseModeFS;
    fl::RuleBlock* ruleBlock;
};

#endif
#ifndef RN_DIALOGS_TASK_H
#define RN_DIALOGS_TASK_H

#include "RNRecurrentTask.h"
#include "GeneralController.h"
#include "DorisLipSync.h"

#define XML_DIALOGS_FILE_PATH "conf/BeaInputOutput.xml"


class Message{
private:
    std::string id;
    std::string type;
    std::string lang;
    std::string text;

public:
    Message(){
        this->id = "-1";
        this->type = "-1";
        this->lang = "es";
        this->text = "";
    }
    virtual ~Message(){}
    

    std::string getId() { return id; }
    void setId(std::string id) { this->id = id; }
    std::string getType() { return type; }
    void setType(std::string type) { this->type = type; }   
    std::string getLang() { return lang; }
    void setLang(std::string lang) { this->lang = lang; }
    std::string getText() { return text; }
    void setText(std::string text) { this->text = text; }

};

class InputMessage : public Message{
private:
    std::string complete;
public:
    InputMessage(){
        complete = "0";

    }
    ~InputMessage(){

    }
    std::string getComplete() { return complete; }
    void setComplete(std::string complete) { this->complete = complete; }
};

class OutputMessage : public Message{
private:
    std::string state;
public:
    OutputMessage(){
        this->state = "-1";
    }
    ~OutputMessage(){

    }
    //std::string getTimesUsed() { return used; }
    //void setTimesUsed(std::string used) { this->used = used; }
    std::string getState() { return state; }
    void setState(std::string state) { this->state = state; }
};

class RNDialogsTask : public RNRecurrentTask{
private:
    GeneralController* gn;
    std::string inputMessage;
    std::string state;
    DorisLipSync* tts;
public:
	RNDialogsTask(const GeneralController* gn, DorisLipSync* tts, const char* name = "Dialogs Task", const char* description = "Doris Speech");
	~RNDialogsTask();
	virtual void task();

    void setInputMessage(std::string inputMessage);
    void setState(std::string state);
    
    std::vector<InputMessage*> *inputMessages;
    std::vector<OutputMessage*> *outputMessages;
};

#endif
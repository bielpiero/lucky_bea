#ifndef RN_GESTURES_TASK_H
#define RN_GESTURES_TASK_H

#include "RNRecurrentTask.h"
#include "GeneralController.h"
#include "FaceGesture.h"
#include "xmldefs.h"

#define XML_GESTURE_FILE_PATH "conf/BeaConSuerte.xml"
#define XML_FACE_LABEL "{face}"
#define XML_NECK_LABEL "{neck}"


class RNGesturesTask : public RNRecurrentTask{
private:
    GeneralController* gn;
    std::vector<FaceGesture*> *gestures;
    std::string gestureId;
    SerialPort* maestroController;
public:
	RNGesturesTask(const GeneralController* gn, const char* name = "Gestures Task", const char* description = "Doris Faces");
	~RNGesturesTask();
	virtual void task();

	void setGesture(std::string gestureId);
    void getGestures(std::string& jsonResponse);
    void saveGesture(std::string token);
    void modifyGesture(std::string token);
    void modifyStaticGesture(std::string gesture_id, std::string name, s_motor servos[]);
    void saveStaticGesture(std::string name, s_motor servos[]);
    void removeGesture(std::string face_id);
};

#endif
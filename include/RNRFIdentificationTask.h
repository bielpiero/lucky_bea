#ifndef RN_RF_IDENTIFICATION_TASK_H
#define RN_RF_IDENTIFICATION_TASK_H

#include "RNRecurrentTask.h"

#include "rfid/ltkcpp.h"

#define RFID_READER_VARIABLE_LENGTH 4

class RFData{
public:
	static const float OFFSET;
	RFData(std::string data = ""){ initializeFromString(data); }
	~RFData();

	void setTagKey(std::string tagKey){ this->tagKey = tagKey; }
	void setTimestamp(std::string timestamp){ this->timestamp = timestamp; }
	void setRSSi(float rssi){ this->rssi = rssi; }

	std::string getTagKey(void){ return tagKey; }
	std::string getTimestamp(void){ return timestamp; }
	float getRSSi(void){ return rssi; }

	void initializeFromString(std::string data){
		if(data != ""){
			std::vector<std::string> info = RNUtils::split((char*)data.c_str(), ",");
			if(info.size() == RFID_READER_VARIABLE_LENGTH){
				tagKey = info.at(3);
				timestamp = info.at(1);
				rssi = std::atof(info.at(2).c_str());
			}
		} else {
			tagKey = "";
			timestamp = "";
			rssi = 0;
		}
	}
private:
	std::string tagKey;
	std::string timestamp;
	float rssi;

};

class RNRFIdentificationTask : public RNRecurrentTask{
public:
	static const unsigned int RF_BUFFER_SIZE;
	RNRFIdentificationTask(const char* name = "RF Identification Task", const char* description = "");
	~RNRFIdentificationTask();
	int init(void);
	virtual void task();
	virtual void onKilled();

private:
	RFData* findRFIDByKey(std::string key);
	float convertToMeters(float rssi);

	int connectTo(const char* reader);
	int resetDeviceConfiguration();
	int resetToDefaultConfiguration(void);

	int deleteAllROSpecs(void);
	int addROSpec(void);
	int enableROSpec(void);
	int startROSpec(void);

	int getDataFromDevice(char* data);
	void getOneTagData(LLRP::CTagReportData* tag, char* data, int& length);

	int checkConnectionStatus();
	int checkLLRPStatus(LLRP::CLLRPStatus* status, const char* what);

	LLRP::CMessage* transact(LLRP::CMessage* msg);
	LLRP::CMessage* recvMessage(int msecMax);

private:
	bool deviceInitialized;
	int readerDescriptor;
	static const char* DEVICE_NAME;
	LLRP::CConnection* conn;
	std::vector<RFData*>* rfids;
	std::vector<RNLandmark*>* landmarks;
};

#endif
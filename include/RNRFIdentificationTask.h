#ifndef RN_RF_IDENTIFICATION_TASK_H
#define RN_RF_IDENTIFICATION_TASK_H

#include "RNRecurrentTask.h"
#include "AntennaDataList.h"

#include "rfid/ltkcpp.h"
#include "rfid/impinj_ltkcpp.h"

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
	static const unsigned int RO_SPEC_ID;
	static const unsigned int ANTENNAS_NUMBER;
	RNRFIdentificationTask(const char* name = "RF Identification Task", const char* description = "");
	~RNRFIdentificationTask();
	int init(void);
	virtual void task();
	virtual void onKilled();

private:
	RFData* findRFIDByKey(std::string key);
	float convertToMeters(float rssi);

	int connectTo(const char* reader);
	int enableImpinjExtensions();
	int resetDeviceConfiguration();
	int resetToDefaultConfiguration(void);

	int getReaderCapabilities();
	int getReaderConfiguration();
	int setReaderConfiguration();

	int addAccessSpecification();
	int enableAccessSpecification();

	int deleteAllROSpecs(void);
	int addROSpec(void);
	int enableROSpec(void);
	int startROSpec(void);
	int stopROSpec(void);

	int getDataFromDevice(std::string& data);

	void handleAntennaEvent(LLRP::CAntennaEvent* antennaEvent);
	void handleReaderExceptionEvent(LLRP::CReaderExceptionEvent* readerExceptionEvent);

	void getOneTagData(LLRP::CTagReportData* tag, std::string& data);

	int checkConnectionStatus();
	int checkLLRPStatus(LLRP::CLLRPStatus* status, const char* what);

	LLRP::CMessage* transact(LLRP::CMessage* msg, int timeout = 5000);
	LLRP::CMessage* recvMessage(int msecMax);

private:
	unsigned int messageId;
	bool deviceInitialized;
	int readerDescriptor;

	unsigned short powerLevelIndex;
	unsigned int deviceModelNumber;
	short transmitPowerValue;

	static const char* DEVICE_NAME;
	LLRP::CConnection* conn;

	AntennaDataList* antennasList;
	
	std::vector<RFData*>* rfids;
	std::vector<RNLandmark*>* landmarks;
};

#endif
#ifndef RN_RF_IDENTIFICATION_TASK_H
#define RN_RF_IDENTIFICATION_TASK_H

#include "RNRecurrentTask.h"
#include "GeneralController.h"
#include "AntennaDataList.h"

#include "rfid/ltkcpp.h"
#include "rfid/impinj_ltkcpp.h"

#define XML_PEOPLE_TAGS_FILE_PATH "conf/BeaTagsList.xml"

#define RFID_READER_VARIABLE_LENGTH 4
#define TID_OP_SPEC_ID          123
#define USER_MEMORY_OP_SPEC_ID  321

#define TRANSMISSION_POWER_INDEX_1 81
#define TRANSMISSION_POWER_INDEX_2 71
#define FRONT_2_BACK_RATIO 18

#define MAX_RSSI_ENVIRONMENT_VALUE -36
#define MIN_RSSI_ENVIRONMENT_VALUE -38

#define XML_TAG_LIST_ATTRIBUTE_ACTIVE_STR "active"
#define XML_TAG_LIST_ATTRIBUTE_HOLDER_NAME_STR "holder-name"
#define XML_TAG_LIST_ATTRIBUTE_HOLDER_ID_STR "holder-id"

struct s_person_tag{
	std::string id;
	int active;
	std::string holderName;
	std::string holderId;
};

class RFData{	
public: 
	RFData(std::string data = ""){
		initializeFromString(data); 
	}
	~RFData() {}

	std::string getTagKey(void){ return tagKey; }
	unsigned long long getTimestamp(void){ return timestamp; }

	void setTimestamp(unsigned long long timestamp, int mode = 0){
		if(mode == 0){
			this->lastTimestamp = this->timestamp;
		}
		this->timestamp = timestamp;
	}

	void setAntenna(std::string antenna){
		this->antenna = antenna;
	}

	void update(double rssi, double phaseAngle, double dopplerFrequency){
		this->rssi = rssi;
		this->angle = phaseAngle;
		this->dopplerFrequency = dopplerFrequency;
		convertToDistance();
	}
	bool isRemovable(){ return ((timestamp - lastTimestamp) > 3e6); }
	double getRSSI(void){ return rssi; }
	double getPhaseAngle(void){ return angle; }
	double getDistance(void){ return distance; }
	double getDopplerFrequency(void){ return dopplerFrequency; }
	std::string getAntenna(void){ return antenna; }

	void initializeFromString(std::string data){
		if(data != ""){
			std::vector<std::string> info = RNUtils::split(data, ",");
			/*for(int i = 0; i < info.size(); i++){
				printf("[%d]: %s\n", i, info.at(i).c_str());
			}*/
			tagKey = std::string(info.at(0));
			if(info.at(1) == "1"){
				antenna = std::string(SEMANTIC_SIDE_LEFT_STR);
			} else {
				antenna = std::string(SEMANTIC_SIDE_RIGHT_STR);
			}
			rssi = (double)std::stof(info.at(2));
			timestamp = std::stoull(info.at(3));
			lastTimestamp = timestamp;
			//convertToDistance();
			
		} else {
			distance = 0.0;
			tagKey = "";
			timestamp = 0;
			rssi = -1.0;
			angle = 0.0;
			dopplerFrequency = 0.0;
			antenna = std::string(SEMANTIC_SIDE_LEFT_STR);
		}
	}

	const char* toString() const{
		std::ostringstream buffer;
		buffer.clear();
		buffer << "Key: " << tagKey << ", ";
		buffer << "Antenna: " << antenna << ", ";
		buffer << "RSSI: " << rssi << ", ";
		buffer << "Angle: " << angle << ", ";
		buffer << "DF: " << dopplerFrequency << ", ";
		buffer << "Distance: " << distance;
		return buffer.str().c_str();
	}
private:
	void convertToDistance(){
		double txPwIndex = (double)TRANSMISSION_POWER_INDEX_1;
		/*if(antenna == 2){
			txPwIndex = (double)TRANSMISSION_POWER_INDEX_2;
		}*/
		double ptx = -txPwIndex * AntennaData::TX_POWER_INDEX_MULTIPLIER + AntennaData::TX_POWER_OFFSET_DBM;
		
		double wavelength = AntennaData::C / AntennaData::FREQUENCY;
		
		double txPowermW = RNUtils::dBmTomilliwatts(ptx);

		double rssimW = RNUtils::dBmTomilliwatts(rssi);
		double txGainmW = RNUtils::dBmTomilliwatts(AntennaData::TX_GAIN);

		this->distance = (wavelength / (4 * M_PI)) * std::sqrt((txPowermW * txGainmW)/rssimW);
		//printf("key: %s, antenna: %d, rssi_mw: %g, rssi: %g, antenna_gain: %lf, distance: %g\n", tagKey.c_str(), antenna, rssimW, rssi, txGainmW, distance);
	}
private:
	std::string tagKey;
	unsigned long long timestamp;
	unsigned long long lastTimestamp;
	double rssi;
	double distance;
	double angle;
	double dopplerFrequency;
	std::string antenna;

};


class RNRFIdentificationTask : public RNRecurrentTask{
public:
	static const unsigned int RF_BUFFER_SIZE;
	static const unsigned int RO_SPEC_ID;
	static const unsigned int ANTENNAS_NUMBER;
	RNRFIdentificationTask(const GeneralController* gn, const char* name = "RF Identification Task", const char* description = "");
	~RNRFIdentificationTask();
	int init(void);
	void reloadCurrentSector(void);
	virtual void task();
	virtual void kill();

	void addTagsCallback(RNFunPointer* func);
	void remTagsCallback(RNFunPointer* func);
private:
	RFData* findByKeyAntenna(std::string key, std::string antenna);

	void loadPeopleTagFile();

	void checkForActions();
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

	LLRP::CMessage* transact(LLRP::CMessage* msg, int timeout = 2000);
	LLRP::CMessage* recvMessage(int msecMax);

	void runTagsCallbacks(std::list<std::string> l);
private:
	GeneralController* gn;
	unsigned int messageId;
	bool deviceInitialized;
	int readerDescriptor;

	MapSector* currentSector;

	int powerIndexAntenna1;
	int powerIndexAntenna2;

	bool isAtHallway;
	bool isAtDoor;

	unsigned short powerLevelIndex;
	unsigned int deviceModelNumber;
	short transmitPowerValue;

	static const char* DEVICE_NAME;
	LLRP::CConnection* conn;

	AntennaDataList* antennasList;
	std::FILE* file;
	std::list<RFData*>* rfids;
	std::list<s_person_tag*>* peopleTags;

	std::list<RNFunPointer*> tagsSubscribers;
};

#endif
#ifndef RN_RF_IDENTIFICATION_TASK_H
#define RN_RF_IDENTIFICATION_TASK_H

#include "RNRecurrentTask.h"
#include "AntennaDataList.h"

#include "rfid/ltkcpp.h"
#include "rfid/impinj_ltkcpp.h"

#define RFID_READER_VARIABLE_LENGTH 5
#define TID_OP_SPEC_ID          123
#define USER_MEMORY_OP_SPEC_ID  321

#define TRANSMISSION_POWER_INDEX_1 81
#define TRANSMISSION_POWER_INDEX_2 71
#define FRONT_2_BACK_RATIO 18



class RFData{	
public: 
	RFData(std::string data = ""){
		ATTENUATIONS.push_back(PointXY(34.61, -26));	// @0.35m
		ATTENUATIONS.push_back(PointXY(37.98, -38.5));	// @1.00m
		ATTENUATIONS.push_back(PointXY(39.01, -45.5));	// @2.00m
		ATTENUATIONS.push_back(PointXY(42.5, -52.5));	// @3.00m
		ATTENUATIONS.push_back(PointXY(44.5, -57));		// @4.00m
		ATTENUATIONS.push_back(PointXY(50.5, -64));		// @5.00m
		ATTENUATIONS.push_back(PointXY(54, -71));		// @6.00m
		ATTENUATIONS.push_back(PointXY(58.57, -78));	// @7.00m
		//ATTENUATIONS.push_back(PointXY(54.57, -67));	// @8.00m
		initializeFromString(data); 
	}
	~RFData() {}

	std::string getTagKey(void){ return tagKey; }
	std::string getTimestamp(void){ return timestamp; }

	void setTimestamp(std::string timestamp){
		this->timestamp = timestamp;
	}

	void setAntenna(int antenna){
		this->antenna = antenna;
	}

	void setPhaseAngleRSSI(float phaseAngle, float rssi){
		this->rssi = rssi;
		this->angle = phaseAngle;
		convertToDistance();
	}

	float getRSSI(void){ return rssi; }
	float getPhaseAngle(void){ return angle; }
	float getDistance(void){ return distance; }
	int getAntenna(void){ return antenna; }

	void initializeFromString(std::string data){
		if(data != ""){
			std::vector<std::string> info = RNUtils::split((char*)data.c_str(), ",");
			if(info.size() == RFID_READER_VARIABLE_LENGTH){
				tagKey = info.at(0);
				timestamp = info.at(4);
				angle = std::atof(info.at(3).c_str());
				rssi = std::atof(info.at(2).c_str());
				antenna = std::atoi(info.at(1).c_str());
				float patt = 0.0, fspl = 0.0;
				float d = 1.683716128092856;
				getFSPLPatt(d, &patt, &fspl);
				RNUtils::printLn("TAG: %s, RSSI: %f, FSPL: %f, Patt: %f", tagKey.c_str(), rssi, fspl, patt);
				//convertToDistance();
			}
		} else {
			distance = 0.0;
			tagKey = "";
			timestamp = "";
			rssi = -1.0;
			angle = 0.0;
			antenna = RN_NONE;
		}
	}
private:

	void getFSPLPatt(float d, float* patt, float* fspl){
		float ptx = ((float)TRANSMISSION_POWER_INDEX_1) * 0.25 + 10.0;
		float fsplmW = std::pow((4 * M_PI * d / 0.346060675971), 2);
		*fspl = RNUtils::milliwattsTodBm(fsplmW);
		*patt = ptx + AntennaData::ANTENNA_GAIN - *fspl - rssi;
	}

	void convertToDistance(){
		float ptx = ((float)TRANSMISSION_POWER_INDEX_1) * 0.25 + 10.0;
		float patt = 0;
		float rssiChecked;
		if(angle > M_PI and angle < (3 * M_PI)){
			rssiChecked = rssi + ((float)FRONT_2_BACK_RATIO);
		} else {
			rssiChecked = rssi;
		}
		for (int i = 0; i < ATTENUATIONS.size() - 1; ++i){
			//if((ATTENUATIONS.at(i).getRSSI() > rssiChecked) and (ATTENUATIONS.at(i + 1).getRSSI() < rssiChecked)){
			//	patt = ((rssiChecked - ATTENUATIONS.at(i).getRSSI())/(ATTENUATIONS.at(i + 1).getRSSI() - ATTENUATIONS.at(i).getRSSI())) * (ATTENUATIONS.at(i + 1).getAttenuation() - ATTENUATIONS.at(i).getAttenuation()) + ATTENUATIONS.at(i).getAttenuation();
			//}
		}
		float fspldBm = -rssiChecked + ptx - patt + .5;
		float fsplmW = RNUtils::dBmTomilliwatts(fspldBm);

		this->distance = std::sqrt(fsplmW) * 0.027567227692947; // (c/(4*pi*f))
	}
private:
	std::vector<PointXY> ATTENUATIONS;
	std::string tagKey;
	std::string timestamp;
	float rssi;
	float distance;
	float angle;
	int antenna;

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
	RFData* findByKeyAntenna(std::string key, int antenna);

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
	std::FILE* file;
	std::vector<RFData*>* rfids;
	std::vector<RNLandmark*>* landmarks;
};

#endif
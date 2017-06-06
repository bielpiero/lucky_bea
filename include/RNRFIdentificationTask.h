#ifndef RN_RF_IDENTIFICATION_TASK_H
#define RN_RF_IDENTIFICATION_TASK_H

#include "RNRecurrentTask.h"
#include "AntennaDataList.h"

#include "rfid/ltkcpp.h"
#include "rfid/impinj_ltkcpp.h"

#define RFID_READER_VARIABLE_LENGTH 6
#define TID_OP_SPEC_ID          123
#define USER_MEMORY_OP_SPEC_ID  321

#define TRANSMISSION_POWER_INDEX_1 81
#define TRANSMISSION_POWER_INDEX_2 71
#define FRONT_2_BACK_RATIO 18

class RFData{	
public: 
	RFData(std::string data = ""){
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

	void update(double rssi, float phaseAngle, float dopplerFrequency){
		this->rssi = rssi;
		this->angle = phaseAngle;
		this->dopplerFrequency = dopplerFrequency;
		convertToDistance();
	}

	float getRSSI(void){ return rssi; }
	float getPhaseAngle(void){ return angle; }
	float getDistance(void){ return distance; }
	float getDopplerFrequency(void){ return dopplerFrequency; }
	int getAntenna(void){ return antenna; }

	void initializeFromString(std::string data){
		if(data != ""){
			std::vector<std::string> info = RNUtils::split((char*)data.c_str(), ",");
			if(info.size() == RFID_READER_VARIABLE_LENGTH){
				tagKey = info.at(0);
				timestamp = info.at(5);
				dopplerFrequency = std::atof(info.at(4).c_str());
				angle = std::atof(info.at(3).c_str());
				rssi = (double)std::atof(info.at(2).c_str());
				antenna = std::atoi(info.at(1).c_str());
				convertToDistance();
			}
		} else {
			distance = 0.0;
			tagKey = "";
			timestamp = "";
			rssi = -1.0;
			angle = 0.0;
			dopplerFrequency = 0.0;
			antenna = RN_NONE;
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
		float txPwIndex = (float)TRANSMISSION_POWER_INDEX_1;
		/*if(antenna == 2){
			txPwIndex = (float)TRANSMISSION_POWER_INDEX_2;
		}*/
		float ptx = -txPwIndex * AntennaData::TX_POWER_INDEX_MULTIPLIER + AntennaData::TX_POWER_OFFSET_DBM;
		
		double wavelength = AntennaData::C / AntennaData::FREQUENCY;
		
		double txPowermW = RNUtils::dBmTomilliwatts(ptx);

		double rssimW = RNUtils::dBmTomilliwatts(rssi);
		double txGainmW = RNUtils::dBmTomilliwatts(AntennaData::TX_GAIN);

		this->distance = (wavelength / (4 * M_PI)) * std::sqrt((txPowermW * txGainmW)/rssimW);
		//printf("key: %s, antenna: %d, rssi_mw: %g, rssi: %g, antenna_gain: %lf, distance: %g\n", tagKey.c_str(), antenna, rssimW, rssi, txGainmW, distance);
	}
private:
	std::string tagKey;
	std::string timestamp;
	double rssi;
	double distance;
	float angle;
	float dopplerFrequency;
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

	int powerIndexAntenna1;
	int powerIndexAntenna2;

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
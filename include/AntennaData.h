#ifndef ANTENNA_DATA_H
#define ANTENNA_DATA_H

#include <iostream>
#include <sstream>
#include <cmath>

class AntennaData{
public:
	AntennaData(unsigned short id = -1) {
		this->antennaId = id;
		txHopTableId = 0;
		txchannelIndex = 0;
		rxSensitivity = 0;
		antennaGain = 0;
		txPower = 0;
	}
	~AntennaData();

	void setAntennaId(unsigned short antennaId){ this->antennaId = antennaId; }
	void setTxHopTableId(unsigned short txHopTableId){ this->txHopTableId = txHopTableId; }
	void setTxchannelIndex(unsigned short txchannelIndex){ this->txchannelIndex = txchannelIndex; }
	void setAntennaGain(int antennaGain){ this->antennaGain = antennaGain; }
	void setTxPower(int txPower){ this->txPower = txPower; }
	void setRxSensitivity(unsigned short rxSensitivity){ this->rxSensitivity = rxSensitivity; }

	unsigned short getAntennaId() const { return antennaId; }
	unsigned short getTxHopTableId() const { return txHopTableId; }
	unsigned short getTxchannelIndex() const { return txchannelIndex; }
	int getAntennaGain() const { return antennaGain; }
	int getTxPower() const { return txPower; }
	unsigned short getRxSensitivity() const { return rxSensitivity; }

	const char* toString() const{
		std::ostringstream buffer;
		buffer.clear();
		buffer << "\n";
		buffer << "Antena Properties and Configuration" << "\n";
		buffer << "--------------------------------------------------" << "\n";
		buffer << "ID: " << getAntennaId() << "\n";
		buffer << "Gain: " << getAntennaGain() << "\n";
		buffer << "Tx Power: " << getTxPower() << "\n";
		buffer << "Tx Hop Table Id: " << getTxHopTableId() << "\n";
		buffer << "Tx Channel Index: " << getTxchannelIndex() << "\n";
		buffer << "Rx Sensitivity: " << getRxSensitivity() << "\n";

		return buffer.str().c_str();
	}
public:
	static const float ANTENNA_GAIN = 8.5;
	static const float FREQUENCY = 866.9e6;
	static const float C = 3e8;
private:
	unsigned short antennaId;
	unsigned short txHopTableId;
	unsigned short txchannelIndex;
	int antennaGain;
	int txPower;
	unsigned short rxSensitivity;
};

#endif
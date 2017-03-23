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
	static const float TX_GAIN = 8.5;
	static const float FREQUENCY = 866.9e6;
	static const float C = 3e8;
	static const float TX_POWER_OFFSET_DBM = -10.0;
	static const float RX_GAIN_1 = 15.0;
	static const float RX_GAIN_2 = 2.0;
	static const float RSSI_MARGIN = -40.0;
	static const float RSSI_MARGIN_LOSS = -55.0;
	static const float TX_POWER_INDEX_MULTIPLIER = 0.25;
	static const float RX_LOSS = 4.0;
private:
	unsigned short antennaId;
	unsigned short txHopTableId;
	unsigned short txchannelIndex;
	int antennaGain;
	int txPower;
	unsigned short rxSensitivity;
};

#endif
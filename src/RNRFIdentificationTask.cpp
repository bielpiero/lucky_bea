#include "RNRFIdentificationTask.h"

const float RFData::OFFSET = -59;
const unsigned int RNRFIdentificationTask::RF_BUFFER_SIZE = 65535;
const unsigned int RNRFIdentificationTask::RO_SPEC_ID = 1111;
const char* RNRFIdentificationTask::DEVICE_NAME = "speedwayr-11-94-a3.local";

RNRFIdentificationTask::RNRFIdentificationTask(const char* name, const char* description) : RNRecurrentTask(name, description){
	conn = NULL;
	readerDescriptor = RN_NONE;
	this->deviceInitialized = false;
	landmarks = new std::vector<RNLandmark*>();
	rfids = new std::vector<RFData*>();
	
}

RNRFIdentificationTask::~RNRFIdentificationTask(){
	if(conn != NULL){
		if(readerDescriptor == 0){
			if(resetDeviceConfiguration() == 0){
				RNUtils::printLn("Success: resetConfiguration on %s OK...", DEVICE_NAME);
			}
			conn->closeConnectionToReader();
			RNUtils::printLn("Disconnected from RF Speedway Reader...");
		}
		delete conn;
	}
}

void RNRFIdentificationTask::task(){
	if(this->deviceInitialized){
		std::string data = "";
		if(startROSpec() == 0){
			if(getDataFromDevice(data) == 0){
				RNUtils::printLn("Success data: %s", data.c_str());
			}
		}	
	} else {
		init();
	}
	/*for (int i = 0; i < rfids->size(); i++){
		float distance = convertToMeters(rfids->at(i)->getRSSi());

		RNUtils::printLn("Distance RF Tag[%s](%d): %f", rfids->at(i)->getTagKey().c_str(), i, rfids->at(i)->getRSSi());
	}*/
	
}

void RNRFIdentificationTask::onKilled(){
	this->deviceInitialized = false;
}

int RNRFIdentificationTask::init(void){
	int result = 0;
	if(connectTo(DEVICE_NAME) == 0){
		if(resetDeviceConfiguration() == 0){
			RNUtils::printLn("Success: resetConfiguration on %s OK...", DEVICE_NAME);
			if(addROSpec() == 0){
				RNUtils::printLn("Success: Added readers operation specifications on %s OK...", DEVICE_NAME);
				if(enableROSpec() == 0){
					RNUtils::printLn("Success: Enabled readers operation specifications on %s OK...", DEVICE_NAME);
					this->deviceInitialized = true;
				} else {
					result = RN_NONE;
				}
			} else {
				result = RN_NONE;
			}
		} else {
			result = RN_NONE;
		}
	} else {
		result = RN_NONE;
	}
}

int RNRFIdentificationTask::connectTo(const char* reader){
	LLRP::CTypeRegistry* typeRegistry;
	
	int result = 0;

	typeRegistry = LLRP::getTheTypeRegistry();
	if(typeRegistry != NULL){
		if(conn != NULL){
			delete conn;
		}
		conn = new LLRP::CConnection(typeRegistry, RF_BUFFER_SIZE);
		if(conn != NULL){
			readerDescriptor = conn->openConnectionToReader(reader);
			if(readerDescriptor == 0){
				RNUtils::printLn("Success: %s connected...", reader);
			} else {
				RNUtils::printLn("Error: connection to %s has failed...", reader);
				result = -3;
			}
		} else {
			RNUtils::printLn("Error: connection failed...");
			result = -2;
		}
	} else {
		RNUtils::printLn("Error: getTheTypeRegistry failed...");
		result = RN_NONE;
	}

	return result;
}

int RNRFIdentificationTask::resetDeviceConfiguration(){
	int result = 0;
	if(resetToDefaultConfiguration() == 0){
		if(deleteAllROSpecs() != 0){
			result = RN_NONE;
		}
	} else {
		result = RN_NONE;
	}
	return result;
}

int RNRFIdentificationTask::getReaderCapabilities(){
	int result = 0;
	LLRP::CGET_READER_CAPABILITIES* cmd;
	LLRP::CMessage* message = NULL;
	LLRP::CGET_READER_CAPABILITIES_RESPONSE* response;

	LLRP::CRegulatoryCapabilities *regCap;
	LLRP::CUHFBandCapabilities *uhfCap;
	LLRP::CTransmitPowerLevelTableEntry *pwrLvl;
	LLRP::CGeneralDeviceCapabilities *ceviceCap;
	std::list<LLRP::CTransmitPowerLevelTableEntry*>::iterator pwrLvlIt;

	cmd = new LLRP::CGET_READER_CAPABILITIES();
	return result;
}

int RNRFIdentificationTask::getReaderConfiguration(){
	int result = 0;

	return result;
}

int RNRFIdentificationTask::setReaderConfiguration(){
	int result = 0;
	LLRP::CSET_READER_CONFIG* cmd;
	LLRP::CMessage* message = NULL;
	LLRP::CSET_READER_CONFIG_RESPONSE* response;

	cmd = new LLRP::CSET_READER_CONFIG();
	return result;
}

int RNRFIdentificationTask::deleteAllROSpecs(void){
	int result = 0;
	LLRP::CDELETE_ROSPEC* cmd;
	LLRP::CMessage* message = NULL;
	LLRP::CDELETE_ROSPEC_RESPONSE* response;

	cmd = new LLRP::CDELETE_ROSPEC();
	cmd->setMessageID(102);
	cmd->setROSpecID(0);

	message = transact(cmd);

	delete cmd;

	if(message != NULL){
		response = (LLRP::CDELETE_ROSPEC_RESPONSE*)message;
		if(checkLLRPStatus(response->getLLRPStatus(), "deleteAllROSpecs") != 0){
			result = RN_NONE;
		}
		delete message;
	} else {
		result = RN_NONE;
	}
	
	return result;	
}


int RNRFIdentificationTask::addROSpec(void){
	int result = 0;

	LLRP::CROSpecStartTrigger* rospecStartTrigger = new LLRP::CROSpecStartTrigger();
    rospecStartTrigger->setROSpecStartTriggerType(LLRP::ROSpecStartTriggerType_Null);

    LLRP::CROSpecStopTrigger* rospecStopTrigger = new LLRP::CROSpecStopTrigger();
    rospecStopTrigger->setROSpecStopTriggerType(LLRP::ROSpecStopTriggerType_Null);
    rospecStopTrigger->setDurationTriggerValue(0);     /* n/a */

    LLRP::CROBoundarySpec* roBoundarySpec = new LLRP::CROBoundarySpec();
    roBoundarySpec->setROSpecStartTrigger(rospecStartTrigger);
    roBoundarySpec->setROSpecStopTrigger(rospecStopTrigger);

    LLRP::CAISpecStopTrigger* aiSpecStopTrigger = new LLRP::CAISpecStopTrigger();
    aiSpecStopTrigger->setAISpecStopTriggerType(LLRP::AISpecStopTriggerType_Duration);
    aiSpecStopTrigger->setDurationTrigger(5000);

    LLRP::CInventoryParameterSpec* inventoryParameterSpec = new LLRP::CInventoryParameterSpec();
    inventoryParameterSpec->setInventoryParameterSpecID(1234);
    inventoryParameterSpec->setProtocolID(LLRP::AirProtocols_EPCGlobalClass1Gen2);

    LLRP::llrp_u16v_t antennaIDs = LLRP::llrp_u16v_t(1); /* Modificar para cuando se agregue la segunda antena */
    antennaIDs.m_pValue[0] = 0;  
    //antennaIDs.m_pValue[1] = 1;        

    LLRP::CAISpec* aiSpec = new LLRP::CAISpec();
    aiSpec->setAntennaIDs(antennaIDs);
    aiSpec->setAISpecStopTrigger(aiSpecStopTrigger);
    aiSpec->addInventoryParameterSpec(inventoryParameterSpec);

    LLRP::CTagReportContentSelector* tagReportContentSelector = new LLRP::CTagReportContentSelector();
    tagReportContentSelector->setEnableROSpecID(FALSE);
    tagReportContentSelector->setEnableSpecIndex(FALSE);
    tagReportContentSelector->setEnableInventoryParameterSpecID(FALSE);
    tagReportContentSelector->setEnableAntennaID(true);
    tagReportContentSelector->setEnableChannelIndex(true);
    tagReportContentSelector->setEnablePeakRSSI(true);
    tagReportContentSelector->setEnableFirstSeenTimestamp(true);
    tagReportContentSelector->setEnableLastSeenTimestamp(true);
    tagReportContentSelector->setEnableTagSeenCount(true);
    tagReportContentSelector->setEnableAccessSpecID(true);

    LLRP::CROReportSpec* roReportSpec = new LLRP::CROReportSpec();
    roReportSpec->setROReportTrigger(LLRP::ROReportTriggerType_Upon_N_Tags_Or_End_Of_ROSpec);
    roReportSpec->setN(0);         /* Unlimited */
    roReportSpec->setTagReportContentSelector(tagReportContentSelector);

    LLRP::CROSpec* rospec = new LLRP::CROSpec();
    rospec->setROSpecID(RO_SPEC_ID);
    rospec->setPriority(0);
    rospec->setCurrentState(LLRP::ROSpecState_Disabled);
    rospec->setROBoundarySpec(roBoundarySpec);
    rospec->addSpecParameter(aiSpec);
    rospec->setROReportSpec(roReportSpec);

	LLRP::CADD_ROSPEC* cmd;
	LLRP::CMessage* message = NULL;
	LLRP::CADD_ROSPEC_RESPONSE* response;

	cmd = new LLRP::CADD_ROSPEC();
	cmd->setMessageID(3);
	cmd->setROSpec(rospec);

	message = transact(cmd);

	delete cmd;

	if(message != NULL){
		response = (LLRP::CADD_ROSPEC_RESPONSE*)message;
		if(checkLLRPStatus(response->getLLRPStatus(), "addROSpec") != 0){
			result = RN_NONE;
		}
		delete message;
	} else {
		result = RN_NONE;
	}
	
	return result;	
}

int RNRFIdentificationTask::addAccessSpecification(){
	int result = 0;

	return result;
}

int RNRFIdentificationTask::enableROSpec(void){
	int result = 0;
	LLRP::CENABLE_ROSPEC* cmd;
	LLRP::CMessage* message = NULL;
	LLRP::CENABLE_ROSPEC_RESPONSE* response;

	cmd = new LLRP::CENABLE_ROSPEC();
	cmd->setMessageID(4);
	cmd->setROSpecID(RO_SPEC_ID);

	message = transact(cmd);

	delete cmd;

	if(message != NULL){
		response = (LLRP::CENABLE_ROSPEC_RESPONSE*)message;
		if(checkLLRPStatus(response->getLLRPStatus(), "enableROSpec") != 0){
			result = RN_NONE;
		}
		delete message;
	} else {
		result = RN_NONE;
	}
	
	return result;
}

int RNRFIdentificationTask::enableAccessSpecification(){
	int result = 0;

	return result;
}

int RNRFIdentificationTask::startROSpec(void){
	int result = 0;
	LLRP::CSTART_ROSPEC* cmd;
	LLRP::CMessage* message = NULL;
	LLRP::CSTART_ROSPEC_RESPONSE* response;

	cmd = new LLRP::CSTART_ROSPEC();
	cmd->setMessageID(5);
	cmd->setROSpecID(RO_SPEC_ID);

	message = transact(cmd);

	delete cmd;

	if(message != NULL){
		response = (LLRP::CSTART_ROSPEC_RESPONSE*)message;
		if(checkLLRPStatus(response->getLLRPStatus(), "startROSpec") != 0){
			result = RN_NONE;
		}
		delete message;
	} else {
		result = RN_NONE;
	}
	
	return result;
}

int RNRFIdentificationTask::getDataFromDevice(std::string& data){
	int result = 0;
	data = "";
	const LLRP::CTypeDescriptor* typeDesc = NULL;
	LLRP::CRO_ACCESS_REPORT* report = NULL;

	LLRP::CMessage* message = NULL;

	LLRP::CREADER_EVENT_NOTIFICATION* ntf = NULL;
    LLRP::CReaderEventNotificationData* ntfData = NULL;

	message = recvMessage(7000);

	if(message != NULL){
		typeDesc = message->m_pType;
		if(typeDesc == &LLRP::CRO_ACCESS_REPORT::s_typeDescriptor){
			report = (LLRP::CRO_ACCESS_REPORT*)message;
			int index = 0;
			for(std::list<LLRP::CTagReportData*>::iterator it = report->beginTagReportData(); it != report->endTagReportData(); it++){
				std::string oneTag;
				getOneTagData(*it, oneTag);
				if((it + 1) != report->endTagReportData()){
					data += oneTag + ";";
				} else {
					data += oneTag;
				}
			}

		} else if(typeDesc == &LLRP::CREADER_EVENT_NOTIFICATION::s_typeDescriptor){
			//future
			ntf = (LLRP::CREADER_EVENT_NOTIFICATION*)message;
			ntfData = ntf->getReaderEventNotificationData();
			if(ntfData != NULL){
				LLRP::CAntennaEvent* antennaEvent;
   				LLRP::CReaderExceptionEvent* readerExceptionEvent;
   				int eventsReported = 0;

   				antennaEvent = ntfData->getAntennaEvent();
   				if(antennaEvent != NULL){
   					handleAntennaEvent(antennaEvent);
   					eventsReported++;
   				}

   				readerExceptionEvent = ntfData->getReaderExceptionEvent();
   				if(readerExceptionEvent != NULL){
   					handleReaderExceptionEvent(readerExceptionEvent);
   					eventsReported++;
   				}
			}
			
		} else {
			RNUtils::printLn("WARNING: Ignored unexpected message during monitor: %s", typeDesc->m_pName);
		}
		delete message;
	} else {
		result = RN_NONE;
	}
	return result;
}

void RNRFIdentificationTask::getOneTagData(LLRP::CTagReportData* tag, std::string& data){
	const LLRP::CTypeDescriptor* type;
	std::ostringstream bufferOut;
	bufferOut.clear();

	LLRP::CParameter* epcParameter = tag->getEPCParameter();
	if(epcParameter != NULL){
        LLRP::llrp_u8_t* value = NULL;
        int length = 0;

        type = epcParameter->m_pType;
        if(type == &LLRP::CEPC_96::s_typeDescriptor){
        	length = 12;
        	LLRP::CEPC_96* epc96 = (LLRP::CEPC_96*)epcParameter;
            value = epc96->getEPC().m_aValue;

        } else if(type == &LLRP::CEPCData::s_typeDescriptor){
        	LLRP::CEPCData* epcData = (LLRP::CEPCData *)epcParameter;
            value = epcData->getEPC().m_pValue;
            length = (epcData->getEPC().m_nBit + 7u) / 8u;
        }

        if(value != NULL){
        	char* bufferIn = new char[2 * length + 5];
        	memset(bufferIn, 0, 2 * length + 5);
        	int index = 0;
        	for(int i = 0; i < length; i++){
        		if(i > 0 and ((i % 2) == 0)){
        			sprintf(bufferIn + index++, "%c", '-');
        		}
        		sprintf(bufferIn + index, "%02x", value[i]);
        		index += 2;
        	}
        	bufferOut << bufferIn;
        	delete[] bufferIn;
        	//RNUtils::printLn("epc: %s", bufferIn);
        	
        }
        bufferOut << ",";
        if(tag->getAntennaID() != NULL){
        	bufferOut << tag->getAntennaID()->getAntennaID();
        	//RNUtils::printLn("antenna: %d", tag->getAntennaID()->getAntennaID());
        }
        bufferOut << ",";
        if(tag->getPeakRSSI() != NULL){
        	bufferOut << (int)tag->getPeakRSSI()->getPeakRSSI();
        	//RNUtils::printLn("Peak RSSI: %d", tag->getPeakRSSI()->getPeakRSSI());
        }
        bufferOut << ",";
        if(tag->getChannelIndex() != NULL){
        	bufferOut << tag->getChannelIndex()->getChannelIndex();
        	//RNUtils::printLn("ChannelIndex: %d", tag->getChannelIndex()->getChannelIndex());
        }
        bufferOut << ",";
        if(tag->getFirstSeenTimestampUTC() != NULL){
        	bufferOut << tag->getFirstSeenTimestampUTC()->getMicroseconds();
        	//RNUtils::printLn("FSeen-T TS Uptime: %llu", tag->getFirstSeenTimestampUTC()->getMicroseconds());
        }
        bufferOut << ",";
        if(tag->getLastSeenTimestampUTC() != NULL){
        	bufferOut << tag->getLastSeenTimestampUTC()->getMicroseconds();
        	//RNUtils::printLn("LSeen-T TS Uptime: %llu", tag->getLastSeenTimestampUTC()->getMicroseconds());
        }
	}
	
	data = bufferOut.str();
	//RNUtils::printLn("Final STR: %s, %d", data, length);
	bufferOut.clear();
}

void RNRFIdentificationTask::handleAntennaEvent(LLRP::CAntennaEvent* antennaEvent){
	LLRP::EAntennaEventType eventType;
    LLRP::llrp_u16_t antennaID;
    std::string status;

    eventType = antennaEvent->getEventType();
    antennaID = antennaEvent->getAntennaID();

    switch(eventType){
	    case LLRP::AntennaEventType_Antenna_Disconnected:
	        status = "disconnected";
	        break;

	    case LLRP::AntennaEventType_Antenna_Connected:
	        status = "connected";
	        break;

	    default:
	        status = "unknown-event";
	        break;
    }
    RNUtils::printLn("NOTICE: Antenna %d is %s", antennaID, status.c_str());
}

void RNRFIdentificationTask::handleReaderExceptionEvent(LLRP::CReaderExceptionEvent* readerExceptionEvent){
	LLRP::llrp_utf8v_t message;

    message = readerExceptionEvent->getMessage();
    if(message.m_nValue > 0 and message.m_pValue != NULL){
        RNUtils::printLn("NOTICE: ReaderException '%.*s'", message.m_nValue, message.m_pValue);
    } else {
        RNUtils::printLn("NOTICE: ReaderException but no message");
    }
}

int RNRFIdentificationTask::resetToDefaultConfiguration(void){
	int result = 0;
	LLRP::CSET_READER_CONFIG* cmd;
	LLRP::CMessage* message = NULL;
	LLRP::CSET_READER_CONFIG_RESPONSE* response;

	cmd = new LLRP::CSET_READER_CONFIG();
	cmd->setMessageID(2);
	cmd->setResetToFactoryDefault(1);

	message = transact(cmd);

	delete cmd;

	if(message != NULL){
		response = (LLRP::CSET_READER_CONFIG_RESPONSE*)message;
		if(checkLLRPStatus(response->getLLRPStatus(), "resetToDefaultConfiguration") != 0){
			result = RN_NONE;
		}
		delete message;
	} else {
		result = RN_NONE;
	}
	
	return result;	
}

int RNRFIdentificationTask::checkLLRPStatus(LLRP::CLLRPStatus* status, const char* what){
	int result = 0;
	if(status != NULL){
		if(LLRP::StatusCode_M_Success != status->getStatusCode()){
			LLRP::llrp_utf8v_t errorDesc;
			errorDesc = status->getErrorDescription();
			if(errorDesc.m_nValue == 0) {
            	RNUtils::printLn("ERROR: %s failed, no error description given\n", what);
        	} else {
        		RNUtils::printLn("ERROR: %s failed, %.*s", what, errorDesc.m_nValue, errorDesc.m_pValue);
        	}
		}
	} else {
		result = RN_NONE;
		RNUtils::printLn("Error: Missing LLRP status (%s)...", what);
	}
	return result;
}

int RNRFIdentificationTask::checkConnectionStatus(void){
	int result = 0;
	LLRP::CMessage* message = NULL;
	LLRP::CREADER_EVENT_NOTIFICATION* ntf;
	LLRP::CReaderEventNotificationData* ntfData;
    LLRP::CConnectionAttemptEvent* event;

    message = recvMessage(10000);
    if(message != NULL){
    	if(&LLRP::CREADER_EVENT_NOTIFICATION::s_typeDescriptor == message->m_pType){
    		ntf = (LLRP::CREADER_EVENT_NOTIFICATION*)message;
    		ntfData = ntf->getReaderEventNotificationData();
    		if(ntfData != NULL){
    			event = ntfData->getConnectionAttemptEvent();
    			if(event != NULL){
    				if(LLRP::ConnectionAttemptStatusType_Success != event->getStatus()){
    					result = RN_NONE;
    				}
    			} else {
    				result == RN_NONE;
    			}
    		} else {
    			result = RN_NONE;
    		}
    	} else {
    		result = RN_NONE;
    	}
    } else {
    	result = RN_NONE;
    }

	return result;
}

LLRP::CMessage* RNRFIdentificationTask::transact(LLRP::CMessage* msg){
	LLRP::CMessage* message = NULL;
	if(readerDescriptor == 0){
		message = conn->transact(msg, 5000);
		if(message == NULL){
			const LLRP::CErrorDetails* error = conn->getRecvError();
			RNUtils::printLn("Error: Receive Message failed (%s)", error->m_pWhatStr ? error->m_pWhatStr : "No reason");
		}
	}
	return message;
}

LLRP::CMessage* RNRFIdentificationTask::recvMessage(int msecMax){
	LLRP::CMessage* message = NULL;
	if(readerDescriptor == 0){
		message = conn->recvMessage(msecMax);
		if(message == NULL){
			const LLRP::CErrorDetails* error = conn->getRecvError();
			RNUtils::printLn("Error: Receive Message failed (%s)", error->m_pWhatStr ? error->m_pWhatStr : "No reason");
		}
	}
	return message;
}

float RNRFIdentificationTask::convertToMeters(float rssi){
	float result = 0;
	if(rssi != 0){
		float ratio = rssi / RFData::OFFSET;
		if(ratio < 1.0){
			result = std::pow(ratio, 10);
		} else {
			result = .89976 * std::pow(ratio, 7.7095) + .111;
		}
	}
	return result;
}

RFData* RNRFIdentificationTask::findRFIDByKey(std::string key){
	RFData* rfid = NULL;
	bool found = false;
	for (int i = 0; i < rfids->size() and not found; i++){
		if (rfids->at(i)->getTagKey() == key){
			found = true;
			rfid = rfids->at(i);
		}
	}
	return rfid;
}
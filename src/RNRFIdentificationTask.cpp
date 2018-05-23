#include "RNRFIdentificationTask.h"

const unsigned int RNRFIdentificationTask::RF_BUFFER_SIZE = 10240;
const unsigned int RNRFIdentificationTask::RO_SPEC_ID = 1111;
const char* RNRFIdentificationTask::DEVICE_NAME = "192.168.0.20";
//const char* RNRFIdentificationTask::DEVICE_NAME = "speedwayr-11-94-a3.local";
const unsigned int RNRFIdentificationTask::ANTENNAS_NUMBER = 2;

RNRFIdentificationTask::RNRFIdentificationTask(const GeneralController* gn, const char* name, const char* description) : RNRecurrentTask(gn, name, description){
	this->gn = (GeneralController*)gn;
	conn = NULL;
	messageId = 0;
	readerDescriptor = RN_NONE;
	this->deviceInitialized = false;
	landmarks = new std::vector<RNLandmark*>();
	rfids = new std::vector<RFData*>();
	powerIndexAntenna1 = 1;
	antennasList = new AntennaDataList();
	//file = std::fopen("data-rfid-1.00m.txt", "w+");
	
}

RNRFIdentificationTask::~RNRFIdentificationTask(){
	if(conn != NULL){
		if(readerDescriptor == 0){
			if(stopROSpec() == 0){
				RNUtils::printLn("Success: Enabled readers operation specifications on %s OK...", DEVICE_NAME);
			}
			if(resetToDefaultConfiguration() == 0){
				RNUtils::printLn("Success: resetConfiguration on %s OK...", DEVICE_NAME);
			}
			conn->closeConnectionToReader();
			RNUtils::printLn("Disconnected from RF Speedway Reader...");
		}
		delete conn;
	}
	/*if(!file){
		std::fclose(file);
	}*/
}

void RNRFIdentificationTask::task(){
	if(this->deviceInitialized){
		std::string data = "";
		//if(startROSpec() == 0){
			if(getDataFromDevice(data) == 0){
				
				/*RFData* detected = new RFData(data);
				RFData* found = NULL;
				if((found = findByKeyAntenna(detected->getTagKey(), detected->getAntenna())) == NULL){
					if(detected->getTagKey() != ""){
						rfids->push_back(detected);	
					}
				} else {
					found->update(detected->getRSSI(), detected->getPhaseAngle(), detected->getDopplerFrequency());
					found->setTimestamp(detected->getTimestamp());
					delete detected;
				}*/
			}
			
			for (int i = 0; i < rfids->size(); ++i){
				RNUtils::printLn("[%d]: %s", i, rfids->at(i)->toString());
			}
		//}	
	} else {
		init();
	}	
}

void RNRFIdentificationTask::onKilled(){
	if(this){
		this->deviceInitialized = false;	
	}
	
}

int RNRFIdentificationTask::init(void){
	int result = 0;
	if(connectTo(DEVICE_NAME) == 0){
		if(checkConnectionStatus() == 0){
			RNUtils::printLn("Success: Connection OK...", DEVICE_NAME);
			if(/*enableImpinjExtensions()*/0 == 0){
				RNUtils::printLn("Success: Enable Impinj Extensions on %s OK...", DEVICE_NAME);
				//if(0 == 0){
				if(resetToDefaultConfiguration() == 0){
					RNUtils::printLn("Success: Reset default Configuration on %s OK...", DEVICE_NAME);
					//setReaderConfiguration();
					if(addROSpec() == 0){ 
						RNUtils::printLn("Added readers operation specifications on %s OK...", DEVICE_NAME);
						if(getReaderConfiguration() == 0){
							RNUtils::printLn("Success: Read Configuration on %s OK...", DEVICE_NAME);
							if(enableROSpec() == 0){
								
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
		//LLRP::enrollImpinjTypesIntoRegistry(typeRegistry);
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
	LLRP::CGeneralDeviceCapabilities *deviceCap;
	std::list<LLRP::CTransmitPowerLevelTableEntry*>::iterator pwrLvlIt;

	cmd = new LLRP::CGET_READER_CAPABILITIES();
	cmd->setMessageID(messageId++);
	cmd->setRequestedData(LLRP::GetReaderCapabilitiesRequestedData_All);

	message = transact(cmd);
	delete cmd;
	if(message != NULL){
		response = (LLRP::CGET_READER_CAPABILITIES_RESPONSE*)message;
		if(checkLLRPStatus(response->getLLRPStatus(), "getReaderCapabilities") == 0){
			if((regCap = response->getRegulatoryCapabilities()) != NULL){
				if((uhfCap = regCap->getUHFBandCapabilities()) != NULL){
					pwrLvlIt = uhfCap->endTransmitPowerLevelTableEntry();
					pwrLvlIt--;
					pwrLvl = *pwrLvlIt;

					this->powerLevelIndex = pwrLvl->getIndex();
					this->transmitPowerValue = pwrLvl->getTransmitPowerValue();
					if((deviceCap = response->getGeneralDeviceCapabilities()) != NULL){
						if(deviceCap->getDeviceManufacturerName() == 25882){
							this->deviceModelNumber = deviceCap->getModelName();

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
		} else {
			result = RN_NONE;
		}
		delete message;
	} else {
		result = RN_NONE;
	}
	return result;
}

int RNRFIdentificationTask::getReaderConfiguration(){
	int result = 0;
	LLRP::CGET_READER_CONFIG* cmd;
	LLRP::CMessage* message = NULL;
	LLRP::CGET_READER_CONFIG_RESPONSE* response;

	std::list<LLRP::CAntennaConfiguration*>::iterator antCnfgIt;
	std::list<LLRP::CAntennaProperties*>::iterator antPropIt;

	cmd = new LLRP::CGET_READER_CONFIG();
	cmd->setMessageID(messageId++);
	cmd->setRequestedData(LLRP::GetReaderConfigRequestedData_All);

	message = transact(cmd);
	delete cmd;
	if(message != NULL){
		response = (LLRP::CGET_READER_CONFIG_RESPONSE*)message;
		if(checkLLRPStatus(response->getLLRPStatus(), "getReaderConfiguration") == 0){
			for(antCnfgIt = response->beginAntennaConfiguration(); antCnfgIt != response->endAntennaConfiguration(); antCnfgIt++){

				AntennaData* antData = new AntennaData((*antCnfgIt)->getAntennaID());
				LLRP::CRFTransmitter* ttx = (*antCnfgIt)->getRFTransmitter();
				antData->setTxHopTableId(ttx->getHopTableID());
				antData->setTxchannelIndex(ttx->getChannelIndex());
				antData->setTxPower(ttx->getTransmitPower());

				LLRP::CRFReceiver* rrx = (*antCnfgIt)->getRFReceiver();
				antData->setRxSensitivity(rrx->getReceiverSensitivity());
				bool antFound = false;
				for(antPropIt = response->beginAntennaProperties(); antPropIt != response->endAntennaProperties() and not antFound; antPropIt++){
					if((*antPropIt)->getAntennaID() == (*antCnfgIt)->getAntennaID()){
						antFound = true;
						antData->setAntennaGain((*antPropIt)->getAntennaGain());
					}
				}
				//std::cout << antData->toString() << std::endl;
				antennasList->add(antData);
			}
		} else {
			result = RN_NONE;
		}
		delete message;
	} else {
		result = RN_NONE;
	}
	return result;
}

int RNRFIdentificationTask::setReaderConfiguration(){
	int result = 0;
	LLRP::CGET_READER_CONFIG* cmdRead;
	LLRP::CGET_READER_CONFIG_RESPONSE* responseRead;

	LLRP::CSET_READER_CONFIG* cmdSet;
	LLRP::CMessage* message = NULL;
	LLRP::CMessage* messageSet = NULL;
	LLRP::CSET_READER_CONFIG_RESPONSE* response;
	std::list<LLRP::CAntennaConfiguration*>::iterator antCnfgIt;

	cmdRead = new LLRP::CGET_READER_CONFIG();
	cmdRead->setMessageID(messageId++);
	cmdRead->setRequestedData(LLRP::GetReaderConfigRequestedData_All);

	message = transact(cmdRead);
	delete cmdRead;

	if(message != NULL){
		responseRead = (LLRP::CGET_READER_CONFIG_RESPONSE*)message;

		cmdSet = new LLRP::CSET_READER_CONFIG();
		cmdSet->setMessageID(messageId++);
		for(antCnfgIt = responseRead->beginAntennaConfiguration(); antCnfgIt != responseRead->endAntennaConfiguration(); antCnfgIt++){
			LLRP::CRFTransmitter* ttx = (*antCnfgIt)->getRFTransmitter();
			//ttx->setHopTableID(0);
			ttx->setChannelIndex(3);
			//ttx->setTransmitPower(81);
			cmdSet->addAntennaConfiguration(new LLRP::CAntennaConfiguration(*(*antCnfgIt)));
		}
		messageSet = transact(cmdSet);
		if(messageSet != NULL){
			response = (LLRP::CSET_READER_CONFIG_RESPONSE*)messageSet;
			if(checkLLRPStatus(response->getLLRPStatus(), "setReaderConfiguration") != 0){
				result = RN_NONE;
				printf("Todo mal... mu mal\n");
			} else {
				printf("Todo bien, todo correcto, y yo que me alegro\n");
			}

		} else {
			printf("El transact ha fallao\n");
		}
	}


	delete response;
	return result;
}

int RNRFIdentificationTask::deleteAllROSpecs(void){
	int result = 0;
	LLRP::CDELETE_ROSPEC* cmd;
	LLRP::CMessage* message = NULL;
	LLRP::CDELETE_ROSPEC_RESPONSE* response;

	cmd = new LLRP::CDELETE_ROSPEC();
	cmd->setMessageID(messageId++);
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
    rospecStartTrigger->setROSpecStartTriggerType(LLRP::ROSpecStartTriggerType_Immediate);

    LLRP::CROSpecStopTrigger* rospecStopTrigger = new LLRP::CROSpecStopTrigger();
    rospecStopTrigger->setROSpecStopTriggerType(LLRP::ROSpecStopTriggerType_Null);
    rospecStopTrigger->setDurationTriggerValue(0);     /* n/a */

    LLRP::CROBoundarySpec* roBoundarySpec = new LLRP::CROBoundarySpec();
    roBoundarySpec->setROSpecStartTrigger(rospecStartTrigger);
    roBoundarySpec->setROSpecStopTrigger(rospecStopTrigger);

    LLRP::CROReportSpec* roReportSpec = new LLRP::CROReportSpec();
    roReportSpec->setROReportTrigger(LLRP::ROReportTriggerType_Upon_N_Tags_Or_End_Of_ROSpec);
    roReportSpec->setN(1);         /* Unlimited */

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

    LLRP::CC1G2EPCMemorySelector* c1g2Memory = new LLRP::CC1G2EPCMemorySelector();
    c1g2Memory->setEnableCRC(false);
    c1g2Memory->setEnablePCBits(false);
    tagReportContentSelector->addAirProtocolEPCMemorySelector(c1g2Memory);

    roReportSpec->setTagReportContentSelector(tagReportContentSelector);

    LLRP::CAISpecStopTrigger* aiSpecStopTrigger = new LLRP::CAISpecStopTrigger();
    aiSpecStopTrigger->setAISpecStopTriggerType(LLRP::AISpecStopTriggerType_Null);
    aiSpecStopTrigger->setDurationTrigger(0);

    LLRP::CInventoryParameterSpec* inventoryParameterSpec = new LLRP::CInventoryParameterSpec();
    inventoryParameterSpec->setInventoryParameterSpecID(1);
    inventoryParameterSpec->setProtocolID(LLRP::AirProtocols_EPCGlobalClass1Gen2);

    for(int i = 1; i <= ANTENNAS_NUMBER; i++){
    	LLRP::CC1G2RFControl* pC1G2RFControl = new LLRP::CC1G2RFControl();
        pC1G2RFControl->setModeIndex(2);

        // Session
        LLRP::CC1G2SingulationControl* pC1G2SingulationControl = new LLRP::CC1G2SingulationControl();
        pC1G2SingulationControl->setSession(2);
        pC1G2SingulationControl->setTagPopulation(32);

        // Inventory search mode
        //LLRP::CImpinjInventorySearchMode *pImpIsm = new LLRP::CImpinjInventorySearchMode();
        //pImpIsm->setInventorySearchMode(LLRP::ImpinjInventorySearchType_Dual_Target);

        // C1G2InventoryCommand
        LLRP::CC1G2InventoryCommand* pC1G2InventoryCommand = new LLRP::CC1G2InventoryCommand();
        pC1G2InventoryCommand->setC1G2RFControl(pC1G2RFControl);
        pC1G2InventoryCommand->setC1G2SingulationControl(pC1G2SingulationControl);
        //pC1G2InventoryCommand->addCustom(pImpIsm);
        
        // Transmitter
        LLRP::CRFTransmitter* pRFTransmitter = new LLRP::CRFTransmitter();
        pRFTransmitter->setHopTableID(1);
        pRFTransmitter->setChannelIndex(1);
        
        // Receiver
        LLRP::CRFReceiver* pRFReceiver = new LLRP::CRFReceiver();

        // Set transmit power and receive sensitivity
        // for each antenna
        switch (i)
        {
            case 1:
            	powerIndexAntenna1 = 40;
                pRFTransmitter->setTransmitPower(powerIndexAntenna1); // (value * .25) + 10.0 = -30.25 dBm // max power when using PoE
                pRFReceiver->setReceiverSensitivity(42); // 1 --> -80 dBm;
                break;
            case 2:
            	powerIndexAntenna2 = 60;
                pRFTransmitter->setTransmitPower(powerIndexAntenna2); // (value * .25) + 10.0 = -27.75 dBm
                pRFReceiver->setReceiverSensitivity(42); // 10 dBm + 3 dBm = 13 dBm + (- 80 dBm) = -67 dBm
                break;
        }

        // Antenna config
        LLRP::CAntennaConfiguration* pAntennaConfig = new LLRP::CAntennaConfiguration();
        pAntennaConfig->setAntennaID(i);
        pAntennaConfig->setRFTransmitter(pRFTransmitter);
        pAntennaConfig->setRFReceiver(pRFReceiver);
        pAntennaConfig->addAirProtocolInventoryCommandSettings(pC1G2InventoryCommand);
        inventoryParameterSpec->addAntennaConfiguration(pAntennaConfig);
    }

    LLRP::llrp_u16v_t antennaIDs = LLRP::llrp_u16v_t(1);
    antennaIDs.m_pValue[0] = 0;     

    LLRP::CAISpec* aiSpec = new LLRP::CAISpec();
    aiSpec->setAntennaIDs(antennaIDs);
    aiSpec->setAISpecStopTrigger(aiSpecStopTrigger);
    aiSpec->addInventoryParameterSpec(inventoryParameterSpec);
   
   
    /*LLRP::CImpinjTagReportContentSelector* impinjTagCnt = new LLRP::CImpinjTagReportContentSelector();
    
    LLRP::CImpinjEnableRFPhaseAngle* impinjPhaseAngle = new LLRP::CImpinjEnableRFPhaseAngle();
    impinjPhaseAngle->setRFPhaseAngleMode(LLRP::ImpinjRFPhaseAngleMode_Enabled);

    LLRP::CImpinjEnablePeakRSSI* impinjPeakRssi= new LLRP::CImpinjEnablePeakRSSI();
    impinjPeakRssi->setPeakRSSIMode(LLRP::ImpinjPeakRSSIMode_Enabled);

    LLRP::CImpinjEnableRFDopplerFrequency* impinjDopplerFreq= new LLRP::CImpinjEnableRFDopplerFrequency();
    impinjDopplerFreq->setRFDopplerFrequencyMode(LLRP::ImpinjRFDopplerFrequencyMode_Enabled);

    LLRP::CImpinjEnableSerializedTID* impinjSerializedTID = new LLRP::CImpinjEnableSerializedTID();
    impinjSerializedTID->setSerializedTIDMode(LLRP::ImpinjSerializedTIDMode_Disabled);


    impinjTagCnt->setImpinjEnableRFPhaseAngle(impinjPhaseAngle);
    impinjTagCnt->setImpinjEnablePeakRSSI(impinjPeakRssi);
    impinjTagCnt->setImpinjEnableRFDopplerFrequency(impinjDopplerFreq);
    impinjTagCnt->setImpinjEnableSerializedTID(impinjSerializedTID);

    roReportSpec->addCustom(impinjTagCnt);*/

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
	cmd->setMessageID(messageId++);
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
	cmd->setMessageID(messageId++);
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

int RNRFIdentificationTask::enableImpinjExtensions(){
	int result = 0;
	LLRP::CIMPINJ_ENABLE_EXTENSIONS* cmd;
	LLRP::CMessage* message = NULL;
	LLRP::CIMPINJ_ENABLE_EXTENSIONS_RESPONSE* response;

	cmd = new LLRP::CIMPINJ_ENABLE_EXTENSIONS();
	cmd->setMessageID(messageId++);
	
	message = transact(cmd);
	RNUtils::printLn("\nqui to ta bien");
	delete cmd;
	
	if(message != NULL){
		response = (LLRP::CIMPINJ_ENABLE_EXTENSIONS_RESPONSE*)message;
		if(checkLLRPStatus(response->getLLRPStatus(), "enableImpinjExtensions") != 0){
			result = RN_NONE;
		}
		delete message;
	} else {
		result = RN_NONE;
	}
	return result;
}

int RNRFIdentificationTask::startROSpec(void){
	int result = 0;
	LLRP::CSTART_ROSPEC* cmd;
	LLRP::CMessage* message = NULL;
	LLRP::CSTART_ROSPEC_RESPONSE* response;

	cmd = new LLRP::CSTART_ROSPEC();
	cmd->setMessageID(messageId++);
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

int RNRFIdentificationTask::stopROSpec(void){
	int result = 0;
	LLRP::CSTOP_ROSPEC* cmd;
	LLRP::CMessage* message = NULL;
	LLRP::CSTOP_ROSPEC_RESPONSE* response;

	cmd = new LLRP::CSTOP_ROSPEC();
	cmd->setMessageID(messageId++);
	cmd->setROSpecID(RO_SPEC_ID);

	message = transact(cmd);

	delete cmd;

	if(message != NULL){
		response = (LLRP::CSTOP_ROSPEC_RESPONSE*)message;
		if(checkLLRPStatus(response->getLLRPStatus(), "stopROSpec") != 0){
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

	message = recvMessage(1500);

	if(message != NULL){
		typeDesc = message->m_pType;
		if(typeDesc == &LLRP::CRO_ACCESS_REPORT::s_typeDescriptor){
			report = (LLRP::CRO_ACCESS_REPORT*)message;
			int index = 0;
			for(std::list<LLRP::CTagReportData*>::iterator it = report->beginTagReportData(); it != report->endTagReportData(); it++){
				std::string oneTag;
				getOneTagData(*it, oneTag);
				//RNUtils::printLn("Y aqui cuanto tarda");
				data += oneTag + ";";
			}
			data = data.substr(0, data.size() - 1);
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
        int phaseAngleValue = 0, rssiValue = 0, dopplerFrequencyValue = 0;
        /*for(std::list<LLRP::CParameter *>::iterator customIt = tag->beginCustom(); customIt != tag->endCustom(); customIt++){
        	if((*customIt)->m_pType == &LLRP::CImpinjRFPhaseAngle::s_typeDescriptor){
        		LLRP::CImpinjRFPhaseAngle* phaseAngle = (LLRP::CImpinjRFPhaseAngle*)(*customIt);
        		phaseAngleValue = phaseAngle->getPhaseAngle();
        		//RNUtils::printLn("phaseAngle: %d", phaseAngleValue);
        	} else if((*customIt)->m_pType == &LLRP::CImpinjPeakRSSI::s_typeDescriptor){
        		LLRP::CImpinjPeakRSSI* rssi = (LLRP::CImpinjPeakRSSI*)(*customIt);
        		rssiValue = rssi->getRSSI();
        		//RNUtils::printLn("rssi: %d", rssiValue);
        	} else if((*customIt)->m_pType == &LLRP::CImpinjRFDopplerFrequency::s_typeDescriptor){
        		LLRP::CImpinjRFDopplerFrequency* dopplerFrequency = (LLRP::CImpinjRFDopplerFrequency*)(*customIt);
        		dopplerFrequencyValue = dopplerFrequency->getDopplerFrequency();
        		//RNUtils::printLn("rssi: %d", rssiValue);
        	} 
        }*/

        //std::fprintf(file, "%d,%f,%f,%d\n", tag->getAntennaID()->getAntennaID(), (((float)rssiValue) / 100.0), (((float)phaseAngleValue) * 2 * M_PI / 4096.0), dopplerFrequencyValue);

        if(tag->getAntennaID() != NULL){
        	bufferOut << tag->getAntennaID()->getAntennaID();
        	//RNUtils::printLn("antenna: %d", tag->getAntennaID()->getAntennaID());
        }
        bufferOut << ",";

        if(rssiValue != 0){
        	bufferOut << ((float)rssiValue) / 100.0;
        } else if(tag->getPeakRSSI() != NULL){
        	bufferOut << (int)tag->getPeakRSSI()->getPeakRSSI();
        	
        	//RNUtils::printLn("Peak RSSI: %d", tag->getPeakRSSI()->getPeakRSSI());
        }
        bufferOut << ",";

        if(phaseAngleValue != 0){
        	bufferOut << ((float)phaseAngleValue) * 2 * M_PI / 4096.0;
        }
        bufferOut << ",";

        if(dopplerFrequencyValue != 0){
        	bufferOut << dopplerFrequencyValue;
        }
        bufferOut << ",";

        /*if(tag->getChannelIndex() != NULL){
        	bufferOut << tag->getChannelIndex()->getChannelIndex();
        	//RNUtils::printLn("ChannelIndex: %d", tag->getChannelIndex()->getChannelIndex());
        }
        bufferOut << ",";*/

        if(tag->getFirstSeenTimestampUTC() != NULL){
        	bufferOut << tag->getFirstSeenTimestampUTC()->getMicroseconds();
        	//RNUtils::printLn("FSeen-T TS Uptime: %llu", tag->getFirstSeenTimestampUTC()->getMicroseconds());
        }
        //bufferOut << ",";

        /*if(tag->getLastSeenTimestampUTC() != NULL){
        	bufferOut << tag->getLastSeenTimestampUTC()->getMicroseconds();
        	//RNUtils::printLn("LSeen-T TS Uptime: %llu", tag->getLastSeenTimestampUTC()->getMicroseconds());
        }*/
	}
	
	data = bufferOut.str();
	RNUtils::printLn("Final STR: %s, %d", data.c_str(), data.length());
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
	cmd->setMessageID(messageId++);
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
    		if(ntf){
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
    } else {
    	result = RN_NONE;
    }

	return result;
}

LLRP::CMessage* RNRFIdentificationTask::transact(LLRP::CMessage* msg, int timeout){
	LLRP::CMessage* message = NULL;
	if(readerDescriptor == 0){
		message = conn->transact(msg, timeout);
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
			if(NULL != error->m_pRefType){
	            RNUtils::printLn("ERROR: ... reference type %s\n", error->m_pRefType->m_pName);
	        }

	      	if(NULL != error->m_pRefField){
	            RNUtils::printLn("ERROR: ... reference field %s\n", error->m_pRefField->m_pName);
	        }
		}
	}
	return message;
}

RFData* RNRFIdentificationTask::findByKeyAntenna(std::string key, int antenna){
	RFData* rfid = NULL;
	bool found = false;
	for (int i = 0; i < rfids->size() and not found; i++){
		if (rfids->at(i)->getTagKey() == key and rfids->at(i)->getAntenna() == antenna){
			found = true;
			rfid = rfids->at(i);
		}
	}
	return rfid;
}

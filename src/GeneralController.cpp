#include "GeneralController.h"

#include "RNRecurrentTask.h"
#include "RNRecurrentTaskMap.h"
#include "RNLocalizationTask.h"
#include "RNKalmanLocalizationTask.h"
#include "RNPFLocalizationTask.h"
#include "RNCameraTask.h"
#include "RNOmnicameraTask.h"
#include "RNRFIdentificationTask.h"
#include "RNEmotionsTask.h"
#include "RNDialogsTask.h"
#include "RNGesturesTask.h"
#include "RNGlobalLocalizationTask.h"

const float GeneralController::LASER_MAX_RANGE = 11.6;
const float GeneralController::LANDMARK_RADIUS = 0.045;

const double AntennaData::TX_GAIN = 5.7;
const double AntennaData::FREQUENCY = 866.9e6;
const double AntennaData::C = 3e8;
const float AntennaData::TX_POWER_OFFSET_DBM = -10.0;
const float AntennaData::TX_POWER_INDEX_MULTIPLIER = 0.25;

GeneralController::GeneralController(const char* port):RobotNode(port){
	
	//this->maestroControllers = new SerialPort();

	tokenRequester = RN_NONE;
	this->setTokenOwner(RN_NONE);

	clientsConnected = 0;
	pendingTransferControl = false;

	this->frontBumpersOk = true;
	this->rearBumpersOk = true;
	this->setChargerPosition = false;
	this->hasAchievedGoal = false;
	this->streamingActive = RN_NO;
	this->keepRobotTracking = RN_NO;
	this->spdUDPPort = 0;

	this->laserSensorActivated = true;
	this->cameraSensorActivated = false;
	this->rfidSensorActivated = false;

    this->lastSiteVisitedIndex = RN_NONE;
	laserLandmarks = new RNLandmarkList();
	visualLandmarks = new RNLandmarkList();
	kalmanFuzzy = new std::vector<fl::Trapezoid*>();
	
	robotState = Matrix(3, 1);
	robotEncoderPosition = Matrix(3, 1);
	robotVelocity = Matrix (2, 1);

	robotRawEncoderPosition = Matrix(3, 1);
	robotRawDeltaPosition = Matrix (2, 1);
	
	P = Matrix(3, 3);
	Q = Matrix(2, 2);
	R = Matrix(3, 3);
	spdUDPClient = NULL;

	xmlFaceFullPath = RNUtils::getApplicationPath() + XML_FILE_PATH;
	xmlMapsFullPath = RNUtils::getApplicationPath() + XML_FILE_MAPS_PATH;
	xmlSectorsPath = RNUtils::getApplicationPath() + XML_FILE_SECTORS_PATH;
	xmlRobotConfigFullPath = RNUtils::getApplicationPath() + XML_FILE_ROBOT_CONFIG_PATH;

	//std::string servo_positions;
	//setGesture("26", servo_positions);
	//ttsLipSync = new DorisLipSync(this->maestroControllers, RNUtils::getApplicationPath());
	//ttsLipSync->textToViseme("Hola, ya estoy lista para funcionar");
	this->currentMapId = RN_NONE;
	this->currentSector = NULL;
	this->nextSectorId = RN_NONE;

	loadRobotConfig();
	pthread_mutex_init(&laserLandmarksLocker, NULL);
	pthread_mutex_init(&rfidLandmarksLocker, NULL);
	pthread_mutex_init(&visualLandmarksLocker, NULL);

	tasks = new RNRecurrentTaskMap(this);

	omnidirectionalTask = new RNOmnicameraTask("Omnidirectional Task");
	//globalLocalization = new RNGlobalLocalizationTask();
	//dialogs = new RNDialogsTask(ttsLipSync);
	//gestures = new RNGesturesTask(this->maestroControllers);
	//emotions = new RNEmotionsTask();
	//eyesCameras = new RNCameraTask();

	if(robotConfig->localization == XML_LOCALIZATION_ALGORITHM_KALMAN_STR){
		localization = new RNKalmanLocalizationTask();
	} else if(robotConfig->localization == XML_LOCALIZATION_ALGORITHM_PF_STR){
		localization = new RNPFLocalizationTask();
	}
	//rfidTask = new RNRFIdentificationTask();

	////Tasks added:
	//tasks->addTask(globalLocalization);
	tasks->addTask(omnidirectionalTask);
	//tasks->addTask(dialogs);
	//tasks->addTask(gestures);
	//tasks->addTask(emotions);
	tasks->addTask(localization);
	//tasks->addTask(eyesCameras);
	//tasks->addTask(rfidTask);
	
	//Start all tasks;
	tasks->startAllTasks();
	printed = false;
	virtualFace = NULL;
	if(RNUtils::isVirtualFaceActivated()){
		std::string ip = "";
		int port = RN_NONE;
		RNUtils::getVirtualFaceIpPort(ip, port);
		if(port != RN_NONE){
			virtualFace = new RNVirtualFace();
			virtualFace->init(ip.c_str(), port);
			virtualFace->startThread();
			RNUtils::printLn("Connected to Virtual-Face: %s over %d", ip.c_str(), port);
		}
		
	}
	
	spdWSServer = new RobotDataStreamer();
	spdWSServer->init("", 0, SOCKET_SERVER);
	spdWSServer->startThread();

	RNUtils::getTimestamp(emotionsTimestamp);
	RNUtils::getTimestamp(mappingSectorTimestamp);
	RNUtils::getTimestamp(mappingLandmarksTimestamp);
	RNUtils::getTimestamp(mappingFeaturesTimestamp);
	RNUtils::getTimestamp(mappingSitesTimestamp);
	//file = std::fopen("data-laser.txt", "w+");
}

GeneralController::~GeneralController(void){

	//rfidConnection->closeConnection();

	closeConnection();
	
	//pthread_mutex_destroy(&laserLandmarksLocker);
	//pthread_mutex_destroy(&rfidLandmarksLocker);
	//pthread_mutex_destroy(&visualLandmarksLocker);
	stopDynamicGesture();
	stopCurrentTour();
	disconnect();

	delete tasks;
	RNUtils::printLn("Deleted tasks...");
	for (int i = 0; i < kalmanFuzzy->size(); i++){
		delete kalmanFuzzy->at(i);
	}
	kalmanFuzzy->clear();
	delete kalmanFuzzy;
	RNUtils::printLn("Deleted kalmanFuzzy...");
	delete ttsLipSync;
	RNUtils::printLn("Deleted ttsLipSync...");
	delete maestroControllers;
	RNUtils::printLn("Deleted maestroControllers...");
	delete spdWSServer;
	RNUtils::printLn("Deleted spdWSServer...");
	delete currentSector;
	RNUtils::printLn("Deleted currentSector...");
	delete robotConfig;
	RNUtils::printLn("Deleted robotConfig...");

	delete laserLandmarks;
	RNUtils::printLn("Deleted laserLandmarks...");

	if(!file){
		std::fclose(file);
	}
}

void GeneralController::onConnection(int socketIndex){ //callback for client and server
	if(isConnected(socketIndex)) {
		clientsConnected++;
		RNUtils::printLn("Client %s is Connected to Doris, using port %d", this->getClientIPAddress(socketIndex), this->getClientPort(socketIndex));
	} else {
		if(socketIndex == getTokenOwner()){
			setTokenOwner(RN_NONE);
		}
		RNUtils::printLn("Client %s has disonnected from Doris", this->getClientIPAddress(socketIndex));
		clientsConnected--;
		if(clientsConnected == 0){
			setTokenOwner(RN_NONE);
			if(spdUDPClient != NULL){
				spdUDPClient->closeConnection();
				spdUDPClient = NULL;
			}
			stopDynamicGesture();
			stopCurrentTour();
		}
	}
	RNUtils::printLn("Clients connected: %d", clientsConnected);
}
void GeneralController::onMsg(int socketIndex, char* cad, unsigned long long int length){//callback for client and server
	cad[length] = 0;
	unsigned char function = *(cad++);
	std::string local_buffer_out = "";
	std::string gestures = "";
	std::string servo_positions = "";
	std::string mapsAvailable = "";
	std::string sectorsAvailable = "";
	std::string mapInformation = "";
	std::string sectorInformation = "";
	std::string jsonRobotOpSuccess = "{\"robot\":{\"error\":\"None.\"}}";
	std::string jsonControlError = "{\"robot\":{\"error\":\"Permission denied.\"}}";
	std::string jsonSectorNotLoadedError = "{\"robot\":{\"error\":\"No sector loaded.\"}}";
	std::ostringstream number_converter;
	
	unsigned char port = 0;
	unsigned char card_id = 0;

	int servo_position = 0;
	int face_id, k=0;
	int mapId = 0;
	int sectorId = 0;
	float lin_vel = 0, ang_vel = 0;
	int cameraCount = 0;
	int videoDevice = 0;
	int videoTxPort = 0;
	int indexAssigned = 0;
	float x, y, theta;

	bool granted = isPermissionNeeded(function) && (socketIndex == getTokenOwner());
	RNUtils::printLn("Function: 0x%x (%d) executed from: %s", function, function, getClientIPAddress(socketIndex));
	switch (function){
		case 0x00:
			RNUtils::printLn("Command 0x00. Static Faces Requested");
			getGestures("0", gestures);
			sendMsg(socketIndex, 0x00, (char*)gestures.c_str(), (unsigned int)(gestures.length())); 
			break;
		case 0x01:
			RNUtils::printLn("Command 0x01. Dynamic Faces Requested");
			getGestures("1", gestures);
			sendMsg(socketIndex, 0x01, (char*)gestures.c_str(), (unsigned int)(gestures.length()));
			break;
		case 0x02:
			RNUtils::printLn("Command 0x02. Saving New Static Face");
			saveGesture(cad, 0);
			break;
		case 0x03:
			RNUtils::printLn("Command 0x03. Saving New Dynamic Face");
			saveGesture(cad, 1);
			break;
		case 0x04:
			RNUtils::printLn("Command 0x04. Modifying Static Face");
			if(granted){
				modifyGesture(cad, 0);
			} else {
				RNUtils::printLn("Command 0x04. Modifying Static Face denied to %s", getClientIPAddress(socketIndex));
				sendMsg(socketIndex, 0x04, (char*)jsonControlError.c_str(), (unsigned int)jsonControlError.length());
			}
			break;
		case 0x05:
			RNUtils::printLn("Command 0x05. Modifying Dynamic Face");
			if(granted){
				modifyGesture(cad, 1);
			} else {
				RNUtils::printLn("Command 0x05. Modifying Dynamic Face denied to %s", getClientIPAddress(socketIndex));
				sendMsg(socketIndex, 0x05, (char*)jsonControlError.c_str(), (unsigned int)jsonControlError.length());
			}
			break;
		case 0x06:
			RNUtils::printLn("Command 0x06. Removing Face");
			if(granted){
				removeGesture(cad);
			} else {
				RNUtils::printLn("Command 0x06. Removing Face denied to %s", getClientIPAddress(socketIndex));
				sendMsg(socketIndex, 0x06, (char*)jsonControlError.c_str(), (unsigned int)jsonControlError.length());
			}
			break;
		case 0x07:
			RNUtils::printLn("Command 0x07. Setting Gesture Id: %s", cad);
			if(granted){
				gestures = "";
				setGesture(cad, servo_positions);
				sendMsg(socketIndex, 0x07, (char*)servo_positions.c_str(), (unsigned int)servo_positions.length());
			} else {
				RNUtils::printLn("Command 0x07. Setting Gesture denied to %s", getClientIPAddress(socketIndex));
				sendMsg(socketIndex, 0x07, (char*)jsonControlError.c_str(), (unsigned int)jsonControlError.length());
			}
			break;
		case 0x08:
            getPololuInstruction(cad, card_id, port, servo_position);            
			setServoPosition(card_id, port, servo_position);
			break;
		case 0x09:
			RNUtils::printLn("Command 0x09. Sending current positions");
			//SendServoPositions(servo_positions);
			//SendMsg(socketIndex, 0x09, (char*)servo_positions.c_str(), (int)(servo_positions.length()));
			break;
		case 0x0A:
			RNUtils::printLn("Command 0x0A. Stopping any Dynamic Face");
			if(granted){
				stopDynamicGesture();
			} else {
				RNUtils::printLn("Command 0x0A. Stopping any Dynamic Face denied to %s", getClientIPAddress(socketIndex));
				sendMsg(socketIndex, 0x0A, (char*)jsonControlError.c_str(), (unsigned int)jsonControlError.length());
			}
			break;
		case 0x0B:
			RNUtils::printLn("Command 0x0B. Text to speech message");
			if(granted){
				dialogs->setInputMessage(cad);
				//ttsLipSync->textToViseme(cad);
				sendMsg(socketIndex, 0x08, (char*)jsonRobotOpSuccess.c_str(), (unsigned int)jsonRobotOpSuccess.length());
			} else {
				RNUtils::printLn("Command 0x0B. Text to speech message denied to %s", getClientIPAddress(socketIndex));
				sendMsg(socketIndex, 0x0B, (char*)jsonControlError.c_str(), (unsigned int)jsonControlError.length());
			}
			break;
		case 0x0C:
			break;
		case 0x10:
			if(granted){
				getVelocities(cad, lin_vel, ang_vel);
				moveRobot(lin_vel, ang_vel);
			} else {
				RNUtils::printLn("Command 0x10. Robot teleoperation denied to %s", getClientIPAddress(socketIndex));
				sendMsg(socketIndex, 0x10, (char*)jsonControlError.c_str(), (unsigned int)jsonControlError.length());
			}

			break;
		case 0x11:

			if(granted){
				if(currentSector != NULL){
					startSitesTour();
				} else {
					RNUtils::printLn("Command 0x11. No current sector available to start tour to ", getClientIPAddress(socketIndex));
					sendMsg(socketIndex, 0x11, (char*)jsonSectorNotLoadedError.c_str(), (unsigned int)jsonSectorNotLoadedError.length());
				}
			} else {
				RNUtils::printLn("Command 0x11. Start robot tour denied to %s", getClientIPAddress(socketIndex));
				sendMsg(socketIndex, 0x11, (char*)jsonControlError.c_str(), (unsigned int)jsonControlError.length());
			}
			break;
		case 0x12:
			if(granted){
				if(currentSector != NULL){
					stopCurrentTour();
				} else {
					RNUtils::printLn("Command 0x12. No current sector available to stop tour to ", getClientIPAddress(socketIndex));
					sendMsg(socketIndex, 0x12, (char*)jsonSectorNotLoadedError.c_str(), (unsigned int)jsonSectorNotLoadedError.length());
				}
			} else {
				RNUtils::printLn("Command 0x12. Stop robot tour denied to %s", getClientIPAddress(socketIndex));
				sendMsg(socketIndex, 0x12, (char*)jsonControlError.c_str(), (unsigned int)jsonControlError.length());
			}
			break;	
		case 0x13:
			if(granted){
				getPositions(cad, x, y, theta);
				if(localization != NULL){
					localization->kill();
				}
				if(currentSector != NULL){
					setRobotPosition(x, y, theta);
					if(localization != NULL){
						localization->reset();
					}
					sendMsg(socketIndex, 0x13, (char*)jsonRobotOpSuccess.c_str(), (unsigned int)jsonRobotOpSuccess.length());
				} else {
					RNUtils::printLn("Command 0x13. No current sector available to set robot position to ", getClientIPAddress(socketIndex));
					sendMsg(socketIndex, 0x13, (char*)jsonSectorNotLoadedError.c_str(), (unsigned int)jsonSectorNotLoadedError.length());
				}
			} else {
				RNUtils::printLn("Command 0x13. Set Robot position denied to %s", getClientIPAddress(socketIndex));
				sendMsg(socketIndex, 0x13, (char*)jsonControlError.c_str(), (unsigned int)jsonControlError.length());
			}
			
			break;
		case 0x14:
			if(granted){
				getPositions(cad, x, y, theta);
				moveRobotToPosition(x, y, theta);
				sendMsg(socketIndex, 0x14, (char*)jsonRobotOpSuccess.c_str(), (unsigned int)jsonRobotOpSuccess.length());
			} else {
				RNUtils::printLn("Command 0x14. Moving Robot to position denied to %s", getClientIPAddress(socketIndex));
				sendMsg(socketIndex, 0x14, (char*)jsonControlError.c_str(), (unsigned int)jsonControlError.length());
			}
			break;
		case 0x15:
			getMapId(cad, mapId);
			getSectorsAvailable(mapId, sectorsAvailable);
			sendMsg(socketIndex, 0x15, (char*)sectorsAvailable.c_str(), (unsigned int)sectorsAvailable.length()); 
			break;
		case 0x16:
			if(granted){
				getMapSectorId(cad, mapId, sectorId);
				this->currentMapId = mapId;
				if(localization != NULL){
					localization->kill();	
				}
				
				loadSector(mapId, sectorId);
				if(localization != NULL){
					localization->reset();	
				}
				
				sendMsg(socketIndex, 0x16, (char*)jsonRobotOpSuccess.c_str(), (unsigned int)jsonRobotOpSuccess.length());
			} else {
				RNUtils::printLn("Command 0x16. Setting Map into Robot denied to %s", getClientIPAddress(socketIndex));
				sendMsg(socketIndex, 0x16, (char*)jsonControlError.c_str(), (unsigned int)jsonControlError.length());
			}
			break;
		case 0x17:
			if(cad != NULL){
				getMapSectorId(cad, mapId, sectorId);
			} else {
				sectorId = currentSector->getId();
				mapId = this->currentMapId;
			}
			getSectorInformationLandmarks(mapId, sectorId, sectorInformation);
			sendMsg(socketIndex, 0x17, (char*)sectorInformation.c_str(), (unsigned int)sectorInformation.length()); 
			break;
		case 0x18:
			if(cad != NULL){
				getMapSectorId(cad, mapId, sectorId);
			} else {
				sectorId = currentSector->getId();
				mapId = this->currentMapId;
			}
			getSectorInformationFeatures(mapId, sectorId, sectorInformation);
			sendMsg(socketIndex, 0x18, (char*)sectorInformation.c_str(), (unsigned int)sectorInformation.length()); 
			break;
		case 0x19:
			if(cad != NULL){
				getMapSectorId(cad, mapId, sectorId);
			} else {
				sectorId = currentSector->getId();
				mapId = this->currentMapId;
			}
			getSectorInformationSites(mapId, sectorId, sectorInformation);
			sendMsg(socketIndex, 0x19, (char*)sectorInformation.c_str(), (unsigned int)sectorInformation.length()); 
			break;
		case 0x1A:
			if(cad != NULL){
				getMapSectorId(cad, mapId, sectorId);
			} else {
				sectorId = currentSector->getId();
				mapId = this->currentMapId;
			}
			getSectorInformationSitesSequence(mapId, sectorId, sectorInformation);
			sendMsg(socketIndex, 0x1A, (char*)sectorInformation.c_str(), (unsigned int)sectorInformation.length()); 
			break;
		case 0x1B:
			//function to add point of interest
			if(granted){
				addSectorInformationSite(cad, indexAssigned);
				number_converter << indexAssigned;
				jsonRobotOpSuccess = "{\"robot\":{\"index\":\"" + number_converter.str() + "\",\"error\":\"0\"}}";
				sendMsg(socketIndex, 0x1B, (char*)jsonRobotOpSuccess.c_str(), (unsigned int)jsonRobotOpSuccess.length());
			} else {
				RNUtils::printLn("Command 0x1B. Adding site to map denied to %s", getClientIPAddress(socketIndex));
				sendMsg(socketIndex, 0x1B, (char*)jsonControlError.c_str(), (unsigned int)jsonControlError.length());
			}
			break;
		case 0x1C:
			//function to modify point of interest
			if(granted){
				modifySectorInformationSite(cad);
				sendMsg(socketIndex, 0x1C, (char*)jsonRobotOpSuccess.c_str(), (unsigned int)jsonRobotOpSuccess.length());
			} else {
				RNUtils::printLn("Command 0x1C. Modifying site to map denied to %s", getClientIPAddress(socketIndex));
				sendMsg(socketIndex, 0x1C, (char*)jsonControlError.c_str(), (unsigned int)jsonControlError.length());
			}
			break;
		case 0x1D:
			//function to delete point of interest
			if(granted){
				deleteSectorInformationSite(cad);
				sendMsg(socketIndex, 0x1D, (char*)jsonRobotOpSuccess.c_str(), (unsigned int)jsonRobotOpSuccess.length());
			} else {
				RNUtils::printLn("Command 0x1D. Delete site to map denied to %s", getClientIPAddress(socketIndex));
				sendMsg(socketIndex, 0x1D, (char*)jsonControlError.c_str(), (unsigned int)jsonControlError.length());
			}
			break;
		case 0x1E:
			//function to modify the execution sequence
			if(granted){
				setSitesExecutionSequence(cad);
			} else {
				RNUtils::printLn("Command 0x1E. Setting sites sequence denied to %s", getClientIPAddress(socketIndex));
				sendMsg(socketIndex, 0x1E, (char*)jsonControlError.c_str(), (unsigned int)jsonControlError.length());
			}
			break;
		case 0x1F:
			//function to add feature
			if(granted){
				addSectorInformationFeatures(cad, indexAssigned);
				number_converter << indexAssigned;
				jsonRobotOpSuccess = "{\"robot\":{\"index\":\"" + number_converter.str() + "\",\"error\":\"0\"}}";
				sendMsg(socketIndex, 0x1F, (char*)jsonRobotOpSuccess.c_str(), (unsigned int)jsonRobotOpSuccess.length());
			} else {
				RNUtils::printLn("Command 0x1F. Adding feature to map denied to %s", getClientIPAddress(socketIndex));
				sendMsg(socketIndex, 0x1F, (char*)jsonControlError.c_str(), (unsigned int)jsonControlError.length());
			}
			break;
		case 0x20:
			//function to modify feature
			if(granted){
				modifySectorInformationFeatures(cad);
				sendMsg(socketIndex, 0x20, (char*)jsonRobotOpSuccess.c_str(), (unsigned int)jsonRobotOpSuccess.length());
			} else {
				RNUtils::printLn("Command 0x20. Modifying feature to map denied to %s", getClientIPAddress(socketIndex));
				sendMsg(socketIndex, 0x20, (char*)jsonControlError.c_str(), (unsigned int)jsonControlError.length());
			}
			break;
		case 0x21:
			//function to delete feature
			if(granted){
				deleteSectorInformationFeatures(cad);
				sendMsg(socketIndex, 0x21, (char*)jsonRobotOpSuccess.c_str(), (unsigned int)jsonRobotOpSuccess.length());
			} else {
				RNUtils::printLn("Command 0x21. Delete feature to map denied to %s", getClientIPAddress(socketIndex));
				sendMsg(socketIndex, 0x21, (char*)jsonControlError.c_str(), (unsigned int)jsonControlError.length());
			}
			break;
		case 0x22:
			getMapsAvailable(mapsAvailable);
			sendMsg(socketIndex, 0x22, (char*)mapsAvailable.c_str(), (unsigned int)mapsAvailable.length()); 
			break;
		case 0x23:
			getMapId(cad, mapId);
			getMapConnection(mapId, mapInformation);
			sendMsg(socketIndex, 0x23, (char*)mapInformation.c_str(), (unsigned int)mapInformation.length()); 
			break;	
		case 0x30:
			
			break;
		case 0x31:
			
			break;
		case 0x7C:
			requestRobotControl(socketIndex);
			break;
		case 0x7D:
			if(granted){
				acceptTransferRobotControl(socketIndex, cad);
			} else {
				RNUtils::printLn("Command 0x7D. Accept transfering robot control denied to %s", getClientIPAddress(socketIndex));
				sendMsg(socketIndex, 0x7D, (char*)jsonControlError.c_str(), (unsigned int)jsonControlError.length());
			}
			break;
		case 0x7E:
			if(granted){
				releaseRobotControl(socketIndex);
				setTokenOwner(RN_NONE);
			} else {
				RNUtils::printLn("Command 0x7E. Release Robot control denied to %s", getClientIPAddress(socketIndex));
				sendMsg(socketIndex, 0x7E, (char*)jsonControlError.c_str(), (unsigned int)jsonControlError.length());
			}

			break;
		case 0x7F:
			initializeSPDPort(socketIndex, cad);
			break;
		default:
			RNUtils::printLn("Command Not Recognized..");
			break;
			
	}
	
}

bool GeneralController::isPermissionNeeded(char function){
	bool result = false;
	xml_document<> doc;
	
	std::string buffer_str = "";
	
    std::ifstream the_file(xmlRobotConfigFullPath.c_str());
	
    std::vector<char> buffer((std::istreambuf_iterator<char>(the_file)), std::istreambuf_iterator<char>());
	buffer.push_back('\0');

	doc.parse<0>(&buffer[0]);
	xml_node<>* root_node = doc.first_node(XML_ELEMENT_ROBOT_STR);
	xml_node<>* permissions_node = root_node->first_node(XML_ELEMENT_PERMISSIONS_STR);
	bool found = false;
	for(xml_node<>* permision_node = permissions_node->first_node(XML_ELEMENT_PERMISSION_STR); permision_node and not found; permision_node = permision_node->next_sibling()){
		s_feature* tempFeature = new s_feature;

		char func = (int)strtol(permision_node->first_attribute(XML_ATTRIBUTE_FUNCTION_STR)->value(), NULL, 0);
		if(func == function){
			found = true;
			int tokenRequired = atoi(permision_node->first_attribute(XML_ATTRIBUTE_TOKEN_REQUIRED_STR)->value());
			result = tokenRequired == RN_YES ? true : false;
		}
		
	}
	the_file.close();
	return result;
}

void GeneralController::requestRobotControl(int socketIndex){
	std::string jsonString;
	if(getTokenOwner() == RN_NONE){
		setTokenOwner(socketIndex);
		jsonString = "{\"control\":{\"granted\":\"1\",\"error\":\"0\"}}";
		sendMsg(getTokenOwner(), 0x7D, (char*)jsonString.c_str(), (unsigned int)jsonString.length());
	} else {
		tokenRequester = socketIndex;
		jsonString = "{\"control\":{\"requester\":\"" + std::string(getClientIPAddress(socketIndex)) + "\",\"error\":\"0\"}}";
		sendMsg(getTokenOwner(), 0x7C, (char*)jsonString.c_str(), (unsigned int)jsonString.length());
	}

}
void GeneralController::acceptTransferRobotControl(int socketIndex, char* acceptance){
	int acceptValue = atoi(acceptance);
	std::string jsonString;
	if(tokenRequester != RN_NONE){
		if(acceptValue == RN_YES){
			setTokenOwner(tokenRequester);
			releaseRobotControl(socketIndex);
			tokenRequester = RN_NONE;
			jsonString = "{\"control\":{\"granted\":\"1\",\"error\":\"0\"}}";
			sendMsg(getTokenOwner(), 0x7D, (char*)jsonString.c_str(), (unsigned int)jsonString.length());
		} else if(acceptValue == RN_NO){
			jsonString = "{\"control\":{\"granted\":\"0\",\"error\":\"0\"}}";
			sendMsg(tokenRequester, 0x7D, (char*)jsonString.c_str(), (unsigned int)jsonString.length());
			tokenRequester = RN_NONE;
		} else {
			jsonString = "{\"control\":{\"error\":\"1\"}}";
			sendMsg(getTokenOwner(), 0x7D, (char*)jsonString.c_str(), (unsigned int)jsonString.length());
		}
	}
}

void GeneralController::releaseRobotControl(int socketIndex){
	std::string jsonString = "{\"control\":{\"released\":\"1\",\"error\":\"0\"}}";
	sendMsg(socketIndex, 0x7E, (char*)jsonString.c_str(), (unsigned int)jsonString.length());

}

void GeneralController::initializeSPDPort(int socketIndex, char* cad){
	this->spdUDPPort = atoi(cad);
	if(!this->isWebSocket(socketIndex)){
		if(this->spdUDPPort <= 0){
			throw std::invalid_argument("Error!!! Could not initialize SPD streaming");
		}
		RNUtils::printLn("Port: %d", this->spdUDPPort);
		spdUDPClient = new UDPClient(this->getClientIPAddress(socketIndex), this->spdUDPPort);	
	} else {
		RNUtils::printLn("Port: %d", spdWSServer->getServerPort());
		std::ostringstream convert;
		convert << spdWSServer->getServerPort();
		std::string jsonString = "{\"streaming\":{\"port\":\""+ convert.str() + "\",\"error\":\"0\"}}";
		sendMsg(socketIndex, 0x7F, (char*)jsonString.c_str(), (unsigned int)jsonString.length());
	}
	
}

void GeneralController::getMapSectorId(char* cad, int& mapId, int& sectorId){
	char* current_number;
	int values[2] = { -1, -1 };
	int index = 0;
	current_number = std::strtok(cad, ",");
	//
	while(current_number != NULL){
		values[index++] = std::atoi(current_number);
		current_number = std::strtok(NULL, ",");
	}
	mapId = values[0];
	sectorId = values[1];
	delete current_number;
}

void GeneralController::getPositions(char* cad, float& x, float& y, float& theta){
	char* current_number;
	float values[3] = { -1, -1, -1 };
	int index = 0;
	current_number = std::strtok(cad, ",");

	while(current_number != NULL){
		int cValue = std::atoi(current_number);
		values[index++] = (float)(((float)cValue)/1000.0);
		current_number = std::strtok(NULL, ",");
	}
	x = values[0];
	y = values[1];
	theta = values[2];
	delete current_number;
}

void GeneralController::getPololuInstruction(char* cad, unsigned char& card_id, unsigned char& servo_id, int& value){
	char* current_number;
	int values[3]  = { -1, -1, -1 };
	int index = 0;
	current_number = std::strtok(cad, ",");
	//
	while(current_number != NULL){
		values[index++] = std::atoi(current_number);
		current_number = std::strtok(NULL, ",");
	}
	card_id = (unsigned char)values[0];
	servo_id = (unsigned char)values[1];
	value = values[2];
	delete current_number;
}

void GeneralController::getVelocities(char* cad, float& lin_vel, float& ang_vel){
	char* current_number;
	float values[2] = { -1, -1 };
	int index = 0;
	current_number = std::strtok(cad, ",");
	
	while(current_number != NULL){
		int cValue = std::atoi(current_number);
		values[index++] = (float)(((float)cValue)/1000.0);
		current_number = std::strtok(NULL, ",");
	}
	lin_vel = values[0];
	ang_vel = values[1];
	delete current_number;
}

void GeneralController::getGestures(std::string type, std::string& gestures){
    xml_document<> doc;
    xml_node<>* root_node;
	
	std::string buffer_str = "";
	
    std::ifstream the_file(xmlFaceFullPath.c_str());
	
    std::vector<char> buffer((std::istreambuf_iterator<char>(the_file)), std::istreambuf_iterator<char>());
	buffer.push_back('\0');
	
	doc.parse<0>(&buffer[0]);
	root_node = doc.first_node(XML_STATIC_GESTURES_STR);
	if(root_node != NULL){
		for (xml_node<> * gesto_node = root_node->first_node(XML_ELEMENT_GESTURE_STR); gesto_node; gesto_node = gesto_node->next_sibling()){	
			std::string nombre(gesto_node->first_attribute(XML_ATTRIBUTE_NAME_STR)->value());
			
			std::string id(gesto_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value());

			std::string tipo(gesto_node->first_attribute(XML_ATTRIBUTE_TYPE_STR)->value());

			if (tipo == type){
				
				buffer_str += id + "," + nombre + "|";
			}
		}
	}
	gestures = buffer_str;
	the_file.close();
	
	
}

void GeneralController::saveGesture(std::string token, int type){
    if(type == 0){
		s_motor motor_positions[SERVOS_COUNT];
		int i = 0;
		std::string gesture_name(strtok((char*)token.c_str(), "|"));
		std::string gesture_positions(strtok(NULL, "|"));
				
		char* positions = strtok((char*)gesture_positions.c_str(), ",");
		while (positions != 0){
			std::ostringstream convert;
			convert << i;
			motor_positions[i].idMotor = convert.str();
			motor_positions[i++].pos = std::string(positions);
			positions = strtok(NULL, ",");
		}
		delete positions;
		saveStaticGesture(gesture_name, motor_positions);
    }
}

void GeneralController::modifyGesture(std::string token, int type){
    if(type == 0){
		s_motor motor_positions[SERVOS_COUNT];
		int i = 0;
		std::string gesture_id(strtok((char*)token.c_str(), "|"));
		std::string gesture_name(strtok(NULL, "|"));
		std::string gesture_positions(strtok(NULL, "|"));
		        
		char* positions = strtok((char*)gesture_positions.c_str(), ",");
		while (positions != 0){
	            std::ostringstream convert;
	            convert << i;
	                motor_positions[i].idMotor = convert.str();
			motor_positions[i++].pos = std::string(positions);
			positions = strtok(NULL, ",");
		}
		delete positions;
		modifyStaticGesture(gesture_id, gesture_name, motor_positions);
    }
}

void GeneralController::modifyStaticGesture(std::string gesture_id, std::string name, s_motor servos[]){
	xml_document<> doc;
    xml_node<>* root_node;
	
	std::string assigned_id;
	std::ostringstream convert;
        
	
    std::ifstream the_file(xmlFaceFullPath.c_str());
    std::vector<char> buffer((std::istreambuf_iterator<char>(the_file)), std::istreambuf_iterator<char>());
	buffer.push_back('\0');
	
	doc.parse<0>(&buffer[0]);
	
	root_node = doc.first_node(XML_STATIC_GESTURES_STR);
	if(root_node != NULL){
		xml_node<> * gesto_node = root_node->last_node(XML_ELEMENT_GESTURE_STR);
			
		std::string id(gesto_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value());

		convert << (atoi(id.c_str()) + 1);
		assigned_id = convert.str();
			
        xml_node<>* new_node_gesture = doc.allocate_node(node_element, XML_ELEMENT_GESTURE_STR);
        new_node_gesture->append_attribute(doc.allocate_attribute(XML_ATTRIBUTE_ID_STR, assigned_id.c_str()));
        new_node_gesture->append_attribute(doc.allocate_attribute(XML_ATTRIBUTE_NAME_STR, name.c_str()));
        new_node_gesture->append_attribute(doc.allocate_attribute(XML_ATTRIBUTE_TYPE_STR, "0"));
        int id_;
        for(id_ = 0; id_ < SERVOS_COUNT; id_++){
            xml_node<>* new_node_motor = doc.allocate_node(node_element, XML_ELEMENT_MOTOR_STR);

            new_node_motor->append_attribute(doc.allocate_attribute(XML_ATTRIBUTE_ID_STR, servos[id_].idMotor.c_str()));
            
            new_node_motor->append_attribute(doc.allocate_attribute(XML_ATTRIBUTE_POSITION_STR, servos[id_].pos.c_str()));
            new_node_gesture->append_node(new_node_motor);
        }
        
        root_node->append_node(new_node_gesture);
        the_file.close();
        
        std::ofstream the_new_file(xmlFaceFullPath.c_str());
        the_new_file << doc;
        the_new_file.close(); 
	}  
	     
}

void GeneralController::saveStaticGesture(std::string name, s_motor servos[]){
	xml_document<> doc;
    xml_node<>* root_node;
	
	std::string assigned_id;
	std::ostringstream convert;
        
	
    std::ifstream the_file(xmlFaceFullPath.c_str());
    std::vector<char> buffer((std::istreambuf_iterator<char>(the_file)), std::istreambuf_iterator<char>());
	buffer.push_back('\0');
	
	doc.parse<0>(&buffer[0]);
	
	root_node = doc.first_node(XML_STATIC_GESTURES_STR);
	if(root_node != NULL){
		xml_node<> * gesto_node = root_node->last_node(XML_ELEMENT_GESTURE_STR);
			
		std::string id(gesto_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value());

		convert << (atoi(id.c_str()) + 1);
		assigned_id = convert.str();
        
        xml_node<>* new_node_gesture = doc.allocate_node(node_element, XML_ELEMENT_GESTURE_STR);
        new_node_gesture->append_attribute(doc.allocate_attribute(XML_ATTRIBUTE_ID_STR, assigned_id.c_str()));
        new_node_gesture->append_attribute(doc.allocate_attribute(XML_ATTRIBUTE_NAME_STR, name.c_str()));
        new_node_gesture->append_attribute(doc.allocate_attribute(XML_ATTRIBUTE_TYPE_STR, "0"));
        int id_;
        for(id_ = 0; id_ < SERVOS_COUNT; id_++){
            xml_node<>* new_node_motor = doc.allocate_node(node_element, XML_ELEMENT_MOTOR_STR);

            new_node_motor->append_attribute(doc.allocate_attribute(XML_ATTRIBUTE_ID_STR, servos[id_].idMotor.c_str()));
            
            new_node_motor->append_attribute(doc.allocate_attribute(XML_ATTRIBUTE_POSITION_STR, servos[id_].pos.c_str()));
            new_node_gesture->append_node(new_node_motor);
        }
        
        root_node->append_node(new_node_gesture);
        the_file.close();
        
        std::ofstream the_new_file(xmlFaceFullPath.c_str());
        the_new_file << doc;
        the_new_file.close();    
	}
	
}

void GeneralController::saveDynamicGesture(std::string name, s_motor servos[]){

}

void GeneralController::removeGesture(std::string face_id){
    xml_document<> doc;
    xml_node<>* root_node;
	
	std::string buffer_str = "";
	
    std::ifstream the_file(xmlFaceFullPath.c_str());
    std::vector<char> buffer((std::istreambuf_iterator<char>(the_file)), std::istreambuf_iterator<char>());
	buffer.push_back('\0');
	
	doc.parse<0>(&buffer[0]);
	
	root_node = doc.first_node(XML_STATIC_GESTURES_STR);
	if(root_node != NULL){
        xml_node<>* node_to_find;
        bool found = false;
		for (xml_node<> * gesto_node = root_node->first_node(XML_ELEMENT_GESTURE_STR); gesto_node && !found; gesto_node = gesto_node->next_sibling()){	

			std::string id(gesto_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value());
			if(id == face_id){
				node_to_find = gesto_node;
				found = true;
			}
		}
			root_node->remove_node(node_to_find);
		the_file.close();
        
        std::ofstream the_new_file(xmlFaceFullPath.c_str());
        the_new_file << doc;
        the_new_file.close();
	}
	
}

void GeneralController::setGesture(std::string face_id, std::string& servo_positions){
    xml_document<> doc;
    xml_node<>* root_node;
	
	std::string buffer_str = "";
	std::ostringstream bufferOut_str;
	
    std::ifstream the_file(xmlFaceFullPath.c_str());
    std::vector<char> buffer((std::istreambuf_iterator<char>(the_file)), std::istreambuf_iterator<char>());
	buffer.push_back('\0');
	
	doc.parse<0>(&buffer[0]);
	stopDynamicGesture();
	root_node = doc.first_node(XML_STATIC_GESTURES_STR);
	if(root_node != NULL){
		for (xml_node<> * gesto_node = root_node->first_node(XML_ELEMENT_GESTURE_STR); gesto_node; gesto_node = gesto_node->next_sibling()){

			std::string id(gesto_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value());

			std::string tipo(gesto_node->first_attribute(XML_ATTRIBUTE_TYPE_STR)->value());
			
			if (face_id == id){
				int currentCardId = RN_NONE;
				std::string nombre(gesto_node->first_attribute(XML_ATTRIBUTE_NAME_STR)->value());

				bufferOut_str << "{\"Gesture\":\"" << nombre << "\",\"Id\":\"" << id << "\",\"Cards\":[";

				for(xml_node<> * motor_node = gesto_node->first_node(XML_ELEMENT_MOTOR_STR); motor_node; motor_node = motor_node->next_sibling()){
					unsigned char card_id = (unsigned char)atoi(motor_node->first_attribute(XML_ATTRIBUTE_CARD_ID_STR)->value());
					unsigned char servo_id = (unsigned char)atoi(motor_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value());
					int position = atoi(motor_node->first_attribute(XML_ATTRIBUTE_POSITION_STR)->value());
					int speed = atoi(motor_node->first_attribute(XML_ATTRIBUTE_SPEED_STR)->value());
					int acceleration = atoi(motor_node->first_attribute(XML_ATTRIBUTE_ACCELERATION_STR)->value());

					if(currentCardId != RN_NONE){
						if(currentCardId != card_id){
							currentCardId = card_id;
							bufferOut_str << "]},{\"CardId\":\"" << (int)card_id << "\",\"Motors\":[";
						}
					} else {
						if(currentCardId != card_id){
							currentCardId = card_id;
							bufferOut_str << "{\"CardId\":\"" << (int)card_id << "\",\"Motors\":[";
						}
					}
					bufferOut_str << "{\"MotorId\":\"" << (int)servo_id << "\",\"Position\":\"" << position << "\"}";
					if(motor_node->next_sibling() != NULL){
						unsigned char card_id_next = (unsigned char)atoi(motor_node->next_sibling()->first_attribute(XML_ATTRIBUTE_CARD_ID_STR)->value());
						if(card_id_next == card_id){
							bufferOut_str << ",";	
						}
						
					}

					//setServoPosition(card_id, servo_id, position);
					//setServoSpeed(card_id, servo_id, speed);
					//setServoAcceleration(card_id, servo_id, acceleration);
				}	
				

				if(tipo == "1"){
					RNUtils::printLn("This is a dynamic face");
					dynamic_face_info *data = new dynamic_face_info;
					data->object = this;
					data->id_gesto = face_id;
					
					pthread_t t1;
					continue_dynamic_thread = true;
					
					pthread_create(&t1, NULL, GeneralController::dynamicFaceThread, (void *)(data));
				}
			}
		}
	}
	the_file.close();
	servo_positions = bufferOut_str.str();
	if(virtualFace != NULL){
		virtualFace->sendBytes((unsigned char*)servo_positions.c_str(), servo_positions.length());	
	}
	
}

void GeneralController::setServoPosition(unsigned char card_id, unsigned char servo_id, int position){
    this->maestroControllers->setTarget(card_id, servo_id, position);
}

void GeneralController::setServoSpeed(unsigned char card_id, unsigned char servo_id, int speed){
    this->maestroControllers->setSpeed(card_id, servo_id, speed);
}

void GeneralController::setServoAcceleration(unsigned char card_id, unsigned char servo_id, int acceleration){
    this->maestroControllers->setAcceleration(card_id, servo_id, acceleration);
}

void GeneralController::stopDynamicGesture(){
	continue_dynamic_thread = false;
}

void* GeneralController::dynamicFaceThread(void* object){
    dynamic_face_info* node = (dynamic_face_info*)object;
	s_movement selected_dynamic_face;
	xml_document<> doc;
    xml_node<>* root_node;
	
	std::string buffer_str = "";
	
    std::ifstream the_file(node->object->xmlFaceFullPath.c_str());
    std::vector<char> buffer((std::istreambuf_iterator<char>(the_file)), std::istreambuf_iterator<char>());
	buffer.push_back('\0');
	
	doc.parse<0>(&buffer[0]);	
	root_node = doc.first_node(XML_DYNAMIC_GESTURES_STR);
	if(root_node != NULL){
		for (xml_node<> * repetir_node = root_node->first_node("Repetir"); repetir_node; repetir_node = repetir_node->next_sibling()){

			std::string idGesture(repetir_node->first_attribute("idGesto")->value());
			if(idGesture == node->id_gesto){
				selected_dynamic_face.idGesture = idGesture;
				
				std::string ts(repetir_node->first_attribute("ts")->value());
				selected_dynamic_face.ts = ts;

				for(xml_node<>* secuencia_node = repetir_node->first_node("Secuencia"); secuencia_node; secuencia_node = secuencia_node->next_sibling()){	    
					s_secuence secuence;   		
					
					std::string idSecuencia(secuencia_node->first_attribute("idSecuencia")->value());	
					secuence.idSecuence = idSecuencia;

					std::string tsec(secuencia_node->first_attribute("tsec")->value());
					secuence.tsec = tsec;
					
					for(xml_node<>* motor_node = secuencia_node->first_node(XML_ELEMENT_MOTOR_STR); motor_node; motor_node = motor_node->next_sibling()){
						s_motor motor;
						std::string cardId(motor_node->first_attribute(XML_ATTRIBUTE_CARD_ID_STR)->value());
						motor.cardId = cardId;
											
						std::string idMotor(motor_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value());
						motor.idMotor = idMotor; //el indice j no se corresponde con el numero de motor, ya que podria estar modificando el motor 3 y 4 y se guardarian en los indices 0 y 1 respectivamente
						
						std::string pos(motor_node->first_attribute(XML_ATTRIBUTE_POSITION_STR)->value());
						std::string speed(motor_node->first_attribute(XML_ATTRIBUTE_SPEED_STR)->value());
						std::string acceleration(motor_node->first_attribute(XML_ATTRIBUTE_ACCELERATION_STR)->value());
						motor.pos = pos; //el indice j no se corresponde con el numero de motor, ya que podria estar modificando el motor 3 y 4 y se guardarian en los indices 0 y 1 respectivamente
						motor.speed = speed;
						motor.acceleration = acceleration;
						secuence.motors.push_back(motor);
					}

					selected_dynamic_face.secuences.push_back(secuence);
				}
			}
		}
	}

	the_file.close();
	

	RNUtils::sleep(atoi(selected_dynamic_face.ts.c_str()));

	while(node->object->continue_dynamic_thread){
		for(int i=0; i<selected_dynamic_face.secuences.size(); i++){
			for(int j=0; j<selected_dynamic_face.secuences[i].motors.size(); j++){
                unsigned char card_id = (unsigned char)atoi(selected_dynamic_face.secuences[i].motors[j].cardId.c_str());
				unsigned char servo_id = (unsigned char)atoi(selected_dynamic_face.secuences[i].motors[j].idMotor.c_str());
				int position = atoi(selected_dynamic_face.secuences[i].motors[j].pos.c_str());
				int speed = atoi(selected_dynamic_face.secuences[i].motors[j].speed.c_str());
				int acceleration = atoi(selected_dynamic_face.secuences[i].motors[j].acceleration.c_str());
				
				node->object->setServoPosition(card_id, servo_id, position);
				node->object->setServoSpeed(card_id, servo_id, speed);
				node->object->setServoAcceleration(card_id, servo_id, acceleration);
			}
			
			RNUtils::sleep(atoi(selected_dynamic_face.secuences[i].tsec.c_str()));
		}
	}
	return NULL;
}

void GeneralController::getMapId(char* cad, int& mapId){
	mapId = atoi(cad);
}

void GeneralController::loadSector(int mapId, int sectorId){
	this->currentMapId = mapId;
	std::string filename;

	getMapFilename(mapId, filename);

	xml_document<> doc;
    xml_node<>* root_node;  

    std::string fullSectorPath = xmlSectorsPath + filename;
    std::ifstream the_file(fullSectorPath.c_str());
    std::vector<char> buffer = std::vector<char>((std::istreambuf_iterator<char>(the_file)), std::istreambuf_iterator<char>());
    buffer.push_back('\0');
    doc.parse<0>(&buffer[0]);

    if(currentSector != NULL){
    	delete currentSector;
    	currentSector = NULL;
    }

    root_node = doc.first_node(XML_ELEMENT_SECTORS_STR);
    currentSector = new MapSector;
    if(root_node != NULL){
    	bool found = false;
    	for (xml_node<> * sector_node = root_node->first_node(XML_ELEMENT_SECTOR_STR); sector_node and not found; sector_node = sector_node->next_sibling()){	
    		int xmlSectorId = atoi(sector_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value());
    		//std::cout << "hasta aqui bien" << std::endl;
    		if(xmlSectorId == sectorId){
    			found = true;
    			currentSector->setId(xmlSectorId);
				currentSector->setName(std::string(sector_node->first_attribute(XML_ATTRIBUTE_NAME_STR)->value()));
				currentSector->setPolygon(std::string(sector_node->first_attribute(XML_ATTRIBUTE_POLYGON_STR)->value()));
				currentSector->setWidth((float)atoi(sector_node->first_attribute(XML_ATTRIBUTE_WIDTH_STR)->value())/100);
				currentSector->setHeight((float)atoi(sector_node->first_attribute(XML_ATTRIBUTE_HEIGHT_STR)->value())/100);
				xml_node<>* landmarks_root_node = sector_node->first_node(XML_ELEMENT_LANDMARKS_STR);
				if(landmarks_root_node->first_node() !=  NULL){
					for(xml_node<>* landmark_node = landmarks_root_node->first_node(XML_ELEMENT_LANDMARK_STR); landmark_node; landmark_node = landmark_node->next_sibling()){
						s_landmark* tempLandmark = new s_landmark;

						tempLandmark->id = atoi(landmark_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value());
						tempLandmark->type = std::string(landmark_node->first_attribute(XML_ATTRIBUTE_TYPE_STR)->value());
						if(landmark_node->first_attribute(XML_ATTRIBUTE_VARIANCE_MIN_X_STR)){
							tempLandmark->varMinX = ((float)atoi(landmark_node->first_attribute(XML_ATTRIBUTE_VARIANCE_MIN_X_STR)->value())) / 100;
						}
						if(landmark_node->first_attribute(XML_ATTRIBUTE_VARIANCE_MAX_X_STR)){
							tempLandmark->varMaxX = ((float)atoi(landmark_node->first_attribute(XML_ATTRIBUTE_VARIANCE_MAX_X_STR)->value())) / 100;
						}
						if(landmark_node->first_attribute(XML_ATTRIBUTE_VARIANCE_MIN_Y_STR)){
							tempLandmark->varMinY = ((float)atoi(landmark_node->first_attribute(XML_ATTRIBUTE_VARIANCE_MIN_Y_STR)->value())) / 100;
						}
						if(landmark_node->first_attribute(XML_ATTRIBUTE_VARIANCE_MAX_Y_STR)){
							tempLandmark->varMaxY = ((float)atoi(landmark_node->first_attribute(XML_ATTRIBUTE_VARIANCE_MAX_Y_STR)->value())) / 100;
						}
						if(landmark_node->first_attribute(XML_ATTRIBUTE_VARIANCE_MIN_Z_STR)){
							tempLandmark->varMinZ = ((float)atoi(landmark_node->first_attribute(XML_ATTRIBUTE_VARIANCE_MIN_Z_STR)->value())) / 100;
						}
						if(landmark_node->first_attribute(XML_ATTRIBUTE_VARIANCE_MAX_Z_STR)){
							tempLandmark->varMaxZ = ((float)atoi(landmark_node->first_attribute(XML_ATTRIBUTE_VARIANCE_MAX_Z_STR)->value())) / 100;
						}	

						tempLandmark->xpos = ((float)atoi(landmark_node->first_attribute(XML_ATTRIBUTE_X_POSITION_STR)->value())) / 100;
						tempLandmark->ypos = ((float)atoi(landmark_node->first_attribute(XML_ATTRIBUTE_Y_POSITION_STR)->value())) / 100;
						if(landmark_node->first_attribute(XML_ATTRIBUTE_Z_POSITION_STR)){
							tempLandmark->zpos = ((float)atoi(landmark_node->first_attribute(XML_ATTRIBUTE_Z_POSITION_STR)->value())) / 100;
						}
						currentSector->addLandmark(tempLandmark);
					}
				}

				xml_node<>* features_root_node = sector_node->first_node(XML_ELEMENT_FEATURES_STR);
				if(features_root_node->first_node() !=  NULL){
					for(xml_node<>* features_node = features_root_node->first_node(XML_ELEMENT_FEATURE_STR); features_node; features_node = features_node->next_sibling()){
						s_feature* tempFeature = new s_feature;

						tempFeature->id = atoi(features_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value());
						tempFeature->name = std::string(features_node->first_attribute(XML_ATTRIBUTE_NAME_STR)->value());
						tempFeature->width = ((float)atoi(features_node->first_attribute(XML_ATTRIBUTE_WIDTH_STR)->value())) / 100;
						tempFeature->height = ((float)atoi(features_node->first_attribute(XML_ATTRIBUTE_HEIGHT_STR)->value())) / 100;
						tempFeature->xpos = ((float)atoi(features_node->first_attribute(XML_ATTRIBUTE_X_POSITION_STR)->value())) / 100;
						tempFeature->ypos = ((float)atoi(features_node->first_attribute(XML_ATTRIBUTE_Y_POSITION_STR)->value())) / 100;

						if(features_node->first_attribute(XML_ATTRIBUTE_LINKED_SECTOR_ID_STR)){
							tempFeature->linkedSectorId = atoi(features_node->first_attribute(XML_ATTRIBUTE_LINKED_SECTOR_ID_STR)->value());
						} else {
							tempFeature->linkedSectorId = RN_NONE;
						}
                        
                        if(features_node->first_attribute(XML_ATTRIBUTE_X_COORD_STR)){
                            tempFeature->xcoord = ((float)atoi(features_node->first_attribute(XML_ATTRIBUTE_X_COORD_STR)->value())) / 100;
                        } else {
                            tempFeature->xcoord = 0.0;
                        }
                        
                        if(features_node->first_attribute(XML_ATTRIBUTE_Y_COORD_STR)){
                            tempFeature->ycoord = ((float)atoi(features_node->first_attribute(XML_ATTRIBUTE_Y_COORD_STR)->value())) / 100;
                        } else {
                            tempFeature->ycoord = 0.0;
                        }
                        
                        currentSector->addFeature(tempFeature);
						
					}
				}

				xml_node<>* sites_root_node = sector_node->first_node(XML_ELEMENT_SITES_STR);
				if(sites_root_node->first_attribute()){
					if(sites_root_node->first_attribute(XML_ATTRIBUTE_CYCLIC_STR)){
						std::string cyclic(sites_root_node->first_attribute(XML_ATTRIBUTE_CYCLIC_STR)->value());
						if(cyclic == "yes"){
							currentSector->setIfSitesCyclic(true);
						}
					}

					if(sites_root_node->first_attribute(XML_ATTRIBUTE_SEQUENCE_STR)){
						std::string sequence(sites_root_node->first_attribute(XML_ATTRIBUTE_SEQUENCE_STR)->value());
						currentSector->setSequence(sequence);
					}
				}
				
				if(sites_root_node->first_node() !=  NULL){
					for(xml_node<>* site_node = sites_root_node->first_node(XML_ELEMENT_SITE_STR); site_node; site_node = site_node->next_sibling()){
						s_site* tempSite = new s_site;

						tempSite->id = atoi(site_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value());
						tempSite->name = std::string(site_node->first_attribute(XML_ATTRIBUTE_NAME_STR)->value());
						tempSite->radius = ((float)atoi(site_node->first_attribute(XML_ATTRIBUTE_RADIUS_STR)->value())) / 100;
						tempSite->tsec = ((float)atoi(site_node->first_attribute(XML_ATTRIBUTE_TIME_STR)->value()));
						tempSite->xpos = ((float)atoi(site_node->first_attribute(XML_ATTRIBUTE_X_POSITION_STR)->value())) / 100;
						tempSite->ypos = ((float)atoi(site_node->first_attribute(XML_ATTRIBUTE_Y_POSITION_STR)->value())) / 100;
						if(site_node->first_attribute(XML_ATTRIBUTE_LINKED_FEATURE_ID_STR)){
							tempSite->linkedFeatureId = atoi(site_node->first_attribute(XML_ATTRIBUTE_LINKED_FEATURE_ID_STR)->value());
						} else {
							tempSite->linkedFeatureId = RN_NONE;
						}

						currentSector->addSite(tempSite);
					}
				}
    		}
    	}

    }
    RNUtils::getTimestamp(mappingSectorTimestamp);
    RNUtils::printLn("Loaded new Sector {id: %d, name: %s}", sectorId, currentSector->getName().c_str());
    the_file.close();

}

void GeneralController::loadRobotConfig(){
	xml_document<> doc;
	
	std::string buffer_str = "";
	
    std::ifstream the_file(xmlRobotConfigFullPath.c_str());
	
    std::vector<char> buffer((std::istreambuf_iterator<char>(the_file)), std::istreambuf_iterator<char>());
	buffer.push_back('\0');

	doc.parse<0>(&buffer[0]);

	if(robotConfig != NULL){
		delete robotConfig;
	}
	robotConfig = new s_robot;

	xml_node<>* root_node = doc.first_node(XML_ELEMENT_ROBOT_STR);
	if(root_node->first_attribute(XML_ATTRIBUTE_ROBOT_HEIGHT_STR)){
		robotConfig->height = std::atof(root_node->first_attribute(XML_ATTRIBUTE_ROBOT_HEIGHT_STR)->value());
	}
	if(root_node->first_attribute(XML_ATTRIBUTE_LOCALIZATION_STR)){
		robotConfig->localization = std::string(root_node->first_attribute(XML_ATTRIBUTE_LOCALIZATION_STR)->value());
	} else {
		robotConfig->localization = std::string(XML_LOCALIZATION_ALGORITHM_KALMAN_STR);
	}
	xml_node<>* nav_root_node = root_node->first_node(XML_ELEMENT_NAV_PARAMS_STR);
	robotConfig->navParams->alpha = (float)atoi(nav_root_node->first_attribute(XML_ATTRIBUTE_ALPHA_STR)->value()) / 100;

	xml_node<>* initial_pos_root_node = nav_root_node->first_node(XML_ELEMENT_INITIAL_POS_STR);
	xml_node<>* pos_root_node = initial_pos_root_node->first_node(XML_ELEMENT_POS_X_ZONE_STR);
	s_trapezoid* x_zone = robotConfig->navParams->initialPosition->xZone;
	x_zone->x1 = (float)atoi(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X1_STR)->value()) / 100;
	x_zone->x2 = (float)atoi(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X2_STR)->value()) / 100;
	x_zone->x3 = (float)atoi(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X3_STR)->value()) / 100;
	x_zone->x4 = (float)atoi(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X4_STR)->value()) / 100;

	pos_root_node = initial_pos_root_node->first_node(XML_ELEMENT_POS_Y_ZONE_STR);
	s_trapezoid* y_zone = robotConfig->navParams->initialPosition->yZone;
	y_zone->x1 = (float)atoi(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X1_STR)->value()) / 100;
	y_zone->x2 = (float)atoi(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X2_STR)->value()) / 100;
	y_zone->x3 = (float)atoi(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X3_STR)->value()) / 100;
	y_zone->x4 = (float)atoi(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X4_STR)->value()) / 100;

	pos_root_node = initial_pos_root_node->first_node(XML_ELEMENT_POS_TH_ZONE_STR);
	s_trapezoid* th_zone = robotConfig->navParams->initialPosition->thZone;
	th_zone->x1 = (float)atof(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X1_STR)->value());
	th_zone->x2 = (float)atof(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X2_STR)->value());
	th_zone->x3 = (float)atof(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X3_STR)->value());
	th_zone->x4 = (float)atof(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X4_STR)->value());
	
	xml_node<>* process_noise_root_node = nav_root_node->first_node(XML_ELEMENT_PROCESS_NOISE_STR);
	pos_root_node = process_noise_root_node->first_node(XML_ELEMENT_POS_D_ZONE_STR);
	s_trapezoid* x1_zone =robotConfig->navParams->processNoise->dZone;
	x1_zone->x1 = (float)atof(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X1_STR)->value()) / 100;
	x1_zone->x2 = (float)atof(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X2_STR)->value()) / 100;
	x1_zone->x3 = (float)atof(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X3_STR)->value()) / 100;
	x1_zone->x4 = (float)atof(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X4_STR)->value()) / 100;

	pos_root_node = process_noise_root_node->first_node(XML_ELEMENT_POS_TH_ZONE_STR);
	s_trapezoid* th1_zone = robotConfig->navParams->processNoise->thZone;
	th1_zone->x1 = (float)atof(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X1_STR)->value());
	th1_zone->x2 = (float)atof(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X2_STR)->value());
	th1_zone->x3 = (float)atof(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X3_STR)->value());
	th1_zone->x4 = (float)atof(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X4_STR)->value());

	xml_node<>* sensors_root_node = nav_root_node->first_node(XML_ELEMENT_SENSORS_STR);
	if(sensors_root_node != NULL){
		for(xml_node<>* sensor_node = sensors_root_node->first_node(XML_ELEMENT_SENSOR_STR); sensor_node; sensor_node = sensor_node->next_sibling()){
			s_sensor* sensor = new s_sensor;
			sensor->type = std::string(sensor_node->first_attribute(XML_ATTRIBUTE_TYPE_STR)->value());
			sensor->activated = atoi(sensor_node->first_attribute(XML_ATTRIBUTE_ACTIVATED_STR)->value()) == RN_YES;

			if(sensor->type == XML_SENSOR_TYPE_LASER_STR){
				this->laserSensorActivated = sensor->activated;
			} else if(sensor->type == XML_SENSOR_TYPE_CAMERA_STR){
				this->cameraSensorActivated = sensor->activated;
			} else if(sensor->type == XML_SENSOR_TYPE_RFID_STR){
				this->rfidSensorActivated = sensor->activated;
			}

			xml_node<>* observation_noise_root_node = sensor_node->first_node(XML_ELEMENT_OBSERV_NOISE_STR);
			pos_root_node = observation_noise_root_node->first_node(XML_ELEMENT_POS_D_ZONE_STR);
			s_trapezoid* d_zone = sensor->observationNoise->dZone;
			d_zone->x1 = (float)atoi(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X1_STR)->value()) / 100;
			d_zone->x2 = (float)atoi(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X2_STR)->value()) / 100;
			d_zone->x3 = (float)atoi(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X3_STR)->value()) / 100;
			d_zone->x4 = (float)atoi(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X4_STR)->value()) / 100;

			pos_root_node = observation_noise_root_node->first_node(XML_ELEMENT_POS_TH_ZONE_STR);
			s_trapezoid* th2_zone = sensor->observationNoise->thZone;
			th2_zone->x1 = (float)atof(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X1_STR)->value());
			th2_zone->x2 = (float)atof(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X2_STR)->value());
			th2_zone->x3 = (float)atof(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X3_STR)->value());
			th2_zone->x4 = (float)atof(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X4_STR)->value());

			robotConfig->navParams->sensors->push_back(sensor);
		}
	}

	the_file.close();
}

void GeneralController::getMapFilename(int mapId, std::string& filename){
	xml_document<> doc;
    xml_node<>* root_node;       
	
    std::ifstream the_file(xmlMapsFullPath.c_str());
    std::vector<char> buffer((std::istreambuf_iterator<char>(the_file)), std::istreambuf_iterator<char>());
	buffer.push_back('\0');
	
	doc.parse<0>(&buffer[0]);
	filename = "";
	root_node = doc.first_node(XML_ELEMENT_MAPS_STR);
	if(root_node != NULL){
		bool found = false;
		for(xml_node<>* map_node = root_node->first_node(XML_ELEMENT_MAP_STR); map_node and not found; map_node = map_node->next_sibling()){
			int id = atoi(map_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value());
			if (id == mapId){
				filename = std::string(map_node->first_attribute(XML_ATTRIBUTE_FILENAME_STR)->value());
				found = true;
			}	
		}
    }
    the_file.close();
}


void GeneralController::getMapsAvailable(std::string& mapsAvailable){
	std::ostringstream buffer_str;
	buffer_str.clear();

	xml_document<> doc;
    xml_node<>* root_node;       
	
    std::ifstream the_file(xmlMapsFullPath.c_str());
    std::vector<char> buffer((std::istreambuf_iterator<char>(the_file)), std::istreambuf_iterator<char>());
	buffer.push_back('\0');
	
	doc.parse<0>(&buffer[0]);
	
	root_node = doc.first_node(XML_ELEMENT_MAPS_STR);
	if(root_node != NULL){
		for(xml_node<>* map_node = root_node->first_node(XML_ELEMENT_MAP_STR); map_node; map_node = map_node->next_sibling()){
			buffer_str << map_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value() << ",";
			buffer_str << map_node->first_attribute(XML_ATTRIBUTE_NAME_STR)->value();

			if(map_node->next_sibling() != NULL){
				buffer_str << "|";
			}
			
		}
    }
    the_file.close();

	mapsAvailable = buffer_str.str();
	buffer_str.clear();
}


void GeneralController::getSectorsAvailable(int mapId, std::string& sectorsAvailable){
	std::ostringstream buffer_str;
	std::string filename;
	buffer_str.clear();
	if(mapId > RN_NONE){
		getMapFilename(mapId, filename);

		xml_document<> doc;
	    xml_node<>* root_node;  

	    std::string fullSectorPath = xmlSectorsPath + filename;
	    std::ifstream the_file(fullSectorPath.c_str());
	    std::vector<char> buffer = std::vector<char>((std::istreambuf_iterator<char>(the_file)), std::istreambuf_iterator<char>());
	    buffer.push_back('\0');
	    doc.parse<0>(&buffer[0]);

	    root_node = doc.first_node(XML_ELEMENT_SECTORS_STR);
		if(root_node != NULL){
			for(xml_node<>* sector_node = root_node->first_node(XML_ELEMENT_SECTOR_STR); sector_node; sector_node = sector_node->next_sibling()){
				buffer_str << sector_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value() << ",";
				buffer_str << sector_node->first_attribute(XML_ATTRIBUTE_NAME_STR)->value() << ",";
				buffer_str << (((float)atoi(sector_node->first_attribute(XML_ATTRIBUTE_WIDTH_STR)->value())) / 100) << ",";
				buffer_str << (((float)atoi(sector_node->first_attribute(XML_ATTRIBUTE_HEIGHT_STR)->value())) / 100) << ",";
				buffer_str << sector_node->first_attribute(XML_ATTRIBUTE_REFERENCE_STR)->value();

				if(sector_node->next_sibling() != NULL){
					buffer_str << "|";
				}
				
			}
	    }
	    the_file.close();

		sectorsAvailable = buffer_str.str();
		buffer_str.clear();
	} else {
		RNUtils::printLn("Invalid map id..");
	}
}


void GeneralController::getMapConnection(int mapId, std::string& connections){
	std::ostringstream buffer_str;
	buffer_str.clear();

	std::string filename;
	if(mapId > RN_NONE){
		getMapFilename(mapId, filename);

		std::string fullSectorPath = xmlSectorsPath + filename;
		xml_document<> doc;
	    xml_node<>* root_node;       
		
	    std::ifstream the_file(fullSectorPath.c_str());
	    std::vector<char> buffer((std::istreambuf_iterator<char>(the_file)), std::istreambuf_iterator<char>());
		buffer.push_back('\0');
		
		doc.parse<0>(&buffer[0]);

		root_node = doc.first_node(XML_ELEMENT_SECTORS_STR);
		if(root_node != NULL){
			for(xml_node<>* sector_node = root_node->first_node(XML_ELEMENT_SECTOR_STR); sector_node; sector_node = sector_node->next_sibling()){				
				xml_node<>* features_root_node = sector_node->first_node(XML_ELEMENT_FEATURES_STR);

				if(features_root_node->first_node() !=  NULL){
					for(xml_node<>* feature_node = features_root_node->first_node(XML_ELEMENT_FEATURE_STR); feature_node; feature_node = feature_node->next_sibling()){
						if(strcmp(feature_node->first_attribute(XML_ATTRIBUTE_NAME_STR)->value(), SEMANTIC_FEATURE_DOOR_STR) == 0){
							if(feature_node->first_attribute(XML_ATTRIBUTE_LINKED_SECTOR_ID_STR)){
								buffer_str << sector_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value() << "," << feature_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value();
							
								buffer_str << "," << feature_node->first_attribute(XML_ATTRIBUTE_LINKED_SECTOR_ID_STR)->value();
							
								if(sector_node->next_sibling() != NULL){
									buffer_str << "|";	
								}
							}
						}
					}	
				}
			}
		}
		the_file.close();
		connections = buffer_str.str();
	} else {
		RNUtils::printLn("getMapConnection(). Invalid map id..");
	}

}

void GeneralController::getSectorInformationLandmarks(int mapId, int sectorId, std::string& sectorInformation){

	std::ostringstream buffer_str;
	buffer_str.clear();
	std::string filename;
	if(mapId > RN_NONE){
		getMapFilename(mapId, filename);
		std::string fullSectorPath = xmlSectorsPath + filename;
		xml_document<> doc;
	    xml_node<>* root_node;       
		
	    std::ifstream the_file(fullSectorPath.c_str());
	    std::vector<char> buffer((std::istreambuf_iterator<char>(the_file)), std::istreambuf_iterator<char>());
		buffer.push_back('\0');
		
		doc.parse<0>(&buffer[0]);
		
		root_node = doc.first_node(XML_ELEMENT_SECTORS_STR);
		if(root_node != NULL){
			bool found = false;
			for(xml_node<>* sector_node = root_node->first_node(XML_ELEMENT_SECTOR_STR); sector_node and not found; sector_node = sector_node->next_sibling()){
				int xmlSectorId = atoi(sector_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value());
				if(sectorId == xmlSectorId){
					found = true;
					xml_node<>* landmarks_root_node = sector_node->first_node(XML_ELEMENT_LANDMARKS_STR);
					if(landmarks_root_node->first_node() !=  NULL){
						for(xml_node<>* landmark_node = landmarks_root_node->first_node(XML_ELEMENT_LANDMARK_STR); landmark_node; landmark_node = landmark_node->next_sibling()){
							buffer_str << landmark_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value() << ",";
							buffer_str << (((float)atoi(landmark_node->first_attribute(XML_ATTRIBUTE_X_POSITION_STR)->value())) / 100) << ",";
							buffer_str << (((float)atoi(landmark_node->first_attribute(XML_ATTRIBUTE_Y_POSITION_STR)->value())) / 100) << ",";
							buffer_str << (((float)atoi(landmark_node->first_attribute(XML_ATTRIBUTE_Z_POSITION_STR)->value())) / 100);
							if(landmark_node->first_attribute(XML_ATTRIBUTE_VARIANCE_MIN_X_STR)){
								buffer_str << "," << (((float)atoi(landmark_node->first_attribute(XML_ATTRIBUTE_VARIANCE_MIN_X_STR)->value())) / 100) << ",";
							}
							if(landmark_node->first_attribute(XML_ATTRIBUTE_VARIANCE_MIN_X_STR)){
								buffer_str << (((float)atoi(landmark_node->first_attribute(XML_ATTRIBUTE_VARIANCE_MAX_X_STR)->value())) / 100) << ",";
							}
							if(landmark_node->first_attribute(XML_ATTRIBUTE_VARIANCE_MIN_X_STR)){
								buffer_str << (((float)atoi(landmark_node->first_attribute(XML_ATTRIBUTE_VARIANCE_MIN_Y_STR)->value())) / 100) << ",";
							}
							if(landmark_node->first_attribute(XML_ATTRIBUTE_VARIANCE_MIN_X_STR)){
								buffer_str << (((float)atoi(landmark_node->first_attribute(XML_ATTRIBUTE_VARIANCE_MAX_Y_STR)->value())) / 100);
							}

							if(landmark_node->next_sibling() != NULL){
								buffer_str << "|";
							}
						}
					}
				}
			}
	    }
	    the_file.close();
		
		sectorInformation = buffer_str.str();
		buffer_str.clear();
	} else {
		RNUtils::printLn("Invalid map id..");
	}
}


void GeneralController::getSectorInformationFeatures(int mapId, int sectorId, std::string& sectorInformation){

	std::ostringstream buffer_str;
	buffer_str.clear();

	std::string filename;
	if(mapId > RN_NONE){
		getMapFilename(mapId, filename);
		std::string fullSectorPath = xmlSectorsPath + filename;
		xml_document<> doc;
	    xml_node<>* root_node;       
		
	    std::ifstream the_file(fullSectorPath.c_str());
	    std::vector<char> buffer((std::istreambuf_iterator<char>(the_file)), std::istreambuf_iterator<char>());
		buffer.push_back('\0');
		
		doc.parse<0>(&buffer[0]);
		
		root_node = doc.first_node(XML_ELEMENT_SECTORS_STR);
		if(root_node != NULL){
			bool found = false;
			for(xml_node<>* sector_node = root_node->first_node(XML_ELEMENT_SECTOR_STR); sector_node and not found; sector_node = sector_node->next_sibling()){
				int xmlSectorId = atoi(sector_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value());
				if(sectorId == xmlSectorId){
					found = true;
					//buffer_str << sectorId << "|"; agregado para lluis, pero luego lo quito porque no le encuentro sentido
					xml_node<>* features_root_node = sector_node->first_node(XML_ELEMENT_FEATURES_STR);
					if(features_root_node->first_node() !=  NULL){
						for(xml_node<>* feature_node = features_root_node->first_node(XML_ELEMENT_FEATURE_STR); feature_node; feature_node = feature_node->next_sibling()){
							buffer_str << feature_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value() << ",";
							buffer_str << feature_node->first_attribute(XML_ATTRIBUTE_NAME_STR)->value() << ",";
							buffer_str << (((float)atoi(feature_node->first_attribute(XML_ATTRIBUTE_X_POSITION_STR)->value())) / 100) << ",";
							buffer_str << (((float)atoi(feature_node->first_attribute(XML_ATTRIBUTE_Y_POSITION_STR)->value())) / 100) << ",";
							buffer_str << (((float)atoi(feature_node->first_attribute(XML_ATTRIBUTE_WIDTH_STR)->value())) / 100) << ",";
							buffer_str << (((float)atoi(feature_node->first_attribute(XML_ATTRIBUTE_HEIGHT_STR)->value())) / 100);

							if(feature_node->first_attribute(XML_ATTRIBUTE_LINKED_SECTOR_ID_STR)){
								buffer_str << "," << feature_node->first_attribute(XML_ATTRIBUTE_LINKED_SECTOR_ID_STR)->value();
							} else {
								buffer_str << "," << RN_NONE;
							}


							if(feature_node->next_sibling() != NULL){
								buffer_str << "|";
							}
						}
					}
				}
			}
	    }
	    the_file.close();

		sectorInformation = buffer_str.str();
		buffer_str.clear();
	} else {
		RNUtils::printLn("Invalid map id..");
	}
}

void GeneralController::getSectorInformationSites(int mapId, int sectorId, std::string& sectorInformation){

	std::ostringstream buffer_str;
	buffer_str.clear();

	std::string filename;
	if(mapId > RN_NONE){
		getMapFilename(mapId, filename);
		std::string fullSectorPath = xmlSectorsPath + filename;
		xml_document<> doc;
	    xml_node<>* root_node;       
		
	    std::ifstream the_file(fullSectorPath.c_str());
	    std::vector<char> buffer((std::istreambuf_iterator<char>(the_file)), std::istreambuf_iterator<char>());
		buffer.push_back('\0');
		
		doc.parse<0>(&buffer[0]);
		
		root_node = doc.first_node(XML_ELEMENT_SECTORS_STR);
		if(root_node != NULL){
			bool found = false;
			for(xml_node<>* sector_node = root_node->first_node(XML_ELEMENT_SECTOR_STR); sector_node and not found; sector_node = sector_node->next_sibling()){
				int xmlSectorId = atoi(sector_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value());
				if(sectorId == xmlSectorId){
					found = true;
					//buffer_str << sectorId << "|"; idem
					xml_node<>* sites_root_node = sector_node->first_node(XML_ELEMENT_SITES_STR);
					if(sites_root_node->first_node() !=  NULL){
						for(xml_node<>* site_node = sites_root_node->first_node(XML_ELEMENT_SITE_STR); site_node; site_node = site_node->next_sibling()){
							buffer_str << site_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value() << ",";
							buffer_str << site_node->first_attribute(XML_ATTRIBUTE_NAME_STR)->value() << ",";
							buffer_str << (((float)atoi(site_node->first_attribute(XML_ATTRIBUTE_X_POSITION_STR)->value())) / 100) << ",";
							buffer_str << (((float)atoi(site_node->first_attribute(XML_ATTRIBUTE_Y_POSITION_STR)->value())) / 100) << ",";
							buffer_str << (((float)atoi(site_node->first_attribute(XML_ATTRIBUTE_RADIUS_STR)->value())) / 100);

							if(site_node->first_attribute(XML_ATTRIBUTE_LINKED_FEATURE_ID_STR)){
								buffer_str << "," << site_node->first_attribute(XML_ATTRIBUTE_LINKED_FEATURE_ID_STR)->value();
							} else {
								buffer_str << "," << RN_NONE;
							}

							
							if(site_node->next_sibling() != NULL){
								buffer_str << "|";
							}
						}
					}
				}
			}
	    }
	    the_file.close();

		sectorInformation = buffer_str.str();
		buffer_str.clear();
	} else {
		RNUtils::printLn("Invalid map id..");
	}
}

void GeneralController::getSectorInformationSitesSequence(int mapId, int sectorId, std::string& sectorInformation){
	sectorInformation = "";

	std::string filename;
	if(mapId > RN_NONE){
		getMapFilename(mapId, filename);
		std::string fullSectorPath = xmlSectorsPath + filename;
		xml_document<> doc;
	    xml_node<>* root_node;       
		
	    std::ifstream the_file(fullSectorPath.c_str());
	    std::vector<char> buffer((std::istreambuf_iterator<char>(the_file)), std::istreambuf_iterator<char>());
		buffer.push_back('\0');
		
		doc.parse<0>(&buffer[0]);
		
		root_node = doc.first_node(XML_ELEMENT_SECTORS_STR);
		if(root_node != NULL){
			bool found = false;
			for(xml_node<>* sector_node = root_node->first_node(XML_ELEMENT_SECTOR_STR); sector_node and not found; sector_node = sector_node->next_sibling()){
				int xmlSectorId = atoi(sector_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value());
				if(sectorId == xmlSectorId){
					found = true;
					xml_node<>* sites_root_node = sector_node->first_node(XML_ELEMENT_SITES_STR);

					if(sites_root_node->first_attribute()){
						
						if(sites_root_node->first_attribute(XML_ATTRIBUTE_SEQUENCE_STR)){
							std::string sequence(sites_root_node->first_attribute(XML_ATTRIBUTE_SEQUENCE_STR)->value());
							sectorInformation = sequence;
						}
					}
				}
			}
	    }
	    the_file.close();
	} else {
		RNUtils::printLn("Invalid map id..");
	}
}

void GeneralController::addSectorInformationSite(char* cad, int& indexAssigned){
	std::vector<std::string> data = RNUtils::split(cad, ",");
	if(data.size() >= ADD_SITE_VARIABLE_LENGTH){
		bool found = false;
		int mapId = atoi(data.at(0).c_str());
		int sectorId = atoi(data.at(1).c_str());
		indexAssigned = 0;
		std::string filename;
		getMapFilename(mapId, filename);
		std::string fullSectorPath = xmlSectorsPath + filename;
		xml_document<> doc;
	    xml_node<>* root_node;       
		
	    std::ifstream the_file(fullSectorPath.c_str());
	    std::vector<char> buffer((std::istreambuf_iterator<char>(the_file)), std::istreambuf_iterator<char>());
		buffer.push_back('\0');
		
		doc.parse<0>(&buffer[0]);
		
		root_node = doc.first_node(XML_ELEMENT_SECTORS_STR);
		if(root_node != NULL){
			found = false;
			for(xml_node<>* sector_node = root_node->first_node(XML_ELEMENT_SECTOR_STR); sector_node and not found; sector_node = sector_node->next_sibling()){
				int xmlSectorId = atoi(sector_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value());
				if(sectorId == xmlSectorId){
					found = true;
					xml_node<>* sites_root_node = sector_node->first_node(XML_ELEMENT_SITES_STR);
					
					if(sites_root_node != NULL){
						bool indexAssignedFound = false;
						
						while(!indexAssignedFound){
							bool indexFounded = false;
							for(xml_node<>* site_node = sites_root_node->first_node(XML_ELEMENT_SITE_STR); site_node and not indexFounded; site_node = site_node->next_sibling()){

								if(atoi(site_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value()) == indexAssigned){
									indexFounded = true;
								}
							}
							if(indexFounded){
								indexAssigned++;
							} else {
								indexAssignedFound = true;
							}

						}

						xml_node<>* new_sites_node = doc.allocate_node(node_element, XML_ELEMENT_SITE_STR);

						std::ostringstream convert;
						convert.str("");
						convert << indexAssigned;
						RNUtils::printLn("New Site Index Assigned: %s", convert.str().c_str());
				        new_sites_node->append_attribute(doc.allocate_attribute(XML_ATTRIBUTE_ID_STR, convert.str().c_str()));
				        new_sites_node->append_attribute(doc.allocate_attribute(XML_ATTRIBUTE_NAME_STR, data.at(2).c_str()));
				        new_sites_node->append_attribute(doc.allocate_attribute(XML_ATTRIBUTE_RADIUS_STR, data.at(3).c_str()));
				        new_sites_node->append_attribute(doc.allocate_attribute(XML_ATTRIBUTE_TIME_STR, data.at(4).c_str()));
				        new_sites_node->append_attribute(doc.allocate_attribute(XML_ATTRIBUTE_X_POSITION_STR, data.at(5).c_str()));
				        new_sites_node->append_attribute(doc.allocate_attribute(XML_ATTRIBUTE_Y_POSITION_STR, data.at(6).c_str()));
				        if(data.size() >= ADD_SITE_VARIABLE_LENGTH + 1){
				        	new_sites_node->append_attribute(doc.allocate_attribute(XML_ATTRIBUTE_LINKED_FEATURE_ID_STR, data.at(7).c_str()));
				        }

				        sites_root_node->append_node(new_sites_node);

			        	if(mapId == currentMapId and sectorId == currentSector->getId()){
				        	s_site* tempSite = new s_site;

				        	tempSite->id = indexAssigned;
							tempSite->name = data.at(2);
							tempSite->radius = ((float)atoi(data.at(3).c_str())) / 100;
							tempSite->tsec = ((float)atoi(data.at(4).c_str()));
							tempSite->xpos = ((float)atoi(data.at(5).c_str())) / 100;
							tempSite->ypos = ((float)atoi(data.at(6).c_str())) / 100;
							if(data.size() >= ADD_SITE_VARIABLE_LENGTH + 1){
								tempSite->linkedFeatureId = atoi(data.at(7).c_str());
							} else {
								tempSite->linkedFeatureId = RN_NONE;
							}
				        	currentSector->addSite(tempSite);
				        }

				        RNUtils::getTimestamp(mappingSitesTimestamp);
					}
				}
			}
	    }
	    the_file.close();
	    std::ofstream the_new_file(fullSectorPath.c_str());
	    the_new_file << doc;
	    the_new_file.close();
		    
	    
	} else {
		RNUtils::printLn("Unable to add new site. Invalid number of arguments...");
	}
}

void GeneralController::modifySectorInformationSite(char* cad){
	std::vector<std::string> data = RNUtils::split(cad, ",");
	if(data.size() >= MODIFY_SITE_VARIABLE_LENGTH){
		bool found = false;
		int mapId = atoi(data.at(0).c_str());
		int sectorId = atoi(data.at(1).c_str());
		int siteId = atoi(data.at(2).c_str());

		std::string filename;
		getMapFilename(mapId, filename);
		std::string fullSectorPath = xmlSectorsPath + filename;

		xml_document<> doc;
	    xml_node<>* root_node;       
		
	    std::ifstream the_file(fullSectorPath.c_str());
	    std::vector<char> buffer((std::istreambuf_iterator<char>(the_file)), std::istreambuf_iterator<char>());
		buffer.push_back('\0');
		
		doc.parse<0>(&buffer[0]);
		
		root_node = doc.first_node(XML_ELEMENT_SECTORS_STR);
		if(root_node != NULL){
			found = false;
			for(xml_node<>* sector_node = root_node->first_node(XML_ELEMENT_SECTOR_STR); sector_node and not found; sector_node = sector_node->next_sibling()){
				int xmlSectorId = atoi(sector_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value());
				if(xmlSectorId == sectorId){
					found = true;
					xml_node<>* sites_root_node = sector_node->first_node(XML_ELEMENT_SITES_STR);
					if(sites_root_node != NULL){
						bool siteFound = false;
						for(xml_node<> *where = sites_root_node->first_node(XML_ELEMENT_SITE_STR); where and not siteFound; where = where->next_sibling()){
							int indexToDelete = atoi(where->first_attribute(XML_ATTRIBUTE_ID_STR)->value());
							if(indexToDelete == siteId){
								siteFound = true;
								where->remove_all_attributes();
								std::ostringstream convert;
								convert << siteId;

				        		where->append_attribute(doc.allocate_attribute(XML_ATTRIBUTE_ID_STR, convert.str().c_str()));
								where->append_attribute(doc.allocate_attribute(XML_ATTRIBUTE_NAME_STR, data.at(3).c_str()));
						        where->append_attribute(doc.allocate_attribute(XML_ATTRIBUTE_RADIUS_STR, data.at(4).c_str()));
						        where->append_attribute(doc.allocate_attribute(XML_ATTRIBUTE_TIME_STR, data.at(5).c_str()));
						        where->append_attribute(doc.allocate_attribute(XML_ATTRIBUTE_X_POSITION_STR, data.at(6).c_str()));
						        where->append_attribute(doc.allocate_attribute(XML_ATTRIBUTE_Y_POSITION_STR, data.at(7).c_str()));

						        if(data.size() >= MODIFY_SITE_VARIABLE_LENGTH + 1){
						        	where->append_attribute(doc.allocate_attribute(XML_ATTRIBUTE_LINKED_FEATURE_ID_STR, data.at(8).c_str()));
						        }

						        
								RNUtils::getTimestamp(mappingSitesTimestamp);
							}
						}
					}
				}
			}
	    }
	    the_file.close();

	    std::ofstream the_new_file(fullSectorPath.c_str());
	    the_new_file << doc;
	    the_new_file.close();

		if(mapId == currentMapId and sectorId == currentSector->getId()){
        	s_site* tempSite = currentSector->findSiteById(siteId);

        	tempSite->name = data.at(3);
			tempSite->radius = ((float)atoi(data.at(4).c_str())) / 100;
			tempSite->tsec = ((float)atoi(data.at(5).c_str()));
			tempSite->xpos = ((float)atoi(data.at(6).c_str())) / 100;
			tempSite->ypos = ((float)atoi(data.at(7).c_str())) / 100;
			if(data.size() >= MODIFY_SITE_VARIABLE_LENGTH + 1){
				tempSite->linkedFeatureId = atoi(data.at(8).c_str());
			} else {
				tempSite->linkedFeatureId = RN_NONE;
			}			
		}
	} else {
		RNUtils::printLn("Unable to edit site. Invalid number of parameters..");
	}
}

void GeneralController::deleteSectorInformationSite(char* cad){
	std::vector<std::string> data = RNUtils::split(cad, ",");
	if(data.size() == DELETE_SITE_VARIABLE_LENGTH){
		bool found = false;
		int mapId = atoi(data.at(0).c_str());
		int sectorId = atoi(data.at(1).c_str());
		int siteId = atoi(data.at(2).c_str());

		std::string filename;
		getMapFilename(mapId, filename);
		std::string fullSectorPath = xmlSectorsPath + filename;

		xml_document<> doc;
	    xml_node<>* root_node;       
		
	    std::ifstream the_file(fullSectorPath.c_str());
	    std::vector<char> buffer((std::istreambuf_iterator<char>(the_file)), std::istreambuf_iterator<char>());
		buffer.push_back('\0');
		
		doc.parse<0>(&buffer[0]);
		
		root_node = doc.first_node(XML_ELEMENT_SECTORS_STR);
		if(root_node != NULL){

			found = false;
			for(xml_node<>* sector_node = root_node->first_node(XML_ELEMENT_SECTOR_STR); sector_node and not found; sector_node = sector_node->next_sibling()){
				int xmlSectorId = atoi(sector_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value());
				if(xmlSectorId == sectorId){
					found = true;
					xml_node<>* sites_root_node = sector_node->first_node(XML_ELEMENT_SITES_STR);
					if(sites_root_node != NULL){
						bool siteFound = false;
						xml_node<> *toDelete = NULL;
						for(xml_node<> *where = sites_root_node->first_node(XML_ELEMENT_SITE_STR); where and not siteFound; where = where->next_sibling()){
							int indexToDelete = atoi(where->first_attribute(XML_ATTRIBUTE_ID_STR)->value());
							if(indexToDelete == siteId){
								siteFound = true;
								toDelete = where;
							}
						}
						if(toDelete != NULL){
							sites_root_node->remove_node(toDelete);
							RNUtils::getTimestamp(mappingSitesTimestamp);
						}

					}
				}
			}
	    }

	    the_file.close();
	    std::ofstream the_new_file(fullSectorPath.c_str());
	    the_new_file << doc;
	    the_new_file.close();
		   
		if(mapId == currentMapId and sectorId == currentSector->getId()){
        	s_site* tempSite = currentSector->findSiteById(siteId);
        	currentSector->deleteSite(tempSite);
        }
		
	} else {
		RNUtils::printLn("Unable to delete site. Invalid number of parameters..");
	}    
}

void GeneralController::setSitesExecutionSequence(char* cad){
	std::vector<std::string> data = RNUtils::split(cad, "|");
	if(data.size() == SITE_SEQUENCE_VARIABLE_LENGTH){
		bool found = false;
		std::vector<std::string> dataInfoMap = RNUtils::split((char*)data.at(0).c_str(), ",");
		int mapId = atoi(dataInfoMap.at(0).c_str());
		int sectorId = atoi(dataInfoMap.at(1).c_str());

		std::string filename;
		getMapFilename(mapId, filename);
		std::string fullSectorPath = xmlSectorsPath + filename;

		xml_document<> doc;
	    xml_node<>* root_node;       
		
	    std::ifstream the_file(fullSectorPath.c_str());
	    std::vector<char> buffer((std::istreambuf_iterator<char>(the_file)), std::istreambuf_iterator<char>());
		buffer.push_back('\0');
		
		doc.parse<0>(&buffer[0]);
		
		root_node = doc.first_node(XML_ELEMENT_SECTORS_STR);
		if(root_node != NULL){
			found = false;
			for(xml_node<>* sector_node = root_node->first_node(XML_ELEMENT_SECTOR_STR); sector_node and not found; sector_node = sector_node->next_sibling()){
				int xmlSectorId = atoi(sector_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value());
				if(xmlSectorId == sectorId){
					found = true;
					xml_node<>* sites_root_node = sector_node->first_node(XML_ELEMENT_SITES_STR);
					if(sites_root_node != NULL){
						xml_attribute<> *where = sites_root_node->first_attribute(XML_ATTRIBUTE_SEQUENCE_STR);
						if(where != NULL){
							sites_root_node->remove_attribute(where);
						}
						sites_root_node->append_attribute(doc.allocate_attribute(XML_ATTRIBUTE_SEQUENCE_STR, data.at(1).c_str()));
						RNUtils::getTimestamp(mappingSitesTimestamp);
					}
				}
			}
	    }

	    the_file.close();

	    std::ofstream the_new_file(fullSectorPath.c_str());
	    the_new_file << doc;
	    the_new_file.close();

	    if(mapId == currentMapId and sectorId == currentSector->getId()){

        	currentSector->setSequence(data.at(1));
        }
		    
		
	} else {
		RNUtils::printLn("Unable to set site sequence. Invalid number or parameters");
	}
}

void GeneralController::addSectorInformationFeatures(char* cad, int& indexAssigned){
	std::vector<std::string> data = RNUtils::split(cad, ",");
	if(data.size() >= ADD_FEATURE_VARIABLE_LENGTH){
		bool found = false;
		int mapId = atoi(data.at(0).c_str());
		int sectorId = atoi(data.at(1).c_str());
		indexAssigned = 0;
		std::string filename;
		getMapFilename(mapId, filename);
		std::string fullSectorPath = xmlSectorsPath + filename;

		xml_document<> doc;
	    xml_node<>* root_node;       
		
	    std::ifstream the_file(fullSectorPath.c_str());
	    std::vector<char> buffer((std::istreambuf_iterator<char>(the_file)), std::istreambuf_iterator<char>());
		buffer.push_back('\0');
		
		doc.parse<0>(&buffer[0]);
		
		root_node = doc.first_node(XML_ELEMENT_SECTORS_STR);
		if(root_node != NULL){
			found = false;
			for(xml_node<>* sector_node = root_node->first_node(XML_ELEMENT_SECTOR_STR); sector_node and not found; sector_node = sector_node->next_sibling()){
				int xmlSectorId = atoi(sector_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value());
				if(xmlSectorId == sectorId){
					found = true;
					xml_node<>* features_root_node = sector_node->first_node(XML_ELEMENT_FEATURES_STR);
					if(features_root_node != NULL){
						bool indexAssignedFound = false;
						
						while(!indexAssignedFound){
							bool indexFounded = false;
							for(xml_node<>* feature_node = features_root_node->first_node(XML_ELEMENT_FEATURE_STR); feature_node and not indexFounded; feature_node = feature_node->next_sibling()){

								if(atoi(feature_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value()) == indexAssigned){
									indexFounded = true;
								}
							}
							if(indexFounded){
								indexAssigned++;
							} else {
								indexAssignedFound = true;
							}

						}
						xml_node<>* new_feature_node = doc.allocate_node(node_element, XML_ELEMENT_FEATURE_STR);

						std::ostringstream convert;
						convert << indexAssigned;

				        new_feature_node->append_attribute(doc.allocate_attribute(XML_ATTRIBUTE_ID_STR, convert.str().c_str()));
				        new_feature_node->append_attribute(doc.allocate_attribute(XML_ATTRIBUTE_NAME_STR, data.at(2).c_str()));
				        new_feature_node->append_attribute(doc.allocate_attribute(XML_ATTRIBUTE_WIDTH_STR, data.at(3).c_str()));
				        new_feature_node->append_attribute(doc.allocate_attribute(XML_ATTRIBUTE_HEIGHT_STR, data.at(4).c_str()));
				        new_feature_node->append_attribute(doc.allocate_attribute(XML_ATTRIBUTE_X_POSITION_STR, data.at(5).c_str()));
				        new_feature_node->append_attribute(doc.allocate_attribute(XML_ATTRIBUTE_Y_POSITION_STR, data.at(6).c_str()));

				        if(data.size() >= ADD_FEATURE_VARIABLE_LENGTH + 1){
				        	new_feature_node->append_attribute(doc.allocate_attribute(XML_ATTRIBUTE_LINKED_SECTOR_ID_STR, data.at(7).c_str()));
				        }

				        features_root_node->append_node(new_feature_node);

				        if(mapId == currentMapId and sectorId == currentSector->getId()){
				        	s_feature* tempFeature = new s_feature;

							tempFeature->id = indexAssigned;
							tempFeature->name = data.at(2);
							tempFeature->width = ((float)atoi(data.at(3).c_str())) / 100;
							tempFeature->height = ((float)atoi(data.at(4).c_str())) / 100;
							tempFeature->xpos = ((float)atoi(data.at(5).c_str())) / 100;
							tempFeature->ypos = ((float)atoi(data.at(6).c_str())) / 100;

							if(data.size() >= ADD_FEATURE_VARIABLE_LENGTH + 1){
								tempFeature->linkedSectorId = atoi(data.at(7).c_str());
							} else {
								tempFeature->linkedSectorId = RN_NONE;
							}

							currentSector->addFeature(tempFeature);
				        }
				        RNUtils::getTimestamp(mappingFeaturesTimestamp);
					}
				}
			}
	    }
	    the_file.close();
	    std::ofstream the_new_file(fullSectorPath.c_str());
	    the_new_file << doc;
	    the_new_file.close();	    
	} else {
		RNUtils::printLn("Unable to add new feature. Invalid number of arguments...");
	}
}

void GeneralController::modifySectorInformationFeatures(char* cad){
	std::vector<std::string> data = RNUtils::split(cad, ",");
	if(data.size() >= MODIFY_FEATURE_VARIABLE_LENGTH){
		bool found = false;
		int mapId = atoi(data.at(0).c_str());
		int sectorId = atoi(data.at(1).c_str());
		int featureId = atoi(data.at(2).c_str());

		std::string filename;
		getMapFilename(mapId, filename);
		std::string fullSectorPath = xmlSectorsPath + filename;

		xml_document<> doc;
	    xml_node<>* root_node;       
		
	    std::ifstream the_file(fullSectorPath.c_str());
	    std::vector<char> buffer((std::istreambuf_iterator<char>(the_file)), std::istreambuf_iterator<char>());
		buffer.push_back('\0');
		
		doc.parse<0>(&buffer[0]);
		
		root_node = doc.first_node(XML_ELEMENT_SECTORS_STR);
		if(root_node != NULL){
			found = false;
			for(xml_node<>* sector_node = root_node->first_node(XML_ELEMENT_SECTOR_STR); sector_node and not found; sector_node = sector_node->next_sibling()){
				int xmlSectorId = atoi(sector_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value());
				if(xmlSectorId == sectorId){
					found = true;
					xml_node<>* features_root_node = sector_node->first_node(XML_ELEMENT_FEATURES_STR);
					if(features_root_node != NULL){
						bool featureFound = false;
						for(xml_node<> *where = features_root_node->first_node(XML_ELEMENT_FEATURE_STR); where and not featureFound; where = where->next_sibling()){
							int indexToDelete = atoi(where->first_attribute(XML_ATTRIBUTE_ID_STR)->value());
							if(indexToDelete == featureId){
								featureFound = true;
								where->remove_all_attributes();
								std::ostringstream convert;
								convert << featureId;

				        		where->append_attribute(doc.allocate_attribute(XML_ATTRIBUTE_ID_STR, convert.str().c_str()));
								where->append_attribute(doc.allocate_attribute(XML_ATTRIBUTE_NAME_STR, data.at(3).c_str()));
						        where->append_attribute(doc.allocate_attribute(XML_ATTRIBUTE_WIDTH_STR, data.at(4).c_str()));
						        where->append_attribute(doc.allocate_attribute(XML_ATTRIBUTE_HEIGHT_STR, data.at(5).c_str()));
						        where->append_attribute(doc.allocate_attribute(XML_ATTRIBUTE_X_POSITION_STR, data.at(6).c_str()));
						        where->append_attribute(doc.allocate_attribute(XML_ATTRIBUTE_Y_POSITION_STR, data.at(7).c_str()));

						        if(data.size() >= MODIFY_FEATURE_VARIABLE_LENGTH + 1){
						        	where->append_attribute(doc.allocate_attribute(XML_ATTRIBUTE_LINKED_SECTOR_ID_STR, data.at(8).c_str()));
						        }

								RNUtils::getTimestamp(mappingFeaturesTimestamp);
							}
						}
					}
				}
			}
	    }
	    the_file.close();

	    std::ofstream the_new_file(fullSectorPath.c_str());
	    the_new_file << doc;
	    the_new_file.close();

	    if(mapId == currentMapId and sectorId == currentSector->getId()){
        	s_feature* tempFeature = currentSector->findFeatureById(featureId);

			tempFeature->name = data.at(3);
			tempFeature->width = ((float)atoi(data.at(4).c_str())) / 100;
			tempFeature->height = ((float)atoi(data.at(5).c_str())) / 100;
			tempFeature->xpos = ((float)atoi(data.at(6).c_str())) / 100;
			tempFeature->ypos = ((float)atoi(data.at(7).c_str())) / 100;
			
			if(data.size() >= MODIFY_FEATURE_VARIABLE_LENGTH + 1){
				tempFeature->linkedSectorId = atoi(data.at(8).c_str());
			} else {
				tempFeature->linkedSectorId = RN_NONE;
			}
        }
	} else {
		RNUtils::printLn("Unable to edit feature. Invalid number of parameters..");
	}
}

void GeneralController::deleteSectorInformationFeatures(char* cad){
	std::vector<std::string> data = RNUtils::split(cad, ",");
	if(data.size() == DELETE_FEATURE_VARIABLE_LENGTH){
		bool found = false;
		int mapId = atoi(data.at(0).c_str());
		int sectorId = atoi(data.at(1).c_str());
		int featureId = atoi(data.at(2).c_str());

		std::string filename;
		getMapFilename(mapId, filename);
		std::string fullSectorPath = xmlSectorsPath + filename;

		xml_document<> doc;
	    xml_node<>* root_node;       
		
	    std::ifstream the_file(fullSectorPath.c_str());
	    std::vector<char> buffer((std::istreambuf_iterator<char>(the_file)), std::istreambuf_iterator<char>());
		buffer.push_back('\0');
		
		doc.parse<0>(&buffer[0]);
		
		root_node = doc.first_node(XML_ELEMENT_SECTORS_STR);
		if(root_node != NULL){

			found = false;
			for(xml_node<>* sector_node = root_node->first_node(XML_ELEMENT_SECTOR_STR); sector_node and not found; sector_node = sector_node->next_sibling()){
				int xmlSectorId = atoi(sector_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value());
				if(xmlSectorId == sectorId){
					found = true;
					xml_node<>* features_root_node = sector_node->first_node(XML_ELEMENT_FEATURES_STR);
					if(features_root_node != NULL){
						bool featureFound = false;
						xml_node<> *toDelete = NULL;
						for(xml_node<> *where = features_root_node->first_node(XML_ELEMENT_FEATURE_STR); where and not featureFound; where = where->next_sibling()){
							int indexToDelete = atoi(where->first_attribute(XML_ATTRIBUTE_ID_STR)->value());
							if(indexToDelete == featureId){
								featureFound = true;
								toDelete = where;
							}
						}
						if(toDelete != NULL){
							features_root_node->remove_node(toDelete);
							RNUtils::getTimestamp(mappingFeaturesTimestamp);
						}

					}
				}
			}
	    }

	    the_file.close();
	    std::ofstream the_new_file(fullSectorPath.c_str());
	    the_new_file << doc;
	    the_new_file.close();

	    if(mapId == currentMapId and sectorId == currentSector->getId()){
        	s_feature* tempFeature = currentSector->findFeatureById(featureId);
        	currentSector->deleteFeature(tempFeature);
        }

	} else {
		RNUtils::printLn("Unable to delete feature. Invalid number of parameters..");
	}    
}

void GeneralController::moveRobot(float lin_vel, float angular_vel){

	bool bumpersOk = false;
	if(frontBumpersOk && rearBumpersOk){
		bumpersOk = true;
	} else if(!frontBumpersOk && lin_vel < 0){
		bumpersOk = true;
	} else if(!rearBumpersOk && lin_vel > 0){
		bumpersOk = true;
	} else {
		bumpersOk = false;
	}
	if (bumpersOk){
		this->moveAtSpeed(lin_vel, angular_vel);
	} else {
		this->stopRobot();
	}
}

void GeneralController::setRobotPosition(float x, float y, float theta){
	RNUtils::printLn("new position is: {x: %f, y: %f, theta: %f}", x, y, theta);
	this->setPosition(x, y, theta);	
}

void GeneralController::setRobotPosition(Matrix Xk){
	this->setRobotPosition(Xk(0, 0), Xk(1, 0), Xk(2, 0));
}

void GeneralController::moveRobotToPosition(float x, float y, float theta){
	this->gotoPosition(x, y, theta);
}

void GeneralController::onBumpersUpdate(std::vector<bool> front, std::vector<bool> rear){
	char* bump = new char[256];

	frontBumpersOk = true;
	rearBumpersOk = true;

	for (int i = 0; i < front.size(); i++){
		if (front[i] && frontBumpersOk){
			frontBumpersOk = false;
		}
	}

	for (int i = 0; i < rear.size(); i++){
		if (rear[i] && rearBumpersOk){
			rearBumpersOk = false;
		}
	}

	sprintf(bump, "$BUMPERS|%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", (int)front[0], (int)front[1], (int)front[2], (int)front[3], (int)front[4], (int)front[5],
			(int)rear[0], (int)rear[1], (int)rear[2], (int)rear[3], (int)rear[4], (int)rear[5]);
	int dataLen = strlen(bump);
	if(spdUDPClient != NULL){
		spdUDPClient->sendData((unsigned char*)bump, dataLen);
	}
	for(int i = 0; i < MAX_CLIENTS; i++){
		if(isConnected(i) && isWebSocket(i)){
			spdWSServer->sendMsg(i, 0x00, bump, dataLen);
		}
	}
	delete[] bump;
}

void GeneralController::onPositionUpdate(double x, double y, double theta, double transSpeed, double rotSpeed){

	robotEncoderPosition(0, 0) = x;
	robotEncoderPosition(1, 0) = y;
	robotEncoderPosition(2, 0) = theta;

	robotVelocity(0, 0) = transSpeed;
	robotVelocity(1, 0) = rotSpeed;
	if(currentSector != NULL){
		std::vector<s_feature*> doors = currentSector->findFeaturesByName(std::string(SEMANTIC_FEATURE_DOOR_STR));
		if(doors.size() > 0){
			float distance = std::numeric_limits<float>::infinity();
			int index = RN_NONE;
	        float nXCoord = 0;
	        float nYCoord = 0;
			for(int i = 0; i < doors.size(); i++){
				float fromHereXY = std::sqrt(std::pow((robotEncoderPosition(0, 0) - doors.at(i)->xpos), 2) + std::pow((robotEncoderPosition(1, 0) - doors.at(i)->ypos), 2));
				if(distance > fromHereXY){
					distance = fromHereXY;
					index = doors.at(i)->linkedSectorId;
	                nXCoord = doors.at(i)->xcoord;
	                nYCoord = doors.at(i)->ycoord;
				}
			}
			this->nextSectorId = index;
			nextSectorCoord = PointXY(nXCoord, nYCoord);
	    }
	    if(RNUtils::toLowercase(currentSector->getName()).find(SEMANTIC_HALLWAY_STR) < std::string(SEMANTIC_HALLWAY_STR).length()){
	    	this->hallwayDetected = true;
	    } else {
	    	this->hallwayDetected = false;
	    }
	}

	
	char* bump = new char[256];
	sprintf(bump, "$POSE_VEL|%.4f,%.4f,%.4f,%.4f,%.4f", robotEncoderPosition(0, 0), robotEncoderPosition(1, 0), robotEncoderPosition(2, 0), robotVelocity(0, 0), robotVelocity(1, 0));
	int dataLen = strlen(bump);
	if(spdUDPClient != NULL){
		spdUDPClient->sendData((unsigned char*)bump, dataLen);
	}
	for(int i = 0; i < MAX_CLIENTS; i++){
		if(isConnected(i) && isWebSocket(i)){
			spdWSServer->sendMsg(i, 0x00, bump, dataLen);
		}
	}
	delete[] bump;
}


void GeneralController::onRawPositionUpdate(double x, double y, double theta, double deltaDistance, double deltaDegrees){
	robotRawEncoderPosition(0, 0) = x;
	robotRawEncoderPosition(1, 0) = y;
	robotRawEncoderPosition(2, 0) = theta;

	robotRawDeltaPosition(0, 0) = deltaDistance;
	robotRawDeltaPosition(1, 0) = deltaDegrees;
}

void GeneralController::onSonarsDataUpdate(std::vector<PointXY*>* data){

}

void GeneralController::onBatteryChargeStateChanged(char battery){
	if(!setChargerPosition && ((int)battery) > 0){
		if(currentSector != NULL){
			
			for(int i = 0; i< currentSector->featuresSize(); i++){
				if(currentSector->featureAt(i)->name == SEMANTIC_FEATURE_CHARGER_STR){
					setChargerPosition = true;
					float xpos = currentSector->featureAt(i)->xpos;
					float ypos = currentSector->featureAt(i)->ypos;
					RNUtils::sleep(100);
					setRobotPosition(xpos, ypos, M_PI/2);
				}
			}
		}
	}
}

void GeneralController::onLaserScanCompleted(LaserScan* laser){
	
	float angle_min = laser->getAngleMin();
	float angle_max = laser->getAngleMax();
	float angle_increment = laser->getIncrement();
	std::vector<float>* data = laser->getRanges();
	std::vector<float> dataIntensities = *laser->getIntensities();

	std::vector<int> dataIndices = stats::findIndicesHigherThan(dataIntensities, 0);

	lockLaserLandmarks();

	laserLandmarks->clear();
	RNLandmark* current = new RNLandmark;


	for(int i = 0; i < dataIndices.size(); i++){
		if(i < dataIndices.size() - 1){
			if((dataIndices[i + 1] - dataIndices[i]) <= 10){
				current->addPoint(data->at(dataIndices[i]), (angle_max - ((float)dataIndices[i] * angle_increment)));
				
			} else {
				current->addPoint(data->at(dataIndices[i]), (angle_max - ((float)dataIndices[i] * angle_increment)));
				if(current->size() > 1){
					laserLandmarks->add(current);
				}
				current = new RNLandmark;
			}
		} else {
			if((dataIndices[i] - dataIndices[i - 1]) <= 10){
				current->addPoint(data->at(dataIndices[i]), (angle_max - ((float)dataIndices[i] * angle_increment)));
				if(current->size() > 1){
					laserLandmarks->add(current);
				}
			} else if(current->size() > 0){
				if(current->size() > 1){
					laserLandmarks->add(current);
				}
			}
		}
	}
	
	std::ostringstream buffFile;
	for(int i = 0; i < laserLandmarks->size(); i++){
		
		//RNUtils::printLn("Questa Merda prima alla correzione [%d] è {d: %f, a: %f}", i, laserLandmarks->at(i)->getPointsXMean() + LANDMARK_RADIUS, laserLandmarks->at(i)->getPointsYMean());
		Matrix Pkl = Matrix::eye(2);
		Matrix Rkl = std::pow(0.003, 2) * Matrix::eye(laserLandmarks->at(i)->size());
		Matrix Zk(laserLandmarks->at(i)->size(), 1);
		Matrix Zke(laserLandmarks->at(i)->size(), 1);
		Matrix Hkl(laserLandmarks->at(i)->size(), 2);
		Matrix Pc(2, 1);
		Pc(0, 0) = laserLandmarks->at(i)->getPointsXMean() + LANDMARK_RADIUS;
		Pc(1, 0) = laserLandmarks->at(i)->getPointsYMean();

		//fprintf(file, "$MEAN_LASER_RAW\t%d\t%f\t%f\n", (i + 1), Pc(0, 0), Pc(1, 0));
		for(int j = 0; j < laserLandmarks->at(i)->size(); j++){
			Zk(j, 0) = laserLandmarks->at(i)->getPointAt(j)->getX();
			Zke(j, 0) = Pc(0, 0) * cos(Pc(1, 0) - laserLandmarks->at(i)->getPointAt(j)->getY()) - LANDMARK_RADIUS * cos(asin((Pc(0, 0) / LANDMARK_RADIUS) * sin(Pc(1, 0) - laserLandmarks->at(i)->getPointAt(j)->getY())));

			//fprintf(file, "$LASER_POINT\t%d\t%d\t%f\t%f\t%f\n", (i + 1), (j + 1), Zk(j, 0), Zke(j, 0), laserLandmarks->at(i)->getPointAt(j)->getY());

			float calc = (1 / LANDMARK_RADIUS) * (1 / std::sqrt(1 - ((std::pow(Pc(0, 0), 2) * std::pow(sin(Pc(1, 0) - laserLandmarks->at(i)->getPointAt(j)->getY()), 2))/(std::pow(LANDMARK_RADIUS, 2)))));
			Hkl(j, 0) = cos(Pc(1, 0) - laserLandmarks->at(i)->getPointAt(j)->getY()) + Pc(0, 0) * std::pow(sin(Pc(1, 0) - laserLandmarks->at(i)->getPointAt(j)->getY()), 2) * calc;
			Hkl(j, 1) = -Pc(0, 0) * sin(Pc(1, 0) - laserLandmarks->at(i)->getPointAt(j)->getY()) + std::pow(Pc(0, 0), 2) * sin(Pc(1, 0) - laserLandmarks->at(i)->getPointAt(j)->getY()) * cos(Pc(1, 0) - laserLandmarks->at(i)->getPointAt(j)->getY()) * calc;
		}

		Matrix Skl = Hkl * Pkl * ~Hkl + Rkl;
		Matrix Wkl = Pkl * ~Hkl * !Skl;
		Pc = Pc + (Wkl * (Zk - Zke));
		//fprintf(file, "$MEAN_LASER_FIXED\t%d\t%f\t%f\n", (i + 1), Pc(0, 0), Pc(1, 0));
		laserLandmarks->at(i)->setPointsXMean(Pc(0, 0));
		laserLandmarks->at(i)->setPointsYMean(Pc(1, 0));

		//RNUtils::printLn("Questa Merda dopo della correzione [%d] è {d: %f, a: %f}", i, laserLandmarks->at(i)->getPointsXMean() + LANDMARK_RADIUS, laserLandmarks->at(i)->getPointsYMean());
		
	}
	unlockLaserLandmarks();
}

void GeneralController::onSecurityDistanceWarningSignal(){
	RNUtils::printLn("Alert on something is blocking Doris to achieve goal...");
}

void GeneralController::onSecurityDistanceStopSignal(){
	if(currentSector != NULL){
		std::vector<std::string> spltdSequence = RNUtils::split((char*)currentSector->getSequence().c_str(), ",");
		if(lastSiteVisitedIndex > 0 and (lastSiteVisitedIndex < spltdSequence.size() - 1)){
			int goalId = atoi(spltdSequence.at(lastSiteVisitedIndex + 1).c_str());
			RNUtils::printLn("Doris Stopped. Could not achieve next goal from current position...");
		}
	}
}

void GeneralController::startSitesTour(){
	pthread_t tourThread;
	stopCurrentTour();
	
	RNUtils::printLn("Starting tour in current sector...");
	pthread_create(&tourThread, NULL, sitesTourThread, (void *)(this));
}

void* GeneralController::sitesTourThread(void* object){
	GeneralController* self = (GeneralController*)object;
	self->keepTourAlive = RN_YES;

    std::vector<std::string> spltdSequence = RNUtils::split((char*)self->currentSector->getSequence().c_str(), ",");
    bool movedToCenter = false;
	
	while(self->keepTourAlive == RN_YES){
        int goalIndex = self->lastSiteVisitedIndex + 1;
        RNUtils::printLn("Goal Index: %d, lastSiteVisitedIndex: %d, splitted: %d, sequence: %s", goalIndex, self->lastSiteVisitedIndex, spltdSequence.size(), (char*)self->currentSector->getSequence().c_str());
        int goalId = atoi(spltdSequence.at(goalIndex).c_str());
        s_site* destinationSite = self->currentSector->findSiteById(goalId);
        
        RNUtils::sleep(100);
        if(destinationSite != NULL){
        	s_feature* linkedFeature = NULL;
        	if(destinationSite->linkedFeatureId != RN_NONE){
        		linkedFeature = self->currentSector->findFeatureById(destinationSite->linkedFeatureId);
        	}

        	PointXY phantomPoint(destinationSite->xpos, destinationSite->ypos);

        	if(linkedFeature != NULL and linkedFeature->name == SEMANTIC_FEATURE_DOOR_STR){
    			if(linkedFeature->width > linkedFeature->height){
    				phantomPoint.setY(self->robotRawDeltaPosition(1, 0));
    			} else {
    				phantomPoint.setX(self->robotRawDeltaPosition(0, 0));
    			}
    			RNUtils::printLn("Door site detected. Creating a virtual point at {x: %f, y: %f}", phantomPoint.getX(), phantomPoint.getY());
    			self->moveRobotToPosition(phantomPoint.getX(), phantomPoint.getY(), 0.0);
    			while((not self->isGoalAchieved()) and (not self->isGoalCanceled()) and (self->keepTourAlive == RN_YES)) RNUtils::sleep(100);
    		}
    		if(self->hallwayDetected and not movedToCenter){
    			if(self->currentSector->getWidth() < self->currentSector->getHeight()){
    				phantomPoint.setX(self->currentSector->getWidth() / 2.0);
    				phantomPoint.setY(self->robotRawDeltaPosition(1, 0));
    				
    			} else {
    				phantomPoint.setX(self->robotRawDeltaPosition(0, 0));
    				phantomPoint.setY(self->currentSector->getHeight() / 2.0);
    			}

				movedToCenter = true;
				RNUtils::printLn("Site in hallway zone. Creating a virtual point at {x: %f, y: %f}", phantomPoint.getX(), phantomPoint.getY());
				self->moveRobotToPosition(phantomPoint.getX(), phantomPoint.getY(), 0.0);
				while((not self->isGoalAchieved()) and (not self->isGoalCanceled()) and (self->keepTourAlive == RN_YES)) RNUtils::sleep(100);
    			
    		}

        	self->moveRobotToPosition(destinationSite->xpos, destinationSite->ypos, 0.0);
        	while((not self->isGoalAchieved()) and (not self->isGoalCanceled()) and (self->keepTourAlive == RN_YES)) RNUtils::sleep(100);
        	if (self->isGoalAchieved()) {
	            self->lastSiteVisitedIndex++;
	            if (self->currentSector->isSitesCyclic()){
	            	self->lastSiteVisitedIndex = RN_NONE;
	            	movedToCenter = false;
	            } else if((self->lastSiteVisitedIndex == (spltdSequence.size() - 1))) {
	                self->lastSiteVisitedIndex = RN_NONE;
	                self->stopRobot();
	            }
	        }
	        if (self->isGoalCanceled()) {
	            self->keepTourAlive = RN_NO;
	        }
        } else {
        	RNUtils::printLn("Non existing site %d. Check sequence...", goalId);
        	self->keepTourAlive = RN_NO;
        }
	}
	self->keepTourAlive = RN_NO;
	return NULL;
}

void GeneralController::stopCurrentTour(){
	if(keepTourAlive == RN_YES){
		keepTourAlive = MAYBE;
		RNUtils::printLn("Stopping all tours...");
		while(keepTourAlive != RN_NO) RNUtils::sleep(100);
		RNUtils::printLn("All tours stopped...");
	}
}


int GeneralController::initializeKalmanVariables(){
	int result = RN_NONE;
	if(currentSector != NULL){
		float uX, uY, uTh;

		int laserLandmarksCount = currentSector->landmarksSizeByType(XML_SENSOR_TYPE_LASER_STR);
		int cameraLandmarksCount = currentSector->landmarksSizeByType(XML_SENSOR_TYPE_CAMERA_STR);
		int rfidLandmarksCount = currentSector->landmarksSizeByType(XML_SENSOR_TYPE_RFID_STR);

		int totalLandmarks = 0;

		fl::Trapezoid* xxKK = new fl::Trapezoid("Xx(k|k)", robotRawEncoderPosition(0, 0) + robotConfig->navParams->initialPosition->xZone->x1, robotRawEncoderPosition(0, 0) + robotConfig->navParams->initialPosition->xZone->x2, robotRawEncoderPosition(0, 0) + robotConfig->navParams->initialPosition->xZone->x3, robotRawEncoderPosition(0, 0) + robotConfig->navParams->initialPosition->xZone->x4);
		
		fl::Trapezoid* xyKK = new fl::Trapezoid("Xy(k|k)", robotRawEncoderPosition(1, 0) + robotConfig->navParams->initialPosition->yZone->x1, robotRawEncoderPosition(1, 0) + robotConfig->navParams->initialPosition->yZone->x2, robotRawEncoderPosition(1, 0) + robotConfig->navParams->initialPosition->yZone->x3, robotRawEncoderPosition(1, 0) + robotConfig->navParams->initialPosition->yZone->x4);	
		
		fl::Trapezoid* xThKK = new fl::Trapezoid("XTh(k|k)", robotRawEncoderPosition(2, 0) + robotConfig->navParams->initialPosition->thZone->x1, robotRawEncoderPosition(2, 0) + robotConfig->navParams->initialPosition->thZone->x2, robotRawEncoderPosition(2, 0) + robotConfig->navParams->initialPosition->thZone->x3, robotRawEncoderPosition(2, 0) + robotConfig->navParams->initialPosition->thZone->x4);	
		
		fl::Trapezoid* vdK1 = new fl::Trapezoid("Vd(k + 1)", robotConfig->navParams->processNoise->dZone->x1, robotConfig->navParams->processNoise->dZone->x2, robotConfig->navParams->processNoise->dZone->x3, robotConfig->navParams->processNoise->dZone->x4);
		
		fl::Trapezoid* vThK1 = new fl::Trapezoid("VTh(k + 1)", robotConfig->navParams->processNoise->thZone->x1, robotConfig->navParams->processNoise->thZone->x2, robotConfig->navParams->processNoise->thZone->x3, robotConfig->navParams->processNoise->thZone->x4);	

		uX = fuzzy::FStats::uncertainty(xxKK->getVertexA(), xxKK->getVertexB(), xxKK->getVertexC(), xxKK->getVertexD());
		uY = fuzzy::FStats::uncertainty(xyKK->getVertexA(), xyKK->getVertexB(), xyKK->getVertexC(), xyKK->getVertexD());
		uTh = fuzzy::FStats::uncertainty(xThKK->getVertexA(), xThKK->getVertexB(), xThKK->getVertexC(), xThKK->getVertexD());
		RNUtils::printLn("Position {ux: %f, uy: %f, uth:%f}", uX, uY, uTh);

		P(0, 0) = uX; 	P(0, 1) = 0; 	P(0, 2) = 0;
		P(1, 0) = 0; 	P(1, 1) = uY;	P(1, 2) = 0;
		P(2, 0) = 0;	P(2, 1) = 0;	P(2, 2) = uTh;
		
		// Variances and Covariances Matrix of Process noise Q

		uX = fuzzy::FStats::uncertainty(vdK1->getVertexA(), vdK1->getVertexB(), vdK1->getVertexC(), vdK1->getVertexD());
		uTh = fuzzy::FStats::uncertainty(vThK1->getVertexA(), vThK1->getVertexB(), vThK1->getVertexC(), vThK1->getVertexD());
		
		Q(0, 0) = uX; 	Q(0, 1) = 0;
		Q(1, 0) = 0; 	Q(1, 1) = uTh;
		RNUtils::printLn("Process {ux: %f, uth:%f}", uX, uTh);
		float laserUX = 0, laserUTh = 0, cameraUX = 0, cameraUTh = 0, rfidUX = 0, rfidUTh = 0;

		for(int i = 0; i < robotConfig->navParams->sensors->size(); i++){
			s_trapezoid* dZone;
			s_trapezoid* thZone;
			dZone = robotConfig->navParams->sensors->at(i)->observationNoise->dZone;
			thZone = robotConfig->navParams->sensors->at(i)->observationNoise->thZone;

			if(robotConfig->navParams->sensors->at(i)->type == XML_SENSOR_TYPE_LASER_STR){
				
				laserUX = fuzzy::FStats::uncertainty(dZone->x1, dZone->x2, dZone->x3, dZone->x4);
				laserUTh = fuzzy::FStats::uncertainty(thZone->x1, thZone->x2, thZone->x3, thZone->x4);
				//laserUX = 0.1355e-3;      //forced meanwhile
				//laserUTh = 0.3206e-3;
				RNUtils::printLn("Laser {ux: %f, uth:%f}", laserUX, laserUTh);

			} else if(robotConfig->navParams->sensors->at(i)->type == XML_SENSOR_TYPE_CAMERA_STR){
				
				cameraUX = fuzzy::FStats::uncertainty(dZone->x1, dZone->x2, dZone->x3, dZone->x4);
				//cameraUTh = fuzzy::FStats::uncertainty(thZone->x1, thZone->x2, thZone->x3, thZone->x4);

				cameraUTh = 0.00022;   //forced meanwhile

				cameraAngleVariance = cameraUTh;

			} else if(robotConfig->navParams->sensors->at(i)->type == XML_SENSOR_TYPE_RFID_STR){
				
				rfidUX = fuzzy::FStats::uncertainty(dZone->x1, dZone->x2, dZone->x3, dZone->x4);
				rfidUTh = fuzzy::FStats::uncertainty(thZone->x1, thZone->x2, thZone->x3, thZone->x4);
			}
		}
		
		if(laserSensorActivated){
			totalLandmarks += laserLandmarksCount;
		}

		if(cameraSensorActivated){
			totalLandmarks += cameraLandmarksCount;
		}

		if(rfidSensorActivated){
			totalLandmarks += rfidLandmarksCount;
		}
 		
		R = Matrix(2 * totalLandmarks, 2 * totalLandmarks);
		int laserIndex = 0, cameraIndex = 0, rfidIndex = 0;
		if(laserSensorActivated){
			cameraIndex += laserLandmarksCount;
			rfidIndex += laserLandmarksCount;
		}
		if(cameraSensorActivated){
			rfidIndex += cameraLandmarksCount;
		}

		for(int i = 0; i < currentSector->landmarksSize(); i++){
			if(laserSensorActivated){
				if(currentSector->landmarkAt(i)->type == XML_SENSOR_TYPE_LASER_STR){
					R(2 * laserIndex, 2 * laserIndex) = laserUX;
					R(2 * laserIndex + 1, 2 * laserIndex + 1) = laserUTh;
					laserIndex++;
				}	
			}

			if(cameraSensorActivated){
				if(currentSector->landmarkAt(i)->type == XML_SENSOR_TYPE_CAMERA_STR){
					R(2 * cameraIndex, 2 * cameraIndex) = cameraUX;
					R(2 * cameraIndex + 1, 2 * cameraIndex + 1) = cameraUTh;
					cameraIndex++;
				}
			}

			if(rfidSensorActivated){
				if(currentSector->landmarkAt(i)->type == XML_SENSOR_TYPE_RFID_STR){
					R(2 * rfidIndex, 2 * rfidIndex) = rfidUX;
					R(2 * rfidIndex + 1, 2 * rfidIndex + 1) = rfidUTh;
					rfidIndex++;
				}
			}
		}
		kalmanFuzzy->clear();
		kalmanFuzzy->push_back(xxKK);
		kalmanFuzzy->push_back(xyKK);
		kalmanFuzzy->push_back(xThKK);
		
		kalmanFuzzy->push_back(vdK1);
		kalmanFuzzy->push_back(vThK1);

		result = 0;
	} 
	return result;
}

bool GeneralController::isLaserSensorActivated(){
	return laserSensorActivated;
}

bool GeneralController::isCameraSensorActivated(){
	return cameraSensorActivated;
}

bool GeneralController::isRfidSensorActivated(){
	return rfidSensorActivated;
}

Matrix GeneralController::getP(){
	return P;
}

Matrix GeneralController::getQ(){
	return Q;
}

Matrix GeneralController::getR(){
	return R;
}

float GeneralController::getCameraAngleVariance(){
	return cameraAngleVariance;
}

MapSector* GeneralController::getCurrentSector(){
	return currentSector;
}

Matrix GeneralController::getRawEncoderPosition(){
	return this->robotRawEncoderPosition;
}

Matrix GeneralController::getRawDeltaPosition(){
	return this->robotRawDeltaPosition;
}

int GeneralController::getCurrenMapId(){
	return this->currentMapId;
}

int GeneralController::getCurrentSectorId(){
	int result = RN_NONE;
	if(currentSector != NULL){
		result = currentSector->getId();
	}
	return result;
}

int GeneralController::getNextSectorId(){
	return this->nextSectorId;
}

void GeneralController::setNextSectorId(int id){
	this->nextSectorId = id;
}

int GeneralController::getLastVisitedNode(){
	return this->lastSiteVisitedIndex;
}

void GeneralController::setLastVisitedNode(int id){
	this->lastSiteVisitedIndex = id;
}

RNLandmarkList* GeneralController::getLaserLandmarks(){
	return this->laserLandmarks;
}

RNLandmarkList* GeneralController::getVisualLandmarks(){
	return this->visualLandmarks;
}

RNLandmarkList* GeneralController::getRFIDLandmarks(){
	return this->rfidLandmarks;
}

void GeneralController::setVisualLandmarks(RNLandmarkList* landmarks){
	lockVisualLandmarks();
	this->visualLandmarks = landmarks;
	unlockVisualLandmarks();
}

void GeneralController::setRFIDLandmarks(RNLandmarkList* landmarks){
	lockRFIDLandmarks();
	this->rfidLandmarks = landmarks;
	unlockRFIDLandmarks();
}

PointXY GeneralController::getNextSectorCoord(){
	return this->nextSectorCoord;
}

bool GeneralController::isFirstQuadrant(float angle){
	return (angle >= 0 && angle <= (M_PI /2));
}

bool GeneralController::isSecondQuadrant(float angle){
	return (angle > (M_PI/2) && angle <= (M_PI));
}

bool GeneralController::isThirdQuadrant(float angle){
	return (angle <= (-M_PI/2) && angle > (-M_PI));
}

bool GeneralController::isFouthQuadrant(float angle){
	return (angle < 0 && angle > (-M_PI/2));
}

int GeneralController::lockLaserLandmarks(){
	return pthread_mutex_lock(&laserLandmarksLocker);
}

int GeneralController::unlockLaserLandmarks(){
	return pthread_mutex_unlock(&laserLandmarksLocker);
}

int GeneralController::lockRFIDLandmarks(){
	return pthread_mutex_lock(&rfidLandmarksLocker);
}

int GeneralController::unlockRFIDLandmarks(){
	return pthread_mutex_unlock(&rfidLandmarksLocker);
}

int GeneralController::lockVisualLandmarks(){
	return pthread_mutex_lock(&visualLandmarksLocker);
}

int GeneralController::unlockVisualLandmarks(){
	return pthread_mutex_unlock(&visualLandmarksLocker);
}

float GeneralController::getRobotHeight(){
	return (robotConfig != NULL ? robotConfig->height : 0.0);
}

void GeneralController::onSensorsScanCompleted(){
	std::ostringstream buffer_str;
	int mapId = currentMapId;
	int sectorId = RN_NONE;
	if(currentSector != NULL){
		sectorId = currentSector->getId();
	}
	buffer_str.clear();
	buffer_str << "$DORIS|" << mapId << "," 
				<< sectorId << "," 
				<< emotionsTimestamp.str() << "," 
				<< mappingSectorTimestamp.str() << "," 
				<< mappingLandmarksTimestamp.str() << "," 
				<< mappingFeaturesTimestamp.str() << "," 
				<< mappingSitesTimestamp.str() << "," 
				<< laserLandmarks->size() << "," 
				<< visualLandmarks->size();

	if(spdUDPClient != NULL){
		spdUDPClient->sendData((unsigned char*)buffer_str.str().c_str(), buffer_str.str().length());
	}
	for(int i = 0; i < MAX_CLIENTS; i++){
		if(isConnected(i) && isWebSocket(i)){
			spdWSServer->sendMsg(i, 0x00, buffer_str.str().c_str(), buffer_str.str().length());
		}
	}

}
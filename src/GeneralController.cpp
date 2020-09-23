#include "GeneralController.h"

#include "RNRecurrentTask.h"
#include "RNRecurrentTaskMap.h"
#include "RNLaserTask.h"
#include "RNLocalizationTask.h"
#include "RNEkfTask.h"
#include "RNPkfTask.h"
#include "RNUkfTask.h"
#include "RNEskfTask.h"
#include "RNUskfTask.h"
#include "RNCameraTask.h"
#include "RNOmnicameraTask.h"
#include "RNRFIdentificationTask.h"
#include "RNEmotionsTask.h"
#include "RNDialogsTask.h"
#include "RNGesturesTask.h"
#include "RNArmTask.h"
#include "RNTourThread.h"

const double AntennaData::TX_GAIN = 9.75;
const double AntennaData::FREQUENCY = 866.9e6;
const double AntennaData::C = 3e8;
const double AntennaData::TX_POWER_OFFSET_DBM = -10.0;
const double AntennaData::TX_POWER_INDEX_MULTIPLIER = 0.25;

GeneralController::GeneralController(const char* port):RobotNode(port){
	
	this->maestroControllers = new SerialPort();
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

	this->laserSensorActivated = true;
	this->cameraSensorActivated = false;
	this->rfidSensorActivated = false;

    this->lastSiteVisitedIndex = RN_NONE;
	laserLandmarks = new RNLandmarkList();
	visualLandmarks = new RNLandmarkList();
	kalmanFuzzy = new std::vector<fl::Trapezoid*>();
	
	robotEncoderPosition = Matrix(3, 1);
	robotVelocity = Matrix (2, 1);
	
	P = Matrix(3, 3);
	Q = Matrix(2, 2);

	b = Matrix(5, 1);
	B = Matrix(5, 5);

	xmlFaceFullPath = RNUtils::getApplicationPath() + XML_FILE_PATH;
	xmlMapsFullPath = RNUtils::getApplicationPath() + XML_FILE_MAPS_PATH;
	xmlSectorsPath = RNUtils::getApplicationPath() + XML_FILE_SECTORS_PATH;
	xmlRobotConfigFullPath = RNUtils::getApplicationPath() + XML_FILE_ROBOT_CONFIG_PATH;
	
	this->currentSector = NULL;
	this->nextSectorId = RN_NONE;

	loadRobotConfig();
	pthread_mutex_init(&laserLandmarksLocker, NULL);
	pthread_mutex_init(&rfidLandmarksLocker, NULL);
	pthread_mutex_init(&visualLandmarksLocker, NULL);
	pthread_mutex_init(&rawPositionLocker, NULL);
	pthread_mutex_init(&currentSectorLocker, NULL);
	
	this->tts = new DorisLipSync(this->maestroControllers, this->getFaceId());

	tasks = new RNRecurrentTaskMap(this);

	laserTask = new RNLaserTask(this);
	//omnidirectionalTask = new RNOmnicameraTask(this, "Omnidirectional Task");
	rfidTask = new RNRFIdentificationTask(this);
	dialogs = new RNDialogsTask(this);
	gestures = new RNGesturesTask(this);
	//armGestures = new RNArmTask(this);
	emotions = new RNEmotionsTask(this);
	tourThread = new RNTourThread(this);
	//eyesCameras = new RNCameraTask(this);

	if(robotConfig->localization == XML_LOCALIZATION_ALGORITHM_EKF_STR){
		localization = new RNEkfTask(this);
	} else if(robotConfig->localization == XML_LOCALIZATION_ALGORITHM_PKF_STR){
		localization = new RNPkfTask(this);
	} else if(robotConfig->localization == XML_LOCALIZATION_ALGORITHM_UKF_STR){
		localization = new RNUkfTask(this);
	} else if(robotConfig->localization == XML_LOCALIZATION_ALGORITHM_ESKF_STR){
		localization = new RNEskfTask(this);
	} else if(robotConfig->localization == XML_LOCALIZATION_ALGORITHM_USKF_STR){
		localization = new RNUskfTask(this);
	}

	////Tasks added:
	tasks->addTask(laserTask);
	//tasks->addTask(omnidirectionalTask);
	tasks->addTask(rfidTask);
	tasks->addTask(dialogs);
	tasks->addTask(gestures);
	//tasks->addTask(armGestures);
	tasks->addTask(emotions);
	//tasks->addTask(eyesCameras);
	tasks->addTask(localization);

	
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
}

GeneralController::~GeneralController(void){
	
	disconnect();
	delete tasks;
	RNUtils::printLn("Deleted tasks...");
	for (int i = 0; i < kalmanFuzzy->size(); i++){
		delete kalmanFuzzy->at(i);
	}
	kalmanFuzzy->clear();
	delete kalmanFuzzy;
	RNUtils::printLn("Deleted kalmanFuzzy...");
	
	delete maestroControllers;
	RNUtils::printLn("Deleted maestroControllers...");
	delete spdWSServer;
	RNUtils::printLn("Deleted spdWSServer...");
	delete currentSector;
	RNUtils::printLn("Deleted currentSector...");
	delete robotConfig;
	RNUtils::printLn("Deleted robotConfig...");

	unlockLaserLandmarks();
	unlockVisualLandmarks();
	unlockRFIDLandmarks();
	RNUtils::printLn("unlocked all mutex...");
	delete rfidLandmarks;
	RNUtils::printLn("Deleted rfidLandmarks...");
	delete laserLandmarks;
	RNUtils::printLn("Deleted laserLandmarks...");
	delete visualLandmarks;
	RNUtils::printLn("Deleted visualLandmarks...");

	pthread_mutex_destroy(&currentSectorLocker);
	RNUtils::printLn("Deleted currentSectorLocker...");
	pthread_mutex_destroy(&rawPositionLocker);
	RNUtils::printLn("Deleted rawPositionLocker...");
	pthread_mutex_destroy(&laserLandmarksLocker);
	RNUtils::printLn("Deleted laserLandmarksLocker...");
	pthread_mutex_destroy(&rfidLandmarksLocker);
	RNUtils::printLn("Deleted rfidLandmarksLocker...");
	pthread_mutex_destroy(&visualLandmarksLocker);
	RNUtils::printLn("Deleted visualLandmarksLocker...");
	
	closeConnection();
	
}
const char* GeneralController::getClassName() const{
    return "GeneralController";
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
			if(this->getClientUDP(socketIndex) != NULL){
				this->getClientUDP(socketIndex)->closeConnection();
				this->connectClientUDP(socketIndex, NULL);
			}
		}
	}
	RNUtils::printLn("Clients connected: %d", clientsConnected);
}
void GeneralController::onMsg(int socketIndex, char* cad, unsigned long long int length){//callback for client and server
	cad[length] = 0;
	unsigned char function = *(cad++);
	std::string local_buffer_out = "";
	std::string jsonResponse = "";
	std::string servo_positions = "";
	std::string mapsAvailable = "";
	std::string availablePrograms = "";
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
	double lin_vel = 0, ang_vel = 0;
	int cameraCount = 0;
	int videoDevice = 0;
	int videoTxPort = 0;
	int indexAssigned = 0;
	int idArmMotor;
	int angleArmMotor;
	std::vector<uint16_t> allMotors;
	double x, y, theta;

	bool granted = isPermissionNeeded(function) && (socketIndex == getTokenOwner());
	RNUtils::printLn("Function: 0x%x (%d) executed from: %s", function, function, getClientIPAddress(socketIndex));
	switch (function){
		case 0x00:
			RNUtils::printLn("Command 0x00. Static Faces Requested");
			gestures->getGestures(jsonResponse);
			sendMsg(socketIndex, 0x00, (char*)jsonResponse.c_str(), (unsigned int)(jsonResponse.length())); 
			break;
		case 0x01:
			RNUtils::printLn("Command 0x01. List of programs requested");
			getAvailablePrograms(availablePrograms);
			sendMsg(socketIndex, 0x01, (char*)availablePrograms.c_str(), (unsigned int)availablePrograms.length()); 
			break;
		case 0x02:
			RNUtils::printLn("Command 0x02. Saving New Face Frame");
			gestures->saveGesture(cad);
			break;
		case 0x03:
			if(granted){
				tourThread->loadProgram(std::string(PROGRAMS_DIR_PATH) + std::string(cad));
			} else {
				RNUtils::printLn("Command 0x03. Loading program denied to %s", getClientIPAddress(socketIndex));
				sendMsg(socketIndex, 0x03, (char*)jsonControlError.c_str(), (unsigned int)jsonControlError.length());
			}
			break;
		case 0x04:
			RNUtils::printLn("Command 0x04. Modifying Face frame");
			if(granted){
				gestures->modifyGesture(cad);
			} else {
				RNUtils::printLn("Command 0x04. Modifying Face denied to %s", getClientIPAddress(socketIndex));
				sendMsg(socketIndex, 0x04, (char*)jsonControlError.c_str(), (unsigned int)jsonControlError.length());
			}
			break;
		case 0x05:
			
			break;
		case 0x06:
			RNUtils::printLn("Command 0x06. Removing Face");
			if(granted){
				gestures->removeGesture(cad);
			} else {
				RNUtils::printLn("Command 0x06. Removing Face denied to %s", getClientIPAddress(socketIndex));
				sendMsg(socketIndex, 0x06, (char*)jsonControlError.c_str(), (unsigned int)jsonControlError.length());
			}
			break;
		case 0x07:
			RNUtils::printLn("Command 0x07. Setting Gesture Id: %s", cad);
			if(granted){
				jsonResponse = "";
				gestures->setGesture(cad);
				sendMsg(socketIndex, 0x07, (char*)jsonResponse.c_str(), (unsigned int)jsonResponse.length());
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
			
			break;
		case 0x0A:
			
			break;
		case 0x0B:
			RNUtils::printLn("Command 0x0B. Text to speech message");
			if(granted){
				dialogs->setInputMessage(cad);
				//tts->textToViseme(cad);
				sendMsg(socketIndex, 0x08, (char*)jsonRobotOpSuccess.c_str(), (unsigned int)jsonRobotOpSuccess.length());
			} else {
				RNUtils::printLn("Command 0x0B. Text to speech message denied to %s", getClientIPAddress(socketIndex));
				sendMsg(socketIndex, 0x0B, (char*)jsonControlError.c_str(), (unsigned int)jsonControlError.length());
			}
			break;
		case 0x0C:
			RNUtils::printLn("Command 0x0C. Static Arm Gestures Requested");
			armGestures->getGestures(XML_STATIC_GESTURES_TYPE_ID, jsonResponse);
			sendMsg(socketIndex, 0x0C, (char*)jsonResponse.c_str(), (unsigned int)(jsonResponse.length())); 
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
				if(getCurrentSector() != NULL){
					tourThread->go();
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
			
			break;	
		case 0x13:
			if(granted){
				getPositions(cad, x, y, theta);
				if(getCurrentSector() != NULL){
					setRobotPosition(x, y, theta);
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
				loadSector(mapId, sectorId);				
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
				pthread_mutex_lock(&currentSectorLocker);
				sectorId = currentSector->getId();
				mapId = currentSector->getMapId();
				pthread_mutex_unlock(&currentSectorLocker);
			}
			getSectorInformationLandmarks(mapId, sectorId, sectorInformation);
			sendMsg(socketIndex, 0x17, (char*)sectorInformation.c_str(), (unsigned int)sectorInformation.length()); 
			break;
		case 0x18:
			if(cad != NULL){
				getMapSectorId(cad, mapId, sectorId);
			} else {
				pthread_mutex_lock(&currentSectorLocker);
				sectorId = currentSector->getId();
				mapId = currentSector->getMapId();
				pthread_mutex_unlock(&currentSectorLocker);
			}
			getSectorInformationFeatures(mapId, sectorId, sectorInformation);
			sendMsg(socketIndex, 0x18, (char*)sectorInformation.c_str(), (unsigned int)sectorInformation.length()); 
			break;
		case 0x19:
			if(cad != NULL){
				getMapSectorId(cad, mapId, sectorId);
			} else {
				pthread_mutex_lock(&currentSectorLocker);
				sectorId = currentSector->getId();
				mapId = currentSector->getMapId();
				pthread_mutex_unlock(&currentSectorLocker);
			}
			getSectorInformationSites(mapId, sectorId, sectorInformation);
			sendMsg(socketIndex, 0x19, (char*)sectorInformation.c_str(), (unsigned int)sectorInformation.length()); 
			break;
		case 0x1A:
			if(cad != NULL){
				getMapSectorId(cad, mapId, sectorId);
			} else {
				pthread_mutex_lock(&currentSectorLocker);
				sectorId = currentSector->getId();
				mapId = currentSector->getMapId();
				pthread_mutex_unlock(&currentSectorLocker);
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
		case 0x24: //temporal incoming
			
			break;
		case 0x30:
			if(granted){
				setRobotPosition(0.0, 0.0, 0.0);
				sendMsg(socketIndex, 0x30, (char*)jsonRobotOpSuccess.c_str(), (unsigned int)jsonRobotOpSuccess.length());
			} else {
				RNUtils::printLn("Command 0x30. Reset odometry denied to ", getClientIPAddress(socketIndex));
				sendMsg(socketIndex, 0x30, (char*)jsonSectorNotLoadedError.c_str(), (unsigned int)jsonSectorNotLoadedError.length());
			}
			break;
		case 0x31:
			
			break;
		//arm functions
		case 0x40:
			if(granted){
				if(armGestures){
					armGestures->setGesture(cad);
				}
			} else {
				RNUtils::printLn("Command 0x40. Preset arm movement denied to ", getClientIPAddress(socketIndex));
				sendMsg(socketIndex, 0x40, (char*)jsonSectorNotLoadedError.c_str(), (unsigned int)jsonSectorNotLoadedError.length());
			}
			break;
		case 0x41:
			if(granted){
				getArmSingleMotorInfo(cad, idArmMotor, angleArmMotor);
				if(armGestures){
					armGestures->moveSingleMotor(idArmMotor, angleArmMotor);
				}
			} else {
				RNUtils::printLn("Command 0x41. Single arm motor movement denied to ", getClientIPAddress(socketIndex));
				sendMsg(socketIndex, 0x41, (char*)jsonSectorNotLoadedError.c_str(), (unsigned int)jsonSectorNotLoadedError.length());
			}
			break;
		case 0x42:
			if(granted){
				getArmAllMotorsInfo(cad, allMotors);
				if(armGestures){
					armGestures->moveAllMotors(allMotors);
				}
			} else {
				RNUtils::printLn("Command 0x42. Bulk motor movement denied to ", getClientIPAddress(socketIndex));
				sendMsg(socketIndex, 0x42, (char*)jsonSectorNotLoadedError.c_str(), (unsigned int)jsonSectorNotLoadedError.length());
			}
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
	if(cad != NULL){
		int spdUDPPort = atoi(cad);
		if(!this->isWebSocket(socketIndex)){
			if(spdUDPPort <= 0){
				throw std::invalid_argument("Error!!! Could not initialize SPD streaming");
			}
			RNUtils::printLn("Connected to Port: %d of %s", spdUDPPort, this->getClientIPAddress(socketIndex));
			UDPClient* spdUDPClient = new UDPClient(this->getClientIPAddress(socketIndex), spdUDPPort);
			this->connectClientUDP(socketIndex, spdUDPClient);
		} else {
			RNUtils::printLn("Port: %d", spdWSServer->getServerPort());
			std::ostringstream convert;
			convert << spdWSServer->getServerPort();
			std::string jsonString = "{\"streaming\":{\"port\":\""+ convert.str() + "\",\"error\":\"0\"}}";
			sendMsg(socketIndex, 0x7F, (char*)jsonString.c_str(), (unsigned int)jsonString.length());
		}
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

void GeneralController::getArmSingleMotorInfo(char* cad, int& id, int& angle){
	char* current_number;
	int values[2] = { -1, -1 };
	int index = 0;
	current_number = std::strtok(cad, ",");
	//
	while(current_number != NULL){
		values[index++] = std::atoi(current_number);
		current_number = std::strtok(NULL, ",");
	}
	id = values[0];
	angle = values[1];
	delete current_number;
}

void GeneralController::getArmAllMotorsInfo(char* cad, std::vector<uint16_t>& motors){
	char* current_number;
	motors.clear();
	int index = 0;
	current_number = std::strtok(cad, ",");
	while(current_number != NULL){
		motors.push_back((uint16_t)std::atoi(current_number));
		current_number = std::strtok(NULL, ",");
	}
	delete current_number;
}

void GeneralController::getPositions(char* cad, double& x, double& y, double& theta){
	char* current_number;
	double values[3] = { -1, -1, -1 };
	int index = 0;
	current_number = std::strtok(cad, ",");

	while(current_number != NULL){
		int cValue = std::atoi(current_number);
		values[index++] = (double)(((double)cValue)/1000.0);
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

void GeneralController::getVelocities(char* cad, double& lin_vel, double& ang_vel){
	char* current_number;
	double values[2] = { -1, -1 };
	int index = 0;
	current_number = std::strtok(cad, ",");
	
	while(current_number != NULL){
		int cValue = std::atoi(current_number);
		values[index++] = (double)(((double)cValue)/1000.0);
		current_number = std::strtok(NULL, ",");
	}
	lin_vel = values[0];
	ang_vel = values[1];
	delete current_number;
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


void GeneralController::getMapId(char* cad, int& mapId){
	mapId = atoi(cad);
}

bool GeneralController::loadSector(int mapId, int sectorId){
	if(localization != NULL){
		localization->kill();	
	}

	pthread_mutex_lock(&currentSectorLocker);
	std::string filename;
	bool reloadMap = true;
	bool sectorLoadedSuccess = false;
	getMapFilename(mapId, filename);
	if (filename != ""){
		xml_document<> doc;
	    xml_node<>* root_node;  

	    std::string fullSectorPath = xmlSectorsPath + filename;
	    std::ifstream the_file(fullSectorPath.c_str());
	    std::vector<char> buffer = std::vector<char>((std::istreambuf_iterator<char>(the_file)), std::istreambuf_iterator<char>());
	    buffer.push_back('\0');
	    doc.parse<0>(&buffer[0]);

	    if(currentSector != NULL){
	    	reloadMap = mapId != currentSector->getMapId();
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
	    			currentSector->setMapId(mapId);
	    			currentSector->setId(xmlSectorId);
					currentSector->setName(std::string(sector_node->first_attribute(XML_ATTRIBUTE_NAME_STR)->value()));
					currentSector->setPolygon(std::string(sector_node->first_attribute(XML_ATTRIBUTE_POLYGON_STR)->value()));
					currentSector->setWidth((double)std::atoi(sector_node->first_attribute(XML_ATTRIBUTE_WIDTH_STR)->value())/100.0);
					currentSector->setHeight((double)std::atoi(sector_node->first_attribute(XML_ATTRIBUTE_HEIGHT_STR)->value())/100.0);
					xml_node<>* landmarks_root_node = sector_node->first_node(XML_ELEMENT_LANDMARKS_STR);
					if(landmarks_root_node->first_node() !=  NULL){
						for(xml_node<>* landmark_node = landmarks_root_node->first_node(XML_ELEMENT_LANDMARK_STR); landmark_node; landmark_node = landmark_node->next_sibling()){
							s_landmark* tempLandmark = new s_landmark;

							tempLandmark->id = atoi(landmark_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value());
							tempLandmark->type = std::string(landmark_node->first_attribute(XML_ATTRIBUTE_TYPE_STR)->value());
							if(landmark_node->first_attribute(XML_ATTRIBUTE_VARIANCE_MIN_X_STR)){
								tempLandmark->varMinX = ((double)std::atoi(landmark_node->first_attribute(XML_ATTRIBUTE_VARIANCE_MIN_X_STR)->value())) / 100.0;
							}
							if(landmark_node->first_attribute(XML_ATTRIBUTE_VARIANCE_MAX_X_STR)){
								tempLandmark->varMaxX = ((double)std::atoi(landmark_node->first_attribute(XML_ATTRIBUTE_VARIANCE_MAX_X_STR)->value())) / 100.0;
							}
							if(landmark_node->first_attribute(XML_ATTRIBUTE_VARIANCE_MIN_Y_STR)){
								tempLandmark->varMinY = ((double)std::atoi(landmark_node->first_attribute(XML_ATTRIBUTE_VARIANCE_MIN_Y_STR)->value())) / 100.0;
							}
							if(landmark_node->first_attribute(XML_ATTRIBUTE_VARIANCE_MAX_Y_STR)){
								tempLandmark->varMaxY = ((double)std::atoi(landmark_node->first_attribute(XML_ATTRIBUTE_VARIANCE_MAX_Y_STR)->value())) / 100.0;
							}
							if(landmark_node->first_attribute(XML_ATTRIBUTE_VARIANCE_MIN_Z_STR)){
								tempLandmark->varMinZ = ((double)std::atoi(landmark_node->first_attribute(XML_ATTRIBUTE_VARIANCE_MIN_Z_STR)->value())) / 100.0;
							}
							if(landmark_node->first_attribute(XML_ATTRIBUTE_VARIANCE_MAX_Z_STR)){
								tempLandmark->varMaxZ = ((double)std::atoi(landmark_node->first_attribute(XML_ATTRIBUTE_VARIANCE_MAX_Z_STR)->value())) / 100.0;
							}	

							tempLandmark->xpos = ((double)std::atoi(landmark_node->first_attribute(XML_ATTRIBUTE_X_POSITION_STR)->value())) / 100.0;
							tempLandmark->ypos = ((double)std::atoi(landmark_node->first_attribute(XML_ATTRIBUTE_Y_POSITION_STR)->value())) / 100.0;
							if(landmark_node->first_attribute(XML_ATTRIBUTE_Z_POSITION_STR)){
								tempLandmark->zpos = ((double)std::atoi(landmark_node->first_attribute(XML_ATTRIBUTE_Z_POSITION_STR)->value())) / 100.0;
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
							tempFeature->width = ((double)std::atoi(features_node->first_attribute(XML_ATTRIBUTE_WIDTH_STR)->value())) / 100.0;
							tempFeature->height = ((double)std::atoi(features_node->first_attribute(XML_ATTRIBUTE_HEIGHT_STR)->value())) / 100.0;
							tempFeature->xpos = ((double)std::atoi(features_node->first_attribute(XML_ATTRIBUTE_X_POSITION_STR)->value())) / 100.0;
							tempFeature->ypos = ((double)std::atoi(features_node->first_attribute(XML_ATTRIBUTE_Y_POSITION_STR)->value())) / 100.0;
	                        
	                        currentSector->addFeature(tempFeature);
							
						}
					}

					xml_node<>* sites_root_node = sector_node->first_node(XML_ELEMENT_SITES_STR);
					if(sites_root_node->first_attribute()){
						if(sites_root_node->first_attribute(XML_ATTRIBUTE_SEQUENCE_STR)){
							std::string sequence(sites_root_node->first_attribute(XML_ATTRIBUTE_SEQUENCE_STR)->value());
							currentSector->setSequence(sequence);
						}
					}
					
					if(sites_root_node->first_node() !=  NULL){
						for(xml_node<>* site_node = sites_root_node->first_node(XML_ELEMENT_SITE_STR); site_node; site_node = site_node->next_sibling()){
							s_site* tempSite = new s_site;

							tempSite->id = std::atoi(site_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value());
							tempSite->name = std::string(site_node->first_attribute(XML_ATTRIBUTE_NAME_STR)->value());
							tempSite->xpos = ((double)std::atoi(site_node->first_attribute(XML_ATTRIBUTE_X_POSITION_STR)->value())) / 100.0;
							tempSite->ypos = ((double)std::atoi(site_node->first_attribute(XML_ATTRIBUTE_Y_POSITION_STR)->value())) / 100.0;

							if(site_node->first_attribute(XML_ATTRIBUTE_LINKED_SECTOR_ID_STR)){
								tempSite->linkedSectorId = std::atoi(site_node->first_attribute(XML_ATTRIBUTE_LINKED_SECTOR_ID_STR)->value());
							} else {
								tempSite->linkedSectorId = RN_NONE;
							}
	                        
	                        if(site_node->first_attribute(XML_ATTRIBUTE_X_COORD_STR)){
	                            tempSite->xcoord = ((double)std::atoi(site_node->first_attribute(XML_ATTRIBUTE_X_COORD_STR)->value())) / 100.0;
	                        } else {
	                            tempSite->xcoord = 0.0;
	                        }
	                        
	                        if(site_node->first_attribute(XML_ATTRIBUTE_Y_COORD_STR)){
	                            tempSite->ycoord = ((double)std::atoi(site_node->first_attribute(XML_ATTRIBUTE_Y_COORD_STR)->value())) / 100.0;
	                        } else {
	                            tempSite->ycoord = 0.0;
	                        }

							currentSector->addSite(tempSite);
						}
					}
					xml_node<>* ways_root_node = sector_node->first_node(XML_ELEMENT_WAYS_STR);
					if(ways_root_node->first_node() !=  NULL){
						for(xml_node<>* way_node = ways_root_node->first_node(XML_ELEMENT_WAY_STR); way_node; way_node = way_node->next_sibling()){
							s_way* tempWay = new s_way;
							tempWay->st = std::atoi(way_node->first_attribute(XML_ATTRIBUTE_ST_STR)->value());
							std::vector<std::string> adyList = RNUtils::split(way_node->first_attribute(XML_ATTRIBUTE_ADJACENCY_STR)->value(), ",");
							for(int i = 0; i < adyList.size(); i++){
								tempWay->adjacencies.push_back(std::stoi(adyList.at(i)));
							}
							currentSector->addWay(tempWay);
						}
					}

					xml_node<>* tags_root_node = sector_node->first_node(XML_ELEMENT_TAGS_STR);
					if(tags_root_node->first_node() !=  NULL){
						for(xml_node<>* tag_node = tags_root_node->first_node(XML_ELEMENT_TAG_STR); tag_node; tag_node = tag_node->next_sibling()){
							s_tag* tempTag = new s_tag;
							tempTag->id = std::string(tag_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value());
							tempTag->name = std::string(tag_node->first_attribute(XML_ATTRIBUTE_NAME_STR)->value());;
							tempTag->antenna = std::atoi(tag_node->first_attribute(XML_ATTRIBUTE_ANTENNA_STR)->value());
							if(tag_node->first_attribute(XML_ATTRIBUTE_LINKED_SITE_ID_STR)){
								tempTag->linkedSiteId = std::atoi(tag_node->first_attribute(XML_ATTRIBUTE_LINKED_SITE_ID_STR)->value());
							}
							currentSector->addTag(tempTag);
						}
					}
	    		}
	    	}
	    }
	    the_file.close();
	    RNUtils::printLn("Loaded new Sector {id: %d, name: %s}", sectorId, currentSector->getName().c_str());
	    sectorLoadedSuccess = true;
	}

	
	pthread_mutex_unlock(&currentSectorLocker);
	if(sectorLoadedSuccess){
		if(tourThread){
	    	if(reloadMap){
	    		tourThread->createCurrentMapGraph();
	    	}
	    	tourThread->createCurrentSectorGraph(true);
	    }
	    if(localization != NULL){
			localization->reset();	
		}

		if(rfidTask){
			rfidTask->reloadCurrentSector();
		}
		
	    RNUtils::getTimestamp(mappingSectorTimestamp);
	    
	    std::list<RNFunPointer*>::iterator sectorSubsIt;
		for(sectorSubsIt = sectorChangedSubscribers.begin(); sectorSubsIt != sectorChangedSubscribers.end(); sectorSubsIt++){
			if(dynamic_cast<RNFunPointer1<int>* >(*sectorSubsIt) != NULL){
				((RNFunPointer1<int>*)(*sectorSubsIt))->invoke(sectorId);
			}
		}
	}
    return sectorLoadedSuccess;

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
		robotConfig->localization = std::string(XML_LOCALIZATION_ALGORITHM_EKF_STR);
	}
	if(root_node->first_attribute(XML_ATTRIBUTE_ROBOT_FACE_ID_STR)){
		robotConfig->faceId = std::atoi(root_node->first_attribute(XML_ATTRIBUTE_ROBOT_FACE_ID_STR)->value());
	}
	if(root_node->first_attribute(XML_ATTRIBUTE_ROBOT_NECK_ID_STR)){
		robotConfig->neckId = std::atoi(root_node->first_attribute(XML_ATTRIBUTE_ROBOT_NECK_ID_STR)->value());
	}
	xml_node<>* rfid_conf_root_node = root_node->first_node(XML_ELEMENT_RFID_CONFIG_STR);
	s_rfid_params* rfidConfig = robotConfig->rfidConfig;
	rfidConfig->txPowerIndex = std::atoi(rfid_conf_root_node->first_attribute(XML_ATTRIBUTE_RFID_TX_POWER_STR)->value());
	rfidConfig->rxSensitivityIndex = std::atoi(rfid_conf_root_node->first_attribute(XML_ATTRIBUTE_RFID_RX_SENSITIVITY_STR)->value());

	xml_node<>* nav_root_node = root_node->first_node(XML_ELEMENT_NAV_PARAMS_STR);




	xml_node<>* initial_pos_root_node = nav_root_node->first_node(XML_ELEMENT_INITIAL_POS_STR);

	xml_node<>* pos_root_node = initial_pos_root_node->first_node(XML_ELEMENT_POS_X_DISTRIBUTION_STR);
	s_distribution* x_distribution = robotConfig->navParams->initialPosition->xDistribution;
	x_distribution->mean = (double)std::atof(pos_root_node->first_attribute(XML_ATTRIBUTE_DIST_MEAN_STR)->value());
	x_distribution->variance = (double)std::atof(pos_root_node->first_attribute(XML_ATTRIBUTE_DIST_VARIANCE_STR)->value());
	x_distribution->bias = (double)std::atof(pos_root_node->first_attribute(XML_ATTRIBUTE_DIST_BIAS_STR)->value());

	pos_root_node = initial_pos_root_node->first_node(XML_ELEMENT_POS_Y_DISTRIBUTION_STR);
	s_distribution* y_distribution = robotConfig->navParams->initialPosition->yDistribution;
	y_distribution->mean = (double)std::atof(pos_root_node->first_attribute(XML_ATTRIBUTE_DIST_MEAN_STR)->value());
	y_distribution->variance = (double)std::atof(pos_root_node->first_attribute(XML_ATTRIBUTE_DIST_VARIANCE_STR)->value());
	y_distribution->bias = (double)std::atof(pos_root_node->first_attribute(XML_ATTRIBUTE_DIST_BIAS_STR)->value());

	pos_root_node = initial_pos_root_node->first_node(XML_ELEMENT_POS_TH_DISTRIBUTION_STR);
	s_distribution* th_distribution = robotConfig->navParams->initialPosition->thDistribution;
	th_distribution->mean = (double)std::atof(pos_root_node->first_attribute(XML_ATTRIBUTE_DIST_MEAN_STR)->value());
	th_distribution->variance = (double)std::atof(pos_root_node->first_attribute(XML_ATTRIBUTE_DIST_VARIANCE_STR)->value());
	th_distribution->bias = (double)std::atof(pos_root_node->first_attribute(XML_ATTRIBUTE_DIST_BIAS_STR)->value());


	
	xml_node<>* process_noise_root_node = nav_root_node->first_node(XML_ELEMENT_PROCESS_NOISE_STR);
	pos_root_node = process_noise_root_node->first_node(XML_ELEMENT_POS_D_DISTRIBUTION_STR);
	s_distribution* d_distribution = robotConfig->navParams->processNoise->dDistribution;
	d_distribution->mean = (double)std::atof(pos_root_node->first_attribute(XML_ATTRIBUTE_DIST_MEAN_STR)->value());
	d_distribution->variance = (double)std::atof(pos_root_node->first_attribute(XML_ATTRIBUTE_DIST_VARIANCE_STR)->value());
	d_distribution->bias = (double)std::atof(pos_root_node->first_attribute(XML_ATTRIBUTE_DIST_BIAS_STR)->value());

	pos_root_node = process_noise_root_node->first_node(XML_ELEMENT_POS_TH_DISTRIBUTION_STR);
	s_distribution* th1_distribution = robotConfig->navParams->processNoise->thDistribution;
	th1_distribution->mean = (double)std::atof(pos_root_node->first_attribute(XML_ATTRIBUTE_DIST_MEAN_STR)->value());
	th1_distribution->variance = (double)std::atof(pos_root_node->first_attribute(XML_ATTRIBUTE_DIST_VARIANCE_STR)->value());
	th1_distribution->bias = (double)std::atof(pos_root_node->first_attribute(XML_ATTRIBUTE_DIST_BIAS_STR)->value());



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
			pos_root_node = observation_noise_root_node->first_node(XML_ELEMENT_POS_D_DISTRIBUTION_STR);
			s_distribution* d1_distribution = sensor->observationNoise->dDistribution;
			d1_distribution->alpha = (double)std::atof(pos_root_node->first_attribute(XML_ATTRIBUTE_ALPHA_STR)->value());
			d1_distribution->mean = (double)std::atof(pos_root_node->first_attribute(XML_ATTRIBUTE_DIST_MEAN_STR)->value());
			d1_distribution->variance = (double)std::atof(pos_root_node->first_attribute(XML_ATTRIBUTE_DIST_VARIANCE_STR)->value());
			d1_distribution->bias = (double)std::atof(pos_root_node->first_attribute(XML_ATTRIBUTE_DIST_BIAS_STR)->value());

			pos_root_node = observation_noise_root_node->first_node(XML_ELEMENT_POS_TH_DISTRIBUTION_STR);
			s_distribution* th2_distribution = sensor->observationNoise->thDistribution;
			th2_distribution->alpha = (double)std::atof(pos_root_node->first_attribute(XML_ATTRIBUTE_ALPHA_STR)->value());
			th2_distribution->mean = (double)std::atof(pos_root_node->first_attribute(XML_ATTRIBUTE_DIST_MEAN_STR)->value());
			th2_distribution->variance = (double)std::atof(pos_root_node->first_attribute(XML_ATTRIBUTE_DIST_VARIANCE_STR)->value());
			th2_distribution->bias = (double)std::atof(pos_root_node->first_attribute(XML_ATTRIBUTE_DIST_BIAS_STR)->value());

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

void GeneralController::getAvailablePrograms(std::string& availablePrograms){
	std::ostringstream buffer_str;
	DIR *folder;
	struct dirent *cont;
	folder = opendir(PROGRAMS_DIR_PATH);
	if(folder){
		while((cont = readdir(folder)) != NULL){
			if(cont->d_type == DT_REG){
				std::string fname(cont->d_name);
				if(fname.substr(fname.find_last_of(".") + 1) == PROGRAM_EXTENSION){
					buffer_str << fname << ",";
				}
			}
		}
		closedir(folder);
	} else {
		RNUtils::printLn("error: could not find programs folder...");
	}

	availablePrograms = buffer_str.str();
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
				buffer_str << (((double)atoi(sector_node->first_attribute(XML_ATTRIBUTE_WIDTH_STR)->value())) / 100) << ",";
				buffer_str << (((double)atoi(sector_node->first_attribute(XML_ATTRIBUTE_HEIGHT_STR)->value())) / 100) << ",";
				
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
							buffer_str << (((double)atoi(landmark_node->first_attribute(XML_ATTRIBUTE_X_POSITION_STR)->value())) / 100) << ",";
							buffer_str << (((double)atoi(landmark_node->first_attribute(XML_ATTRIBUTE_Y_POSITION_STR)->value())) / 100) << ",";
							buffer_str << (((double)atoi(landmark_node->first_attribute(XML_ATTRIBUTE_Z_POSITION_STR)->value())) / 100);
							if(landmark_node->first_attribute(XML_ATTRIBUTE_VARIANCE_MIN_X_STR)){
								buffer_str << "," << (((double)atoi(landmark_node->first_attribute(XML_ATTRIBUTE_VARIANCE_MIN_X_STR)->value())) / 100) << ",";
							}
							if(landmark_node->first_attribute(XML_ATTRIBUTE_VARIANCE_MIN_X_STR)){
								buffer_str << (((double)atoi(landmark_node->first_attribute(XML_ATTRIBUTE_VARIANCE_MAX_X_STR)->value())) / 100) << ",";
							}
							if(landmark_node->first_attribute(XML_ATTRIBUTE_VARIANCE_MIN_X_STR)){
								buffer_str << (((double)atoi(landmark_node->first_attribute(XML_ATTRIBUTE_VARIANCE_MIN_Y_STR)->value())) / 100) << ",";
							}
							if(landmark_node->first_attribute(XML_ATTRIBUTE_VARIANCE_MIN_X_STR)){
								buffer_str << (((double)atoi(landmark_node->first_attribute(XML_ATTRIBUTE_VARIANCE_MAX_Y_STR)->value())) / 100);
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
							buffer_str << (((double)atoi(feature_node->first_attribute(XML_ATTRIBUTE_X_POSITION_STR)->value())) / 100) << ",";
							buffer_str << (((double)atoi(feature_node->first_attribute(XML_ATTRIBUTE_Y_POSITION_STR)->value())) / 100) << ",";
							buffer_str << (((double)atoi(feature_node->first_attribute(XML_ATTRIBUTE_WIDTH_STR)->value())) / 100) << ",";
							buffer_str << (((double)atoi(feature_node->first_attribute(XML_ATTRIBUTE_HEIGHT_STR)->value())) / 100);

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
							buffer_str << (((double)atoi(site_node->first_attribute(XML_ATTRIBUTE_X_POSITION_STR)->value())) / 100) << ",";
							buffer_str << (((double)atoi(site_node->first_attribute(XML_ATTRIBUTE_Y_POSITION_STR)->value())) / 100);

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
				        new_sites_node->append_attribute(doc.allocate_attribute(XML_ATTRIBUTE_X_POSITION_STR, data.at(3).c_str()));
				        new_sites_node->append_attribute(doc.allocate_attribute(XML_ATTRIBUTE_Y_POSITION_STR, data.at(4).c_str()));
				        sites_root_node->append_node(new_sites_node);

			        	if(mapId == currentSector->getMapId() and sectorId == currentSector->getId()){
				        	s_site* tempSite = new s_site;

				        	tempSite->id = indexAssigned;
							tempSite->name = data.at(2);
							tempSite->xpos = ((double)atoi(data.at(3).c_str())) / 100;
							tempSite->ypos = ((double)atoi(data.at(4).c_str())) / 100;
							
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
						        where->append_attribute(doc.allocate_attribute(XML_ATTRIBUTE_X_POSITION_STR, data.at(4).c_str()));
						        where->append_attribute(doc.allocate_attribute(XML_ATTRIBUTE_Y_POSITION_STR, data.at(5).c_str()));
						        
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

		if(mapId == currentSector->getMapId() and sectorId == currentSector->getId()){
        	s_site* tempSite = currentSector->findSiteById(siteId);

        	tempSite->name = data.at(3);
			tempSite->xpos = ((double)atoi(data.at(4).c_str())) / 100;
			tempSite->ypos = ((double)atoi(data.at(5).c_str())) / 100;		
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
		   
		if(mapId == currentSector->getMapId() and sectorId == currentSector->getId()){
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

	    if(mapId == currentSector->getMapId() and sectorId == currentSector->getId()){

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

				        features_root_node->append_node(new_feature_node);

				        if(mapId == currentSector->getMapId() and sectorId == currentSector->getId()){
				        	s_feature* tempFeature = new s_feature;

							tempFeature->id = indexAssigned;
							tempFeature->name = data.at(2);
							tempFeature->width = ((double)atoi(data.at(3).c_str())) / 100;
							tempFeature->height = ((double)atoi(data.at(4).c_str())) / 100;
							tempFeature->xpos = ((double)atoi(data.at(5).c_str())) / 100;
							tempFeature->ypos = ((double)atoi(data.at(6).c_str())) / 100;

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

	    if(mapId == currentSector->getMapId() and sectorId == currentSector->getId()){
        	s_feature* tempFeature = currentSector->findFeatureById(featureId);

			tempFeature->name = data.at(3);
			tempFeature->width = ((double)atoi(data.at(4).c_str())) / 100;
			tempFeature->height = ((double)atoi(data.at(5).c_str())) / 100;
			tempFeature->xpos = ((double)atoi(data.at(6).c_str())) / 100;
			tempFeature->ypos = ((double)atoi(data.at(7).c_str())) / 100;
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

	    if(mapId == currentSector->getMapId() and sectorId == currentSector->getId()){
        	s_feature* tempFeature = currentSector->findFeatureById(featureId);
        	currentSector->deleteFeature(tempFeature);
        }

	} else {
		RNUtils::printLn("Unable to delete feature. Invalid number of parameters..");
	}    
}

void GeneralController::moveRobot(double lin_vel, double angular_vel){

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
		if(tourThread){
			tourThread->kill();
		}
		this->moveAtSpeed(lin_vel, angular_vel);
	} else {
		this->stopRobot();
	}
}

void GeneralController::setRobotPosition(double x, double y, double theta){
	if(localization != NULL){
		localization->kill();
	}
	RNUtils::printLn("Changin' position...");
	this->setPosition(x, y, theta);

	if(localization != NULL){
		localization->reset();	
	}
	RNUtils::printLn("new position is: {x: %f, y: %f, \u03d1: %f}", x, y, theta);
}

void GeneralController::setRobotPosition(Matrix Xk){
	this->setRobotPosition(Xk(0, 0), Xk(1, 0), Xk(2, 0));
}

void GeneralController::moveRobotToPosition(double x, double y, double theta){
	pthread_mutex_lock(&currentSectorLocker);
	this->gotoPosition(x, y, theta, (currentSector != NULL ? currentSector->isHallway() : false));
	pthread_mutex_unlock(&currentSectorLocker);
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

	for(int i = 0; i < MAX_CLIENTS; i++){
		if(isConnected(i)){
			if(isWebSocket(i)){
				spdWSServer->sendMsg(i, 0x00, bump, dataLen);
			} else {
				if(this->getClientUDP(i) != NULL){
					this->getClientUDP(i)->sendData((unsigned char*)bump, dataLen);	
				}
				
			}
		}
	}
	delete[] bump;
}

void GeneralController::onPositionUpdate(double x, double y, double theta, double transSpeed, double rotSpeed){

	
	char* bump = new char[256];
	sprintf(bump, "$POSE_VEL|%.4f,%.4f,%.4f,%.4f,%.4f", x, y, theta, transSpeed, rotSpeed);
	int dataLen = strlen(bump);

	for(int i = 0; i < MAX_CLIENTS; i++){
		if(isConnected(i)){
			if(isWebSocket(i)){
				spdWSServer->sendMsg(i, 0x00, bump, dataLen);
			} else {
				if(this->getClientUDP(i) != NULL){
					this->getClientUDP(i)->sendData((unsigned char*)bump, dataLen);
				}
			}
		}
	}
	delete[] bump;
}

void GeneralController::onSonarsDataUpdate(std::vector<PointXY*>* data){

}

void GeneralController::onBatteryChargeStateChanged(char battery){
	if(!setChargerPosition && ((int)battery) > 0){
		pthread_mutex_lock(&currentSectorLocker);
		if(currentSector != NULL){
			
			for(int i = 0; i< currentSector->featuresSize(); i++){
				if(currentSector->featureAt(i)->name == SEMANTIC_FEATURE_CHARGER_STR){
					setChargerPosition = true;
					double xpos = currentSector->featureAt(i)->xpos;
					double ypos = currentSector->featureAt(i)->ypos;
					RNUtils::sleep(100);
					setRobotPosition(xpos, ypos, M_PI/2);
				}
			}
		}
		pthread_mutex_unlock(&currentSectorLocker);
	}
}

void GeneralController::onSecurityDistanceWarningSignal(){
	RNUtils::printLn("Alert on something is blocking Doris to achieve goal...");
}

void GeneralController::onSecurityDistanceStopSignal(){
	pthread_mutex_lock(&currentSectorLocker);
	if(currentSector != NULL){
		std::vector<std::string> spltdSequence = RNUtils::split((char*)currentSector->getSequence().c_str(), ",");
		if(lastSiteVisitedIndex > 0 and (lastSiteVisitedIndex < spltdSequence.size() - 1)){
			int goalId = atoi(spltdSequence.at(lastSiteVisitedIndex + 1).c_str());
			RNUtils::printLn("Doris Stopped. Could not achieve next goal from current position...");
		}
	}
	pthread_mutex_unlock(&currentSectorLocker);
}

void GeneralController::getInitialLocation(){
	lockVisualLandmarks();
	if(visualLandmarks){
		std::map<int, int> maps;
		std::map<int, int> sectors;
		for(int i = 0; i < visualLandmarks->size(); i++){
			std::map<int, int>::iterator it;
			if((it = maps.find(visualLandmarks->at(i)->getMapId())) != maps.end()){
				maps.insert(std::pair<int, int>(visualLandmarks->at(i)->getMapId(), it->second + 1));
			} else {
				maps.insert(std::pair<int, int>(visualLandmarks->at(i)->getMapId(), 1));
			}

			if((it = sectors.find(visualLandmarks->at(i)->getSectorId())) != sectors.end()){
				sectors.insert(std::pair<int, int>(visualLandmarks->at(i)->getSectorId(), it->second + 1));
			} else {
				sectors.insert(std::pair<int, int>(visualLandmarks->at(i)->getSectorId(), 1));
			}
		}
		std::map<int, int>::iterator it;
		int mapH = RN_NONE, sectorH = RN_NONE, map = RN_NONE, sector = RN_NONE;
		for(it = maps.begin(); it != maps.end(); it++){
			if(mapH < it->second){
				mapH = it->second;
				map = it->first;
			}
		}

		for(it = sectors.begin(); it != sectors.end(); it++){
			if(sectorH < it->second){
				sectorH = it->second;
				sector = it->first;
			}
		}
		
		if(map != RN_NONE and sector != RN_NONE){
			if(loadSector(map, sector)){
				std::cout << "-------> N Markers: " << visualLandmarks->size() << std::endl;
				if(visualLandmarks->size() >= 2){
					double minDist = std::numeric_limits<double>::max();

					//Checks pairs of markers and saves the pair that's closest to Doris
					
					int maIdx = RN_NONE, mbIdx = RN_NONE;
					for(int i = 0; i < visualLandmarks->size(); i++){
						for(int j = i + 1; j < visualLandmarks->size(); j++){
							double combinedDistance = visualLandmarks->at(i)->getPointsXMean() + visualLandmarks->at(j)->getPointsXMean();
							if(minDist > combinedDistance){
								minDist = combinedDistance;
								maIdx = i;
								mbIdx = j;
							}
						}
					}
					double r0 = visualLandmarks->at(maIdx)->getPointsXMean();
					double r1 = visualLandmarks->at(mbIdx)->getPointsXMean();

					s_landmark* cma = currentSector->landmarkByTypeAndId(XML_SENSOR_TYPE_CAMERA_STR, visualLandmarks->at(maIdx)->getMarkerId());
					s_landmark* cmb = currentSector->landmarkByTypeAndId(XML_SENSOR_TYPE_CAMERA_STR, visualLandmarks->at(mbIdx)->getMarkerId());

					PointXYZ possiblePosition0;
					PointXYZ possiblePosition1;
					double d = RNUtils::distanceTo(cma->xpos, cma->ypos, cmb->xpos, cmb->ypos);
					if(d <= r0 + r1 and d >= std::abs(r0 - r1)){
						double a = (pow(r0, 2) - pow(r1, 2) + pow(d, 2)) / (2 * d);
						double h = sqrt(pow(r0, 2) - pow(a, 2));
						PointXY middlePoint(cma->xpos + a * (cmb->xpos - cma->xpos) / d, cma->ypos + a * (cmb->ypos - cma->ypos) / d);
						/*double x2 = x0 + a * (x1 - x0) / d;
						double y2 = y0 + a * (y1 - y0) / d;*/
					
						possiblePosition0.setX (middlePoint.getX() + h * (cmb->ypos - cma->ypos) / d);
						possiblePosition0.setY (middlePoint.getY() - h * (cmb->xpos - cma->xpos) / d);
						possiblePosition1.setX (middlePoint.getX() - h * (cmb->ypos - cma->ypos) / d);
						possiblePosition1.setY (middlePoint.getY() + h * (cmb->xpos - cma->xpos) / d);

						possiblePosition0.setZ(std::atan2((possiblePosition0.getY() - cma->ypos), (possiblePosition0.getX() - cma->xpos)) - visualLandmarks->at(maIdx)->getPointsYMean() - M_PI);
						if(possiblePosition0.getZ() > M_PI){
							possiblePosition0.setZ(possiblePosition0.getZ() - 2 * M_PI);
						} else if(possiblePosition0.getZ() < -M_PI){
							possiblePosition0.setZ(possiblePosition0.getZ() + 2 * M_PI);
						}

						possiblePosition1.setZ(std::atan2((possiblePosition1.getY() - cma->ypos ), (possiblePosition1.getX() - cma->xpos)) - visualLandmarks->at(maIdx)->getPointsYMean() - M_PI);
						if(possiblePosition1.getZ() > M_PI){
							possiblePosition1.setZ(possiblePosition1.getZ() - 2 * M_PI);
						} else if(possiblePosition1.getZ() < -M_PI){
							possiblePosition1.setZ(possiblePosition1.getZ() + 2 * M_PI);
						}
						std::cout << "P1: " << possiblePosition0.toString() << std:: endl;
						std::cout << "P2: " << possiblePosition1.toString() << std:: endl;

						bool s1 = currentSector->checkPointXYInPolygon(possiblePosition0);
						bool s2 = currentSector->checkPointXYInPolygon(possiblePosition1);
						if(s1 and s2){
							if (visualLandmarks->size() >= 3){
								double thirdMinRadius = std::numeric_limits<double>::max();
								int thirdIndex = 0;

								for(int i = 0; i < visualLandmarks->size(); i++){
									if(thirdMinRadius > visualLandmarks->at(i)->getPointsXMean() and i != maIdx and i != mbIdx){
										thirdMinRadius = visualLandmarks->at(i)->getPointsXMean();
										thirdIndex = i;
									}
								}
								s_landmark* cmc = currentSector->landmarkByTypeAndId(XML_SENSOR_TYPE_CAMERA_STR, visualLandmarks->at(thirdIndex)->getMarkerId());
								double difference0 = std::abs(visualLandmarks->at(thirdIndex)->getPointsXMean() - sqrt(pow(possiblePosition0.getX() - cmc->xpos, 2) + pow(possiblePosition0.getY() - cmc->ypos, 2)));
								double difference1 = std::abs(visualLandmarks->at(thirdIndex)->getPointsXMean() - sqrt(pow(possiblePosition1.getX() - cmc->xpos, 2) + pow(possiblePosition1.getY() - cmc->ypos, 2)));

								if (difference0 < difference1){
									setRobotPosition(possiblePosition0.getX(), possiblePosition0.getY(), possiblePosition0.getZ());
								} else {
									setRobotPosition(possiblePosition1.getX(), possiblePosition1.getY(), possiblePosition1.getZ());
								}
							}
						} else if(s1){
							setRobotPosition(possiblePosition0.getX(), possiblePosition0.getY(), possiblePosition0.getZ());
						} else if(s2){
							setRobotPosition(possiblePosition1.getX(), possiblePosition1.getY(), possiblePosition1.getZ());
						}
					}
				}
			}
		}

	}
	unlockVisualLandmarks();
}

int GeneralController::initializeKalmanVariables(){
	int result = RN_NONE;
	bool currentSectorIsNull = true;

	pthread_mutex_lock(&currentSectorLocker);
	currentSectorIsNull = currentSector == NULL;
	pthread_mutex_unlock(&currentSectorLocker);

	if(currentSectorIsNull){
		getInitialLocation();
	}

	pthread_mutex_lock(&currentSectorLocker);
	currentSectorIsNull = currentSector == NULL;
	pthread_mutex_unlock(&currentSectorLocker);

	if(not currentSectorIsNull){
		double uX, uY, uTh;
		Matrix rpk = getRawEncoderPosition();
		rpk.print();
		
		pthread_mutex_lock(&currentSectorLocker);
		int laserLandmarksCount = currentSector->landmarksSizeByType(XML_SENSOR_TYPE_LASER_STR);
		int cameraLandmarksCount = currentSector->landmarksSizeByType(XML_SENSOR_TYPE_CAMERA_STR);
		int rfidLandmarksCount = currentSector->landmarksSizeByType(XML_SENSOR_TYPE_RFID_STR);
		pthread_mutex_unlock(&currentSectorLocker);

		int totalLandmarks = 0;

		uX = robotConfig->navParams->initialPosition->xDistribution->variance;
		uY = robotConfig->navParams->initialPosition->yDistribution->variance;
		uTh = robotConfig->navParams->initialPosition->thDistribution->variance;
		RNUtils::printLn("Position {ux: %f m^2, uy: %f m^2, uth:%f rads^2}", uX, uY, uTh);

		P(0, 0) = uX; 	P(0, 1) = 0; 	P(0, 2) = 0;
		P(1, 0) = 0; 	P(1, 1) = uY;	P(1, 2) = 0;
		P(2, 0) = 0;	P(2, 1) = 0;	P(2, 2) = uTh;
		
		// Variances and Covariances Matrix of Process noise Q

		uX = robotConfig->navParams->processNoise->dDistribution->variance;
		uTh = robotConfig->navParams->processNoise->thDistribution->variance;
		
		Q(0, 0) = uX; 	Q(0, 1) = 0;
		Q(1, 0) = 0; 	Q(1, 1) = uTh;
		RNUtils::printLn("Process {ux: %f m^2, uth: %f rads^2}", uX, uTh);


		for(int i = 0; i < robotConfig->navParams->sensors->size(); i++){
			double laserDistanceBias, laserAngleBias, cameraDistanceBias, cameraAngleBias;
			s_distribution* dDistribution;
			s_distribution* thDistribution;
			dDistribution = robotConfig->navParams->sensors->at(i)->observationNoise->dDistribution;
			thDistribution = robotConfig->navParams->sensors->at(i)->observationNoise->thDistribution;

			if(robotConfig->navParams->sensors->at(i)->type == XML_SENSOR_TYPE_LASER_STR){
				this->laserDistanceAlpha = dDistribution->alpha;
				this->laserAngleAlpha = thDistribution->alpha;
				this->laserDistanceVariance = dDistribution->variance;
				this->laserAngleVariance = thDistribution->variance;
				this->laserDistanceBias = dDistribution->bias;
				this->laserAngleBias = thDistribution->bias;
				RNUtils::printLn("Laser {ux: %f m^2, uth:%f rads^2}", this->laserDistanceVariance, this->laserAngleVariance);

			} else if(robotConfig->navParams->sensors->at(i)->type == XML_SENSOR_TYPE_CAMERA_STR){
				this->cameraDistanceAlpha = dDistribution->alpha;
				this->cameraAngleAlpha = thDistribution->alpha;
				this->cameraDistanceVariance = dDistribution->variance;;
				this->cameraAngleVariance = thDistribution->variance;
				this->cameraDistanceBias = dDistribution->bias;
				this->cameraAngleBias = thDistribution->bias;
				RNUtils::printLn("Camara {ux: %f m^2, uth:%f rads^2}", this->cameraDistanceVariance, this->cameraAngleVariance);
				//cameraUTh = 0.00022;   //forced meanwhile

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


		// Values of the bias vector b

		double bD, bTh, bDLaser, bThLaser, bDCamera, bThCamera;
		bD = robotConfig->navParams->processNoise->dDistribution->bias;
		bTh = robotConfig->navParams->processNoise->thDistribution->bias;
		bDLaser = this->laserDistanceBias;
		bThLaser = this->laserAngleBias; 
		bDCamera = this->cameraDistanceBias;
		bThCamera = this->cameraAngleBias;

		b(0, 0) = bD;
		b(1, 0) = bTh;
		b(2, 0) = bDLaser;
		b(3, 0) = bThLaser;
		b(4, 0) = bThCamera;
		b.print();
		// Variances and Covariances Matrix of the bias B

		B(0, 0) = Q(0,0); 	B(0, 1) = Q(0,1);	B(0, 2) = 0;							B(0, 3) = 0;							B(0, 4) = 0;
		B(1, 0) = Q(1,0); 	B(1, 1) = Q(1,1);	B(1, 2) = 0;							B(1, 3) = 0;							B(1, 4) = 0;
		B(2, 0) = 0; 		B(2, 1) = 0;		B(2, 2) = this->laserDistanceVariance;	B(2, 3) = 0;							B(2, 4) = 0;
		B(3, 0) = 0; 		B(3, 1) = 0;		B(3, 2) = 0;							B(3, 3) = this->laserAngleVariance;		B(3, 4) = 0;
		B(4, 0) = 0; 		B(4, 1) = 0;		B(4, 2) = 0;							B(4, 3) = 0;							B(4, 4) = this->cameraAngleVariance;
		B.print();
		
		result = 0;
	}
	
	return result;
}

void GeneralController::setTextInputIdToEmotion(std::string textInputId){
	this->textInputId = textInputId;
	if(emotions){
		emotions->setSpokenImpulseId(std::stoi(textInputId));
	}
}

void GeneralController::setEmotionsResult(std::string emotionState, std::string faceIdFromEmotions){
	this->emotionState = emotionState;
	this->faceIdFromEmotions = faceIdFromEmotions;

	if(dialogs and not this->emotionState.empty()){
		dialogs->setState(this->emotionState);
	}
	if(this->gestures and not this->faceIdFromEmotions.empty()){
		this->gestures->setGesture(this->faceIdFromEmotions);
	}
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

Matrix GeneralController::getb(){
	return b;
}

Matrix GeneralController::getB(){
	return B;
}

double GeneralController::getLaserDistanceAlpha(){
	return laserDistanceAlpha;
}

double GeneralController::getLaserAngleAlpha(){
	return laserAngleAlpha;
}

double GeneralController::getCameraDistanceAlpha(){
	return cameraDistanceAlpha;
}

double GeneralController::getCameraAngleAlpha(){
	return cameraAngleAlpha;
}

double GeneralController::getLaserDistanceVariance(){
	return laserDistanceVariance;
}

double GeneralController::getLaserAngleVariance(){
	return laserAngleVariance;
}

double GeneralController::getCameraDistanceVariance(){
	return cameraDistanceVariance;
}

double GeneralController::getCameraAngleVariance(){
	return cameraAngleVariance;
}

MapSector* GeneralController::getCurrentSector(){
	MapSector* copy = NULL;
	pthread_mutex_lock(&currentSectorLocker);
	if(currentSector){
		copy = currentSector->clone();
	}
	pthread_mutex_unlock(&currentSectorLocker);
	return copy;
}

void GeneralController::addSectorChangedCallback(RNFunPointer* func){
	sectorChangedSubscribers.emplace_back(func);
}

void GeneralController::remSectorChangedCallback(RNFunPointer* func){
	sectorChangedSubscribers.remove(func);
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

void GeneralController::setLaserLandmarks(RNLandmarkList* landmarks){
	lockLaserLandmarks();
	this->laserLandmarks = landmarks;
	unlockLaserLandmarks();
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

RNRFIdentificationTask* GeneralController::getRfidTask(){
	return rfidTask;
}

RNEmotionsTask* GeneralController::getEmotionsTask(){
	return emotions;
}

RNLaserTask* GeneralController::getLaserTask(){
	return laserTask;
}

PointXY GeneralController::getNextSectorCoord(){
	return this->nextSectorCoord;
}

bool GeneralController::isFirstQuadrant(double angle){
	return (angle >= 0 && angle <= (M_PI /2));
}

bool GeneralController::isSecondQuadrant(double angle){
	return (angle > (M_PI/2) && angle <= (M_PI));
}

bool GeneralController::isThirdQuadrant(double angle){
	return (angle <= (-M_PI/2) && angle > (-M_PI));
}

bool GeneralController::isFouthQuadrant(double angle){
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

double GeneralController::getRobotHeight(){
	return (robotConfig != NULL ? robotConfig->height : 0.0);
}

int GeneralController::getFaceId(){
	return (robotConfig != NULL ? robotConfig->faceId : 0.0);
}

int GeneralController::getNeckId(){
	return (robotConfig != NULL ? robotConfig->neckId : 0.0);
}

int GeneralController::getRfidTxPower(){
	return (robotConfig != NULL ? robotConfig->rfidConfig->txPowerIndex : 8); //default value
}

int GeneralController::getRfidRxSensitivity(){
	return (robotConfig != NULL ? robotConfig->rfidConfig->rxSensitivityIndex : 22); //default value
}

DorisLipSync* GeneralController::getTTS(){
	return tts;
}

SerialPort* GeneralController::getMaestroController(){
	return this->maestroControllers;
}

void GeneralController::onLaserScanCompleted(LaserScan* data){
	RobotNode::onLaserScanCompleted(data);
	std::ostringstream buffer_str;
	buffer_str.clear();
	buffer_str << "$LASER|";
	for(int i = 0; i < data->size(); i++){
		buffer_str << data->getRanges()->at(i);
		if(i < data->size() - 1){
			buffer_str << ",";
		}
	}
	for(int i = 0; i < MAX_CLIENTS; i++){
		if(isConnected(i)){
			if(isWebSocket(i)){
				//spdWSServer->sendMsg(i, 0x00, buffer_str.str().c_str(), buffer_str.str().length());
			} else {
				if(this->getClientUDP(i) != NULL){
					this->getClientUDP(i)->sendData((unsigned char*)buffer_str.str().c_str(), buffer_str.str().length());
				}
			}
		}
	}
}

void GeneralController::onSensorsScanCompleted(){
	std::ostringstream buffer_str;
	int mapId = RN_NONE;
	int sectorId = RN_NONE;
	pthread_mutex_lock(&currentSectorLocker);
	if(currentSector != NULL){
		mapId = currentSector->getMapId();
		sectorId = currentSector->getId();
	}
	pthread_mutex_unlock(&currentSectorLocker);
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

	for(int i = 0; i < MAX_CLIENTS; i++){
		if(isConnected(i)){
			if(isWebSocket(i)){
				spdWSServer->sendMsg(i, 0x00, buffer_str.str().c_str(), buffer_str.str().length());
			} else {
				if(this->getClientUDP(i) != NULL){
					this->getClientUDP(i)->sendData((unsigned char*)buffer_str.str().c_str(), buffer_str.str().length());
				}
			}
		}
	}
}

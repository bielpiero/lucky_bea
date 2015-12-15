#include <stdlib.h>
#include <stdio.h>
#include "GeneralController.h"


const float GeneralController::LASER_MAX_RANGE = 11.6;

GeneralController::GeneralController(ros::NodeHandle nh_):RobotNode("/dev/ttyS0"){
	this->nh = nh_;
	
	this->maestroControllers = new SerialPort();
	this->tts = new TextToSpeech();
	this->tts->setDefaultConfiguration();
	this->tts->setString("Hola, me llamo Doris.");

	this->frontBumpersOk = true;
	this->rearBumpersOk = true;
	this->setChargerPosition = false;
	this->hasAchievedGoal = false;
	this->streamingActive = NO;
	this->keepRobotTracking = NO;
	this->udpPort = 0;
	this->spdUDPPort = 0;
	
	kalmanFuzzy = new std::vector<fuzzy::trapezoid*>();
	
	robotState = Matrix(3, 1);
	robotEncoderPosition = Matrix(3, 1);
	robotVelocity = Matrix (2, 1);

	robotRawEncoderPosition = Matrix(3, 1);
	robotRawDeltaPosition = Matrix (2, 1);
	
	P = Matrix(3, 3);
	Q = Matrix(2, 2);
	R = Matrix(3, 3);
	spdUDPClient = NULL;

	xmlFaceFullPath = ros::package::getPath(PACKAGE_NAME) + XML_FILE_PATH;
	xmlSectorsFullPath = ros::package::getPath(PACKAGE_NAME) + XML_FILE_SECTORS_PATH;
	xmlRobotConfigFullPath = ros::package::getPath(PACKAGE_NAME) + XML_FILE_ROBOT_CONFIG_PATH;

	navSectors = NULL;
	currentSector = NULL;
	loadRobotConfig();
	loadSectors();
	loadSector(0);
	pthread_mutex_init(&mutexLandmarkLocker, NULL);
}


GeneralController::~GeneralController(void){
	disconnect();
	stopDynamicGesture();
	stopVideoStreaming();
	stopRobotTracking();
	stopCurrentTour();
	pthread_mutex_destroy(&mutexLandmarkLocker);
}

void GeneralController::OnConnection()//callback for client and server
{
	if(IsConnected()) {
		std::cout << "Client "<< this->getClientIPAddress() << " is Connected to Doris, using port " << this->getClientPort() << std::endl;	
	} else {
		std::cout << "Disconnected..." << std::endl;
		
		if(spdUDPClient != NULL){
			spdUDPClient->closeConnection();
			spdUDPClient = NULL;
		}
		stopDynamicGesture();
		stopVideoStreaming();
		stopCurrentTour();
	}
}
void GeneralController::OnMsg(char* cad,int length){//callback for client and server
	cad[length] = 0;
	unsigned char function = *(cad++);
	std::string local_buffer_out = "";
	std::string gestures = "";
	std::string servo_positions = "";
	std::string mapsAvailable = "";
	std::string mapInformation = "";
	std::ostringstream number_converter;
	
	unsigned char port = 0;
	unsigned char card_id = 0;

	int servo_position = 0;
	int face_id, k=0;
	int mapId = 0;
	float lin_vel = 0, ang_vel = 0;
	int cameraCount = 0;
	int videoDevice = 0;
	float x, y, theta;
	
	switch (function){
		case 0x00:
			std::cout << "Command 0x00. Static Faces Requested" << std::endl;
			getGestures("0", gestures);
			SendMsg(0x00, (char*)gestures.c_str(), (int)(gestures.length())); 
			break;
		case 0x01:
			std::cout << "Command 0x01. Dynamic Faces Requested" << std::endl;
			getGestures("1", gestures);
			SendMsg(0x01, (char*)gestures.c_str(), (int)(gestures.length()));
			break;
		case 0x02:
			std::cout << "Command 0x02. Saving New Static Face" << std::endl;
			saveGesture(cad, 0);
			break;
		case 0x03:
			std::cout << "Command 0x03. Saving New Dynamic Face" << std::endl;
			saveGesture(cad, 1);
			break;
		case 0x04:
			std::cout << "Command 0x04. Modifying Static Face" << std::endl;
			modifyGesture(cad, 0);
			break;
		case 0x05:
			std::cout << "Command 0x05. Modifying Dynamic Face" << std::endl;
			modifyGesture(cad, 1);
			break;
		case 0x06:
			std::cout << "Command 0x06. Removing Face" << std::endl;
			removeGesture(cad);
			break;
		case 0x07:
			std::cout << "Command 0x07. Setting Gesture Id: " << cad << std::endl;
			setGesture(cad);
			break;
		case 0x08:
            getPololuInstruction(cad, card_id, port, servo_position);            
			//std::cout << "Command 0x08. Moving from CardId: " << (int)card_id << " Servo: " << (int)port << " To Position: " << servo_position << std::endl;
			setServoPosition(card_id, port, servo_position);
			break;
		case 0x09:
			std::cout << "Command 0x09. Sending current positions" << std::endl;
			//SendServoPositions(servo_positions);
			//SendMsg(0x09, (char*)servo_positions.c_str(), (int)(servo_positions.length()));
			break;
		case 0x0A:
			std::cout << "Command 0x0A. Stopping any Dynamic Face" << std::endl;
			stopDynamicGesture();
			break;
		case 0x10:
			getVelocities(cad, lin_vel, ang_vel);
			moveRobot(lin_vel, ang_vel);
			break;
		case 0x11:
			trackRobot();
			//startSitesTour();
			break;
		case 0x12:
			stopRobotTracking();
			break;	
		case 0x13:
			getPositions(cad, x, y, theta);
			setRobotPosition(x, y, theta);
			break;
		case 0x14:
			getPositions(cad, x, y, theta);
			moveRobotToPosition(x, y, theta);
			break;
		case 0x15:
			getMapsAvailable(mapsAvailable);
			SendMsg(0x15, (char*)mapsAvailable.c_str(), (int)mapsAvailable.length()); 
			break;
		case 0x16:
			getMapId(cad, mapId);
			loadSector(mapId);
			break;
		case 0x17:
			getMapInformationLandmarks(mapInformation);
			SendMsg(0x17, (char*)mapInformation.c_str(), (int)mapInformation.length()); 
			break;
		case 0x18:
			getMapInformationFeatures(mapInformation);
			SendMsg(0x18, (char*)mapInformation.c_str(), (int)mapInformation.length()); 
			break;
		case 0x19:
			getMapInformationSites(mapInformation);
			SendMsg(0x19, (char*)mapInformation.c_str(), (int)mapInformation.length()); 
			break;
		case 0x30:
			getCameraDevicePort(cad, videoDevice, udpPort);
			beginVideoStreaming(videoDevice);
			break;
		case 0x31:
			stopVideoStreaming();
			break;
		case 0xFF:
			initializeSPDPort(cad);
			break;
		default:
			std::cout << "Command Not Recognized.." << std::endl;
			break;
			
	}
	
}

void GeneralController::initializeSPDPort(char* cad){
	this->spdUDPPort = atoi(cad);
	if(this->spdUDPPort <= 0){
		throw std::invalid_argument("Error!!! Could not initialize SPD streaming");
	}
	spdUDPClient = new UDPClient(this->getClientIPAddress(), this->spdUDPPort);
}

void GeneralController::getMapId(char* cad, int& mapId){
	mapId = atoi(cad);
}

void GeneralController::getPositions(char* cad, float& x, float& y, float& theta){
	char* current_number;
	float values[6];
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
}

void GeneralController::getCameraDevicePort(char* cad, int& device, int& port){
	char* current_number;
	int values[6];
	int index = 0;
	current_number = std::strtok(cad, ":");
	//
	while(current_number != NULL){
		values[index++] = std::atoi(current_number);
		current_number = std::strtok(NULL, ":");
	}
	device = values[0];
	port = values[1];
}

void GeneralController::getPololuInstruction(char* cad, unsigned char& card_id, unsigned char& servo_id, int& value){
	char* current_number;
	int values[6];
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
}

void GeneralController::getVelocities(char* cad, float& lin_vel, float& ang_vel){
	char* current_number;
	float values[6];
	int index = 0;
	current_number = std::strtok(cad, ",");
	
	while(current_number != NULL){
		int cValue = std::atoi(current_number);
		values[index++] = (float)(((float)cValue)/1000.0);
		current_number = std::strtok(NULL, ",");
	}
	lin_vel = values[0];
	ang_vel = values[1];
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
	while (positions != 0)
	{
            std::ostringstream convert;
            convert << i;
                motor_positions[i].idMotor = convert.str();
		motor_positions[i++].pos = std::string(positions);
		positions = strtok(NULL, ",");
	}
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
	if(root_node != NULL)
	{
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
	if(root_node != NULL)
	{
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

void GeneralController::setGesture(std::string face_id){
    xml_document<> doc;
    xml_node<>* root_node;
	
	std::string buffer_str = "";
	
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
			
			if (face_id == id)
			{
				for(xml_node<> * motor_node = gesto_node->first_node(XML_ELEMENT_MOTOR_STR); motor_node; motor_node = motor_node->next_sibling()){
					unsigned char card_id = (unsigned char)atoi(motor_node->first_attribute(XML_ATTRIBUTE_CARD_ID_STR)->value());
					unsigned char servo_id = (unsigned char)atoi(motor_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value());
					int position = atoi(motor_node->first_attribute(XML_ATTRIBUTE_POSITION_STR)->value());
					int speed = atoi(motor_node->first_attribute(XML_ATTRIBUTE_SPEED_STR)->value());
					int acceleration = atoi(motor_node->first_attribute(XML_ATTRIBUTE_ACCELERATION_STR)->value());

					setServoPosition(card_id, servo_id, position);
					//setServoSpeed(card_id, servo_id, speed);
					//setServoAcceleration(card_id, servo_id, acceleration);
				}	
					
				if(tipo == "1"){
					std::cout << "This is a dynamic face" << std::endl;
					dynamic_face_info *data = new dynamic_face_info;
					data->object = this;
					data->id_gesto = face_id;
					
					pthread_t t1;
					continue_dynamic_thread = true;
					
					pthread_create(&t1,NULL, GeneralController::dynamicFaceThread, (void *)(data));
				}
			}
		}
	}
	the_file.close();
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

	usleep(1000*atoi(selected_dynamic_face.ts.c_str()));

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
			
			usleep(1000*atoi(selected_dynamic_face.secuences[i].tsec.c_str()));
		}
	}
}

void GeneralController::loadSector(int sectorId){
	bool found = false;
	for(int i = 0; i < navSectors->size() and not found; i++){
		if(navSectors->at(i)->id == sectorId){
			found = true;
			currentSector = navSectors->at(i);
		}
	}
}

void GeneralController::loadSectors(){
	xml_document<> doc;
    xml_node<>* root_node;
	
	std::string buffer_str = "";
	
    std::ifstream the_file(xmlSectorsFullPath.c_str());
	
    std::vector<char> buffer((std::istreambuf_iterator<char>(the_file)), std::istreambuf_iterator<char>());
	buffer.push_back('\0');

	doc.parse<0>(&buffer[0]);
	root_node = doc.first_node(XML_ELEMENT_SECTORS_STR);

	if(navSectors != NULL){
		for(int i = 0; i < navSectors->size(); i++){
			delete navSectors->at(i)->landmarks;
			delete navSectors->at(i)->features;
			delete navSectors->at(i)->sites;
		}
		delete navSectors;
	}
	navSectors = new std::vector<s_sector*>();
	
	if(root_node != NULL){
		for (xml_node<> * sector_node = root_node->first_node(XML_ELEMENT_SECTOR_STR); sector_node; sector_node = sector_node->next_sibling()){	
			s_sector* navSector = new s_sector;
			navSector->sitesCyclic = false;
			int xmlSectorId = atoi(sector_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value());

			navSector->id = xmlSectorId;
			navSector->name = std::string(sector_node->first_attribute(XML_ATTRIBUTE_NAME_STR)->value());
			navSector->width = atoi(sector_node->first_attribute(XML_ATTRIBUTE_WIDTH_STR)->value());
			navSector->height = atoi(sector_node->first_attribute(XML_ATTRIBUTE_HEIGHT_STR)->value());

			navSector->landmarks = new std::vector<s_landmark*>();
			navSector->features = new std::vector<s_feature*>();
			navSector->sites = new std::vector<s_site*>();

			xml_node<>* landmarks_root_node = sector_node->first_node(XML_ELEMENT_LANDMARKS_STR);
			for(xml_node<>* landmark_node = landmarks_root_node->first_node(XML_ELEMENT_LANDMARK_STR); landmark_node; landmark_node = landmark_node->next_sibling()){
				s_landmark* tempLandmark = new s_landmark;

				tempLandmark->id = atoi(landmark_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value());
				tempLandmark->var = ((float)atoi(landmark_node->first_attribute(XML_ATTRIBUTE_VARIANCE_STR)->value())) / 100;
				tempLandmark->xpos = ((float)atoi(landmark_node->first_attribute(XML_ATTRIBUTE_X_POSITION_STR)->value())) / 100;
				tempLandmark->ypos = ((float)atoi(landmark_node->first_attribute(XML_ATTRIBUTE_Y_POSITION_STR)->value())) / 100;

				navSector->landmarks->push_back(tempLandmark);
			}

			xml_node<>* features_root_node = sector_node->first_node(XML_ELEMENT_FEATURES_STR);
			for(xml_node<>* features_node = features_root_node->first_node(XML_ELEMENT_FEATURE_STR); features_node; features_node = features_node->next_sibling()){
				s_feature* tempFeature = new s_feature;

				tempFeature->id = atoi(features_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value());
				tempFeature->name = std::string(features_node->first_attribute(XML_ATTRIBUTE_NAME_STR)->value());
				tempFeature->var = ((float)atoi(features_node->first_attribute(XML_ATTRIBUTE_VARIANCE_STR)->value())) / 100;
				tempFeature->xpos = ((float)atoi(features_node->first_attribute(XML_ATTRIBUTE_X_POSITION_STR)->value())) / 100;
				tempFeature->ypos = ((float)atoi(features_node->first_attribute(XML_ATTRIBUTE_Y_POSITION_STR)->value())) / 100;

				navSector->features->push_back(tempFeature);
				
			}

			xml_node<>* sites_root_node = sector_node->first_node(XML_ELEMENT_SITES_STR);
			std::string cyclic(sites_root_node->first_attribute(XML_ATTRIBUTE_CYCLIC_STR)->value());
			if(cyclic == "yes"){
				navSector->sitesCyclic = true;
			}
			for(xml_node<>* site_node = sites_root_node->first_node(XML_ELEMENT_SITE_STR); site_node; site_node = site_node->next_sibling()){
				s_site* tempSite = new s_site;

				tempSite->id = atoi(site_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value());
				tempSite->name = std::string(site_node->first_attribute(XML_ATTRIBUTE_NAME_STR)->value());
				tempSite->var = ((float)atoi(site_node->first_attribute(XML_ATTRIBUTE_VARIANCE_STR)->value())) / 100;
				tempSite->tsec = ((float)atoi(site_node->first_attribute(XML_ATTRIBUTE_TIME_STR)->value()));
				tempSite->xpos = ((float)atoi(site_node->first_attribute(XML_ATTRIBUTE_X_POSITION_STR)->value())) / 100;
				tempSite->ypos = ((float)atoi(site_node->first_attribute(XML_ATTRIBUTE_Y_POSITION_STR)->value())) / 100;

				navSector->sites->push_back(tempSite);
			}
			navSectors->push_back(navSector);
		}
	}
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
		delete robotConfig->navParams->observationNoise;
		delete robotConfig->navParams->processNoise;
		delete robotConfig->navParams->initialPosition;
		delete robotConfig->navParams;
		delete robotConfig;
	}
	robotConfig = new s_robot;
	robotConfig->navParams = new s_navigation_params;
	robotConfig->navParams->initialPosition = new s_position;
	robotConfig->navParams->processNoise = new s_obs_dth;
	robotConfig->navParams->observationNoise = new s_obs_dth;

	xml_node<>* root_node = doc.first_node(XML_ELEMENT_ROBOT_STR);
	xml_node<>* nav_root_node = root_node->first_node(XML_ELEMENT_NAV_PARAMS_STR);
	robotConfig->navParams->alpha = (float)atoi(nav_root_node->first_attribute(XML_ATTRIBUTE_ALPHA_STR)->value()) / 100;

	xml_node<>* initial_pos_root_node = nav_root_node->first_node(XML_ELEMENT_INITIAL_POS_STR);
	xml_node<>* pos_root_node = initial_pos_root_node->first_node(XML_ELEMENT_POS_X_ZONE_STR);
	s_trapezoid* x_zone = new s_trapezoid;
	x_zone->x1 = (float)atoi(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X1_STR)->value()) / 100;
	x_zone->x2 = (float)atoi(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X2_STR)->value()) / 100;
	x_zone->x3 = (float)atoi(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X3_STR)->value()) / 100;
	x_zone->x4 = (float)atoi(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X4_STR)->value()) / 100;

	pos_root_node = initial_pos_root_node->first_node(XML_ELEMENT_POS_Y_ZONE_STR);
	s_trapezoid* y_zone = new s_trapezoid;
	y_zone->x1 = (float)atoi(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X1_STR)->value()) / 100;
	y_zone->x2 = (float)atoi(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X2_STR)->value()) / 100;
	y_zone->x3 = (float)atoi(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X3_STR)->value()) / 100;
	y_zone->x4 = (float)atoi(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X4_STR)->value()) / 100;

	pos_root_node = initial_pos_root_node->first_node(XML_ELEMENT_POS_TH_ZONE_STR);
	s_trapezoid* th_zone = new s_trapezoid;
	th_zone->x1 = (float)atof(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X1_STR)->value());
	th_zone->x2 = (float)atof(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X2_STR)->value());
	th_zone->x3 = (float)atof(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X3_STR)->value());
	th_zone->x4 = (float)atof(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X4_STR)->value());

	robotConfig->navParams->initialPosition->xZone = x_zone;
	robotConfig->navParams->initialPosition->yZone = y_zone;
	robotConfig->navParams->initialPosition->thZone = th_zone;
	

	xml_node<>* process_noise_root_node = nav_root_node->first_node(XML_ELEMENT_PROCESS_NOISE_STR);
	pos_root_node = process_noise_root_node->first_node(XML_ELEMENT_POS_D_ZONE_STR);
	s_trapezoid* x1_zone = new s_trapezoid;
	x1_zone->x1 = (float)atof(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X1_STR)->value()) / 1000;
	x1_zone->x2 = (float)atof(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X2_STR)->value()) / 1000;
	x1_zone->x3 = (float)atof(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X3_STR)->value()) / 1000;
	x1_zone->x4 = (float)atof(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X4_STR)->value()) / 1000;

	pos_root_node = process_noise_root_node->first_node(XML_ELEMENT_POS_TH_ZONE_STR);
	s_trapezoid* th1_zone = new s_trapezoid;
	th1_zone->x1 = (float)atof(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X1_STR)->value());
	th1_zone->x2 = (float)atof(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X2_STR)->value());
	th1_zone->x3 = (float)atof(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X3_STR)->value());
	th1_zone->x4 = (float)atof(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X4_STR)->value());

	robotConfig->navParams->processNoise->dZone = x1_zone;
	robotConfig->navParams->processNoise->thZone = th1_zone;


	xml_node<>* observation_noise_root_node = nav_root_node->first_node(XML_ELEMENT_OBSERV_NOISE_STR);
	pos_root_node = observation_noise_root_node->first_node(XML_ELEMENT_POS_D_ZONE_STR);
	s_trapezoid* d_zone = new s_trapezoid;
	d_zone->x1 = (float)atoi(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X1_STR)->value()) / 1000;
	d_zone->x2 = (float)atoi(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X2_STR)->value()) / 1000;
	d_zone->x3 = (float)atoi(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X3_STR)->value()) / 1000;
	d_zone->x4 = (float)atoi(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X4_STR)->value()) / 1000;

	pos_root_node = observation_noise_root_node->first_node(XML_ELEMENT_POS_TH_ZONE_STR);
	s_trapezoid* th2_zone = new s_trapezoid;
	th2_zone->x1 = (float)atof(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X1_STR)->value());
	th2_zone->x2 = (float)atof(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X2_STR)->value());
	th2_zone->x3 = (float)atof(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X3_STR)->value());
	th2_zone->x4 = (float)atof(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X4_STR)->value());

	robotConfig->navParams->observationNoise->dZone = d_zone;
	robotConfig->navParams->observationNoise->thZone = th2_zone;

	
	the_file.close();
}

void GeneralController::getMapsAvailable(std::string& mapsAvailable){
	std::ostringstream buffer_str;
	buffer_str.clear();
	for(int i = 0; i < navSectors->size(); i++){

		buffer_str << navSectors->at(i)->id << ",";
		buffer_str << navSectors->at(i)->name << ",";
		buffer_str << navSectors->at(i)->width << ",";
		buffer_str << navSectors->at(i)->height;
		if(i != navSectors->size() - 1){
			buffer_str << "|";
		}
	}
	mapsAvailable = buffer_str.str();
}

void GeneralController::getMapInformationLandmarks(std::string& mapInformation){

	std::ostringstream buffer_str;
	buffer_str.clear();
	if(currentSector != NULL){
		for(int j = 0; j < currentSector->landmarks->size(); j++){
			buffer_str << currentSector->landmarks->at(j)->id << ",";
			buffer_str << currentSector->landmarks->at(j)->xpos << ",";
			buffer_str << currentSector->landmarks->at(j)->ypos;
			if(j != currentSector->landmarks->size() - 1){
				buffer_str << "|";
			}
		}			
	}
	mapInformation = buffer_str.str();

}


void GeneralController::getMapInformationFeatures(std::string& mapInformation){

	std::ostringstream buffer_str;
	buffer_str.clear();
	if(currentSector != NULL){
		for(int j = 0; j < currentSector->features->size(); j++){
			buffer_str << currentSector->features->at(j)->id << ",";
			buffer_str << currentSector->features->at(j)->name << ",";
			buffer_str << currentSector->features->at(j)->xpos << ",";
			buffer_str << currentSector->features->at(j)->ypos;
			if(j != currentSector->features->size() - 1){
				buffer_str << "|";
			}
		}	
	}
	mapInformation = buffer_str.str();
}

void GeneralController::getMapInformationSites(std::string& mapInformation){

	std::ostringstream buffer_str;
	buffer_str.clear();
	if(currentSector != NULL){
		for(int j = 0; j < currentSector->sites->size(); j++){
			buffer_str << currentSector->sites->at(j)->id << ",";
			buffer_str << currentSector->sites->at(j)->name << ",";
			buffer_str << currentSector->sites->at(j)->xpos << ",";
			buffer_str << currentSector->sites->at(j)->ypos;
			if(j != currentSector->sites->size() - 1){
				buffer_str << "|";
			}
		}	
	}
	mapInformation = buffer_str.str();
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
	Sleep(100);
	ros::spinOnce();
}

void GeneralController::setRobotPosition(float x, float y, float theta){
	ROS_INFO("new position is: {x: %f, y: %f, theta: %f}", x, y, theta);
	this->setPosition(x, y, theta);	
}

void GeneralController::setRobotPosition(Matrix Xk){
	this->setRobotPosition(Xk(0, 0), Xk(1, 0), Xk(2, 0));
}

void GeneralController::moveRobotToPosition(float x, float y, float theta){
	ROS_INFO("Moving to position: {x: %f, y: %f, theta: %f}", x, y, theta);
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

	sprintf(bump, "$BUMPERS|%d,%d,%d,%d,%d,%d|%d,%d,%d,%d,%d,%d", (int)front[0], (int)front[1], (int)front[2], (int)front[3], (int)front[4], (int)front[5],
			(int)rear[0], (int)rear[1], (int)rear[2], (int)rear[3], (int)rear[4], (int)rear[5]);
	int dataLen = strlen(bump);
	if(spdUDPClient != NULL){
		//spdUDPClient->sendData((unsigned char*)bump, dataLen);
	}
}

void GeneralController::onPositionUpdate(double x, double y, double theta, double transSpeed, double rotSpeed){

	robotEncoderPosition(0, 0) = x;
	robotEncoderPosition(1, 0) = y;
	robotEncoderPosition(2, 0) = theta;
	
	robotVelocity(0, 0) = transSpeed;
	robotVelocity(1, 0) = rotSpeed;
	
	char* bump = new char[256];
	sprintf(bump, "$POSE_VEL|%.4f,%.4f,%.4f|%.4f,%.4f", robotEncoderPosition(0, 0), robotEncoderPosition(1, 0), robotEncoderPosition(2, 0), robotVelocity(0, 0), robotVelocity(1, 0));
	int dataLen = strlen(bump);
	if(spdUDPClient != NULL){
		spdUDPClient->sendData((unsigned char*)bump, dataLen);
	}
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
			
			for(int i = 0; i< currentSector->features->size(); i++){
				if(currentSector->features->at(i)->name == SEMANTIC_FEATURE_CHARGER_STR){
					setChargerPosition = true;
					float xpos = currentSector->features->at(i)->xpos;
					float ypos = currentSector->features->at(i)->ypos;
					Sleep(100);
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
	pthread_mutex_lock(&mutexLandmarkLocker);
	landmarks.clear();
	
	std::vector<float> dataMean;
	std::vector<float> dataAngles;

	if(dataIndices.size() > 0){
		for(int i = 0; i < dataIndices.size() - 1; i++){
			if((dataIndices[i + 1] - dataIndices[i]) <= 4){
				dataMean.push_back(data->at(dataIndices[i]));
				dataAngles.push_back((angle_min + ((float)dataIndices[i] * angle_increment)));
			} else {
				Matrix temp = Matrix(2, 1);
		
				dataMean.push_back(data->at(dataIndices[i]));
				dataAngles.push_back((angle_min + ((float)dataIndices[i] * angle_increment)));
				
				
				float distMean = stats::expectation(dataMean);
				float angleMean = stats::expectation(dataAngles);
				//float distMean = (dataMean.front() + dataMean.back()) / 2.0;
				//float angleMean = (dataAngles.front() + dataAngles.back()) / 2.0;
				
				temp(0, 0) = distMean;
				temp(1, 0) = angleMean;
				landmarks.push_back(temp);
				dataMean.clear();
				dataAngles.clear();
			}
		}

		if(dataMean.size() > 0){
			Matrix temp = Matrix(2, 1);
		
			dataMean.push_back(data->at(dataIndices[dataIndices.size() - 1]));
			dataAngles.push_back((angle_min + ((float)dataIndices[dataIndices.size() - 1] * angle_increment)));
			
			float distMean = stats::expectation(dataMean);
			float angleMean = stats::expectation(dataAngles);
			
			temp(0, 0) = distMean;
			temp(1, 0) = angleMean;
			landmarks.push_back(temp);
			dataMean.clear();
			dataAngles.clear();
		}
	}

	pthread_mutex_unlock(&mutexLandmarkLocker);
}

void GeneralController::startSitesTour(){
	pthread_t tourThread;
	stopCurrentTour();
	
	std::cout << "Starting tour in current sector..." << std::endl;
	pthread_create(&tourThread, NULL, sitesTourThread, (void *)(this));
}

void* GeneralController::sitesTourThread(void* object){
	GeneralController* self = (GeneralController*)object;
	self->keepTourAlive = YES;
	int goalIndex = 0;
	
	while(ros::ok() && self->keepTourAlive == YES){
		float xpos = self->currentSector->sites->at(goalIndex)->xpos;
		float ypos = self->currentSector->sites->at(goalIndex)->ypos;
		Sleep(100);
		self->moveRobotToPosition(xpos, ypos, 0.0);
		goalIndex++;
		if(goalIndex == self->currentSector->sites->size()) goalIndex = 0;
		while(!self->isGoalAchieved() && self->keepTourAlive == YES) Sleep(100);
	}
	self->keepTourAlive = NO;
	return NULL;
}

void GeneralController::stopCurrentTour(){
	if(keepTourAlive == YES){
		keepTourAlive = MAYBE;
		std::cout << "Stopping all tours..." << std::endl;
		while(keepTourAlive != NO) Sleep(100);
		ROS_INFO("All tours stopped...");
	}
}


void GeneralController::initializeKalmanVariables(){
	
	float uX, uY, uTh;

	fuzzy::trapezoid* xxKK = new fuzzy::trapezoid("Xx(k|k)", robotRawEncoderPosition(0, 0) + robotConfig->navParams->initialPosition->xZone->x1, robotRawEncoderPosition(0, 0) + robotConfig->navParams->initialPosition->xZone->x2, robotRawEncoderPosition(0, 0) + robotConfig->navParams->initialPosition->xZone->x3, robotRawEncoderPosition(0, 0) + robotConfig->navParams->initialPosition->xZone->x4);
	
	fuzzy::trapezoid* xyKK = new fuzzy::trapezoid("Xy(k|k)", robotRawEncoderPosition(1, 0) + robotConfig->navParams->initialPosition->yZone->x1, robotRawEncoderPosition(1, 0) + robotConfig->navParams->initialPosition->yZone->x2, robotRawEncoderPosition(1, 0) + robotConfig->navParams->initialPosition->yZone->x3, robotRawEncoderPosition(1, 0) + robotConfig->navParams->initialPosition->yZone->x4);	
	
	fuzzy::trapezoid* xThKK = new fuzzy::trapezoid("XTh(k|k)", robotRawEncoderPosition(2, 0) + robotConfig->navParams->initialPosition->thZone->x1, robotRawEncoderPosition(2, 0) + robotConfig->navParams->initialPosition->thZone->x2, robotRawEncoderPosition(2, 0) + robotConfig->navParams->initialPosition->thZone->x3, robotRawEncoderPosition(2, 0) + robotConfig->navParams->initialPosition->thZone->x4);	
	
	fuzzy::trapezoid* vdK1 = new fuzzy::trapezoid("Vd(k + 1)", robotConfig->navParams->processNoise->dZone->x1, robotConfig->navParams->processNoise->dZone->x2, robotConfig->navParams->processNoise->dZone->x3, robotConfig->navParams->processNoise->dZone->x4);
	
	fuzzy::trapezoid* vThK1 = new fuzzy::trapezoid("VTh(k + 1)", robotConfig->navParams->processNoise->thZone->x1, robotConfig->navParams->processNoise->thZone->x2, robotConfig->navParams->processNoise->thZone->x3, robotConfig->navParams->processNoise->thZone->x4);	
	
	fuzzy::trapezoid* wdK1 = new fuzzy::trapezoid("Wd(k + 1)", robotConfig->navParams->observationNoise->dZone->x1, robotConfig->navParams->observationNoise->dZone->x2, robotConfig->navParams->observationNoise->dZone->x3, robotConfig->navParams->observationNoise->dZone->x4);
	
	fuzzy::trapezoid* wThK1 = new fuzzy::trapezoid("WTh(k + 1)", robotConfig->navParams->observationNoise->thZone->x1, robotConfig->navParams->observationNoise->thZone->x2, robotConfig->navParams->observationNoise->thZone->x3, robotConfig->navParams->observationNoise->thZone->x4);	
	
	uX = fuzzy::fstats::uncertainty(xxKK->getVertexA(), xxKK->getVertexB(), xxKK->getVertexC(), xxKK->getVertexD());
	uY = fuzzy::fstats::uncertainty(xyKK->getVertexA(), xyKK->getVertexB(), xyKK->getVertexC(), xyKK->getVertexD());
	uTh = fuzzy::fstats::uncertainty(xThKK->getVertexA(), xThKK->getVertexB(), xThKK->getVertexC(), xThKK->getVertexD());

	P(0, 0) = uX; 	P(0, 1) = 0; 	P(0, 2) = 0;
	P(1, 0) = 0; 	P(1, 1) = uY;	P(1, 2) = 0;
	P(2, 0) = 0;	P(2, 1) = 0;	P(2, 2) = uTh;
	
	// Variances and Covariances Matrix of Process noise Q

	uX = fuzzy::fstats::uncertainty(vdK1->getVertexA(), vdK1->getVertexB(), vdK1->getVertexC(), vdK1->getVertexD());
	uTh = fuzzy::fstats::uncertainty(vThK1->getVertexA(), vThK1->getVertexB(), vThK1->getVertexC(), vThK1->getVertexD());
	
	Q(0, 0) = uX; 	Q(0, 1) = 0;
	Q(1, 0) = 0; 	Q(1, 1) = uTh;

	uX = fuzzy::fstats::uncertainty(wdK1->getVertexA(), wdK1->getVertexB(), wdK1->getVertexC(), wdK1->getVertexD());
	uTh = fuzzy::fstats::uncertainty(wThK1->getVertexA(), wThK1->getVertexB(), wThK1->getVertexC(), wThK1->getVertexD());

	R =  Matrix(2 * currentSector->landmarks->size(), 2 * currentSector->landmarks->size());
	for (int i = 0; i < R.cols_size(); i++){
		if((i % 2) != 0){
			R(i, i) = uTh;
		} else {
			R(i, i) = uX;
		}
	}

	kalmanFuzzy->clear();
	kalmanFuzzy->push_back(xxKK);
	kalmanFuzzy->push_back(xyKK);
	kalmanFuzzy->push_back(xThKK);
	
	kalmanFuzzy->push_back(vdK1);
	kalmanFuzzy->push_back(vThK1);
	
	kalmanFuzzy->push_back(wdK1);
	kalmanFuzzy->push_back(wThK1);
}

void GeneralController::trackRobot(){
	stopRobotTracking();
	initializeKalmanVariables();
	std::cout << "Tracking Doris..." << std::endl;
	pthread_create(&trackThread, NULL, trackRobotThread, (void *)(this));
	
}

void* GeneralController::trackRobotThread(void* object){
	GeneralController* self = (GeneralController*)object;
		
	self->keepRobotTracking = YES;
	float alpha = self->robotConfig->navParams->alpha;
	Matrix Ak = Matrix::eye(3);
	Matrix Bk(3, 2);
	Matrix pk1;
	Matrix Hk;
	Matrix Pk = self->P;
	
	while(ros::ok() && self->keepRobotTracking == YES){
		//1 - Prediction
		//ROS_INFO("1 - Prediction");

		pk1 = Pk;
		
		Matrix matXk(STATE_VARIABLES, TRAP_VERTEX);

		matXk(0, 0) = self->kalmanFuzzy->at(X_INDEX)->getVertexA();
		matXk(0, 1) = self->kalmanFuzzy->at(X_INDEX)->getVertexB();
		matXk(0, 2) = self->kalmanFuzzy->at(X_INDEX)->getVertexC();
		matXk(0, 3) = self->kalmanFuzzy->at(X_INDEX)->getVertexD();

		matXk(1, 0) = self->kalmanFuzzy->at(X_INDEX + 1)->getVertexA();
		matXk(1, 1) = self->kalmanFuzzy->at(X_INDEX + 1)->getVertexB();
		matXk(1, 2) = self->kalmanFuzzy->at(X_INDEX + 1)->getVertexC();
		matXk(1, 3) = self->kalmanFuzzy->at(X_INDEX + 1)->getVertexD();

		matXk(2, 0) = self->kalmanFuzzy->at(X_INDEX + 2)->getVertexA();
		matXk(2, 1) = self->kalmanFuzzy->at(X_INDEX + 2)->getVertexB();
		matXk(2, 2) = self->kalmanFuzzy->at(X_INDEX + 2)->getVertexC();
		matXk(2, 3) = self->kalmanFuzzy->at(X_INDEX + 2)->getVertexD();
		
		std::cout << "Position Xk(k|k): " << std::endl;
		matXk.print();
        std::cout << "Pk(k|k): " << std::endl;
		Pk.print();

		Ak(0, 2) = -self->robotRawDeltaPosition(0, 0) * std::sin(self->robotRawEncoderPosition(2, 0) + self->robotRawDeltaPosition(1, 0)/2);
		Ak(1, 2) = self->robotRawDeltaPosition(0, 0) * std::cos(self->robotRawEncoderPosition(2, 0) + self->robotRawDeltaPosition(1, 0)/2);

		Bk(0, 0) = std::cos(self->robotRawEncoderPosition(2, 0) + self->robotRawDeltaPosition(1, 0)/2);
		Bk(0, 1) = -0.5 * self->robotRawDeltaPosition(0, 0) * std::sin(self->robotRawEncoderPosition(2, 0) + self->robotRawDeltaPosition(1, 0)/2);

		Bk(1, 0) = std::sin(self->robotRawEncoderPosition(2, 0) + self->robotRawDeltaPosition(1, 0)/2);
		Bk(1, 1) = 0.5 * self->robotRawDeltaPosition(0, 0) * std::cos(self->robotRawEncoderPosition(2, 0) + self->robotRawDeltaPosition(1, 0)/2);

		Bk(2, 0) = 0.0;
		Bk(2, 1) = 1.0;


		Matrix currentQ = self->Q;
		if(self->robotRawDeltaPosition(0, 0) == 0.0 and self->robotRawDeltaPosition(1, 0) == 0.0){
			currentQ = Matrix(2, 2);
		}

		//Pk = (Ak * pk1 * ~Ak) + (Bk * currentQ * ~Bk);
		Pk = (Ak * pk1 * ~Ak);
        std::cout << "Pk(k+1|k): " << std::endl;
		Pk.print();
		// 2 - Observation
		//ROS_INFO("2 - Observation");
		
		Hk = Matrix(2 * self->currentSector->landmarks->size(), STATE_VARIABLES);
		for(int i = 0, zIndex = 0; i < self->currentSector->landmarks->size(); i++, zIndex += 2){
			Hk(zIndex, 0) = -((self->currentSector->landmarks->at(i)->xpos) - self->robotRawDeltaPosition(0, 0))/std::sqrt(std::pow((self->currentSector->landmarks->at(i)->xpos) - self->robotRawDeltaPosition(0, 0), 2) + std::pow((self->currentSector->landmarks->at(i)->ypos) - self->robotRawDeltaPosition(1, 0), 2));
			Hk(zIndex, 1) = -((self->currentSector->landmarks->at(i)->ypos) - self->robotRawDeltaPosition(1, 0))/std::sqrt(std::pow((self->currentSector->landmarks->at(i)->xpos) - self->robotRawDeltaPosition(0, 0), 2) + std::pow((self->currentSector->landmarks->at(i)->ypos) - self->robotRawDeltaPosition(1, 0), 2));
			Hk(zIndex, 2) = 0.0;

			Hk(zIndex + 1, 0) = ((self->currentSector->landmarks->at(i)->ypos) - self->robotRawDeltaPosition(1, 0))/(std::pow((self->currentSector->landmarks->at(i)->xpos) - self->robotRawDeltaPosition(0, 0), 2) + std::pow((self->currentSector->landmarks->at(i)->ypos) - self->robotRawDeltaPosition(1, 0), 2));
			Hk(zIndex + 1, 1) = -((self->currentSector->landmarks->at(i)->xpos) - self->robotRawDeltaPosition(0, 0))/(std::pow((self->currentSector->landmarks->at(i)->xpos) - self->robotRawDeltaPosition(0, 0), 2) + std::pow((self->currentSector->landmarks->at(i)->ypos) - self->robotRawDeltaPosition(1, 0), 2));
			Hk(zIndex + 1, 2) = -1.0;
		}

		std::vector<fuzzy::trapezoid*> zklWNoise;
		std::vector<fuzzy::trapezoid*> zklWONoise;
		self->getObservationsTrapezoids(zklWNoise, zklWONoise);

		for(int i = 0; i < zklWONoise.size(); i++){
		    std::cout << "Zkl Trap " << i  << ": (";
			std::cout << zklWONoise.at(i)->getVertexA() << ", ";
			std::cout << zklWONoise.at(i)->getVertexB() << ", ";
			std::cout << zklWONoise.at(i)->getVertexC() << ", ";
			std::cout << zklWONoise.at(i)->getVertexD() << ")" << std::endl;
		}
		
		// 3 - Matching
		//ROS_INFO("3 - Matching");
		ROS_INFO("Seen landmarks: %d", self->landmarks.size());
		pthread_mutex_lock(&self->mutexLandmarkLocker);
		Matrix zl(2 * self->currentSector->landmarks->size(), TRAP_VERTEX);
		for (int i = 0; i < self->landmarks.size(); i++){
			Matrix l = self->landmarks.at(i);
			int mode = 0;
			ROS_INFO("landmarks {d: %f, a: %f}", l(0, 0), l(1, 0));

			for (int j = 0; j < zklWONoise.size(); j+=2){
				float mdDistance = zklWONoise.at(j)->evaluate(l(0, 0));

				Matrix tempZkl(1, TRAP_VERTEX);
				tempZkl(0, 0) = zklWONoise.at(j+1)->getVertexA();
				tempZkl(0, 1) = zklWONoise.at(j+1)->getVertexB();
				tempZkl(0, 2) = zklWONoise.at(j+1)->getVertexC();
				tempZkl(0, 3) = zklWONoise.at(j+1)->getVertexD();

				if(self->isThirdQuadrant(l(1, 0))){
					mode = 1;
				}

				tempZkl = self->denormalizeAngles(tempZkl, mode);
				fuzzy::trapezoid* tempTrap = new fuzzy::trapezoid("", tempZkl(0, 0), tempZkl(0, 1), tempZkl(0, 2), tempZkl(0, 3));
				float mdAngle = tempTrap->evaluate(l(1, 0));
				
				if(mdDistance >= alpha && mdAngle >= alpha){
					ROS_INFO("Matched landmark: {d: %d, a: %d}", j, j+1);
					
					Matrix tempY(1, TRAP_VERTEX);
					tempY(0, 0) = l(0, 0) - zklWONoise.at(j)->getVertexA();
					tempY(0, 1) = l(0, 0) - zklWONoise.at(j)->getVertexB();
					tempY(0, 2) = l(0, 0) - zklWONoise.at(j)->getVertexC();
					tempY(0, 3) = l(0, 0) - zklWONoise.at(j)->getVertexD();

					for (int k = 0; k < tempY.cols_size(); k++){
						zl(j, k) = tempY(0, k);
					}

					tempY = l(1, 0) - tempZkl;

					for (int k = 0; k < tempY.cols_size(); k++){
						zl(j+1, k) = tempY(0, k);
					}
				}
			}
		}
		pthread_mutex_unlock(&self->mutexLandmarkLocker);
		// 4 - Correction
		//ROS_INFO("4 - Correction");	
		//Matrix matXk(STATE_VARIABLES, TRAP_VERTEX);

		matXk(0, 0) = self->kalmanFuzzy->at(X_INDEX)->getVertexA();
		matXk(0, 1) = self->kalmanFuzzy->at(X_INDEX)->getVertexB();
		matXk(0, 2) = self->kalmanFuzzy->at(X_INDEX)->getVertexC();
		matXk(0, 3) = self->kalmanFuzzy->at(X_INDEX)->getVertexD();

		matXk(1, 0) = self->kalmanFuzzy->at(X_INDEX + 1)->getVertexA();
		matXk(1, 1) = self->kalmanFuzzy->at(X_INDEX + 1)->getVertexB();
		matXk(1, 2) = self->kalmanFuzzy->at(X_INDEX + 1)->getVertexC();
		matXk(1, 3) = self->kalmanFuzzy->at(X_INDEX + 1)->getVertexD();

		matXk(2, 0) = self->kalmanFuzzy->at(X_INDEX + 2)->getVertexA();
		matXk(2, 1) = self->kalmanFuzzy->at(X_INDEX + 2)->getVertexB();
		matXk(2, 2) = self->kalmanFuzzy->at(X_INDEX + 2)->getVertexC();
		matXk(2, 3) = self->kalmanFuzzy->at(X_INDEX + 2)->getVertexD();

		std::cout << "Position Xk(k+1|k): " << std::endl;
		matXk.print();

		Matrix matThk(1, TRAP_VERTEX);
		matThk(0, 0) = matXk(2, 0);
		matThk(0, 1) = matXk(2, 1);
		matThk(0, 2) = matXk(2, 2);
		matThk(0, 3) = matXk(2, 3);

		matThk = self->denormalizeAngles(matThk);
		
		matXk(2, 0) = matThk(0, 0);
		matXk(2, 1) = matThk(0, 1);
		matXk(2, 2) = matThk(0, 2);
		matXk(2, 3) = matThk(0, 3);


		Matrix Sk = Hk * Pk * ~Hk + self->R;
		Matrix Wk = Pk * ~Hk * !Sk;

		Matrix stateVariation = self->multTrapMatrix(Wk, zl);
		stateVariation = Wk * zl;
		std::cout << "State Variation: " << std::endl;
		stateVariation.print();
		matXk = matXk + stateVariation;

		matThk = matXk(2);


		matXk = matXk.sort_cols();
        Pk = Pk - Wk * Sk * ~Wk;

        float xkn = fuzzy::fstats::expectation(matXk(0, 0), matXk(0, 1), matXk(0, 2), matXk(0, 3));
        float ykn = fuzzy::fstats::expectation(matXk(1, 0), matXk(1, 1), matXk(1, 2), matXk(1, 3));
        float thkn = fuzzy::fstats::expectation(matThk(0, 0), matThk(0, 1), matThk(0, 2), matThk(0, 3));
                
		matThk = self->normalizeAngles(matThk);
		//std::cout << "normalizeAngles:" << std::endl << matThk;	

		matXk(2, 0) = matThk(0, 0);
		matXk(2, 1) = matThk(0, 1);
		matXk(2, 2) = matThk(0, 2);
		matXk(2, 3) = matThk(0, 3);

		self->kalmanFuzzy->at(X_INDEX)->setVertexA(matXk(0, 0));
		self->kalmanFuzzy->at(X_INDEX)->setVertexB(matXk(0, 1));
		self->kalmanFuzzy->at(X_INDEX)->setVertexC(matXk(0, 2));
		self->kalmanFuzzy->at(X_INDEX)->setVertexD(matXk(0, 3));

		self->kalmanFuzzy->at(X_INDEX + 1)->setVertexA(matXk(1, 0));
		self->kalmanFuzzy->at(X_INDEX + 1)->setVertexB(matXk(1, 1));
		self->kalmanFuzzy->at(X_INDEX + 1)->setVertexC(matXk(1, 2));
		self->kalmanFuzzy->at(X_INDEX + 1)->setVertexD(matXk(1, 3));

		self->kalmanFuzzy->at(X_INDEX + 2)->setVertexA(matThk(0, 0));
		self->kalmanFuzzy->at(X_INDEX + 2)->setVertexB(matThk(0, 1));
		self->kalmanFuzzy->at(X_INDEX + 2)->setVertexC(matThk(0, 2));
		self->kalmanFuzzy->at(X_INDEX + 2)->setVertexD(matThk(0, 3));	
		
		self->setRobotPosition(xkn, ykn, thkn);
		
		Sleep(90);
		//self->keepRobotTracking = NO;
	}
	self->keepRobotTracking = NO;
	return NULL;
}

Matrix GeneralController::multTrapMatrix(Matrix mat, Matrix trap){
	Matrix result(STATE_VARIABLES, TRAP_VERTEX);
	for(int i = 0; i < mat.rows_size(); i++){
		Matrix sumResult(1, TRAP_VERTEX);
		for(int j = 0; j < mat.cols_size(); j++){
			Matrix tempResult(1, TRAP_VERTEX);
			tempResult(0, 0) = mat(i, j) * trap(j, 0);
			tempResult(0, 1) = mat(i, j) * trap(j, 1);
			tempResult(0, 2) = mat(i, j) * trap(j, 2);
			tempResult(0, 3) = mat(i, j) * trap(j, 3);

			//tempResult = tempResult.sort_cols();
			sumResult = sumResult + tempResult;
			//sumResult = sumResult.sort_cols();
		}

		result(i, 0) = sumResult(0, 0);
		result(i, 1) = sumResult(0, 1);
		result(i, 2) = sumResult(0, 2);
		result(i, 3) = sumResult(0, 3);
	}
	return result;
}

void GeneralController::getObservationsTrapezoids(std::vector<fuzzy::trapezoid*> &obsWithNoise, std::vector<fuzzy::trapezoid*> &obsWONoise){
	
	fuzzy::trapezoid *trapX = kalmanFuzzy->at(X_INDEX);
	fuzzy::trapezoid *trapY = kalmanFuzzy->at(X_INDEX + 1);
	fuzzy::trapezoid *trapTh = kalmanFuzzy->at(X_INDEX + 2);

	Matrix zklt(2 * currentSector->landmarks->size(), 4);
	Matrix matXk(STATE_VARIABLES, TRAP_VERTEX);
	matXk(0, 0) = trapX->getVertexA();
	matXk(0, 1) = trapX->getVertexB();
	matXk(0, 2) = trapX->getVertexC();
	matXk(0, 3) = trapX->getVertexD();

	matXk(1, 0) = trapY->getVertexA();
	matXk(1, 1) = trapY->getVertexB();
	matXk(1, 2) = trapY->getVertexC();
	matXk(1, 3) = trapY->getVertexD();

	matXk(2, 0) = trapTh->getVertexA();
	matXk(2, 1) = trapTh->getVertexB();
	matXk(2, 2) = trapTh->getVertexC();
	matXk(2, 3) = trapTh->getVertexD();

	Matrix matThk(1, TRAP_VERTEX);
	matThk(0, 0) = matXk(2, 0);
	matThk(0, 1) = matXk(2, 1);
	matThk(0, 2) = matXk(2, 2);
	matThk(0, 3) = matXk(2, 3);

	matThk = denormalizeAngles(matThk, 1);
		
	matXk(2, 0) = matThk(0, 0);
	matXk(2, 1) = matThk(0, 1);
	matXk(2, 2) = matThk(0, 2);
	matXk(2, 3) = matThk(0, 3);

	Matrix var(STATE_VARIABLES, 1);
	var(0, 0) = (matXk(0, 3) - matXk(0, 0)) / 4;
	var(1, 0) = (matXk(1, 3) - matXk(1, 0)) / 4;
	var(2, 0) = (matXk(2, 3) - matXk(2, 0)) / 4;

	for(int k = 0; k < currentSector->landmarks->size(); k++){
		s_landmark* landmark = currentSector->landmarks->at(k);
		Matrix state(STATE_VARIABLES, 1);
		float distance = 0, angle = 0;
		state(0, 0) = matXk(0, 0);
		state(1, 0) = matXk(1, 0);
		state(2, 0) = matXk(2, 0);
		landmarkObservation(state, landmark, distance, angle);

		zklt(2*k, 0) = distance;
		zklt(2*k + 1, 0) = angle;

		zklt(2*k, 3) = distance;
		zklt(2*k + 1, 3) = angle;

	}

	for(float x = matXk(0, 0); x < (matXk(0, 3) + var(0, 0)/4); x+=var(0, 0)){
		for(float y = matXk(1, 0); y < (matXk(1, 3) + var(1, 0)/4); y+=var(1, 0)){
			for(float th = matXk(2, 0); th < (matXk(2, 3) + var(2, 0)/4); th+=var(2, 0)){
				for(int k = 0; k < currentSector->landmarks->size(); k++){
					s_landmark* landmark = currentSector->landmarks->at(k);
					Matrix state(STATE_VARIABLES, 1);
					float distance = 0, angle = 0;
					state(0, 0) = x;
					state(1, 0) = y;
					state(2, 0) = th;
					landmarkObservation(state, landmark, distance, angle);

					if(distance < zklt(2*k, 0)){
	                    zklt(2*k, 0) = distance;
	            	} else if (distance > zklt(2*k, 3)){
	                    zklt(2*k, 3) = distance;
	                }

					if(angle < zklt(2*k + 1, 0)){
						zklt(2*k + 1, 0) = angle;
					} else if(angle > zklt(2*k + 1, 3)){
						zklt(2*k + 1, 3) = angle;
					}
				}
			}
		}
	}

	var(0, 0) = (matXk(0, 2) - matXk(0, 1)) / 4;
	var(1, 0) = (matXk(1, 2) - matXk(1, 1)) / 4;
	var(2, 0) = (matXk(2, 2) - matXk(2, 1)) / 4;

	for(int k = 0; k < currentSector->landmarks->size(); k++){
		s_landmark* landmark = currentSector->landmarks->at(k);
		Matrix state(STATE_VARIABLES, 1);
		float distance = 0, angle = 0;
		state(0, 0) = matXk(0, 1);
		state(1, 0) = matXk(1, 1);
		state(2, 0) = matXk(2, 1);
		landmarkObservation(state, landmark, distance, angle);

		zklt(2*k, 1) = distance;
		zklt(2*k + 1, 1) = angle;

		zklt(2*k, 2) = distance;
		zklt(2*k + 1, 2) = angle;

	}
	
	for(float x = matXk(0, 1); x < (matXk(0, 2) + var(0, 0)/4); x+=var(0, 0)){
		for(float y = matXk(1, 1); y < (matXk(1, 2) + var(1, 0)/4); y+=var(1, 0)){
			for(float th = matXk(2, 1); th < (matXk(2, 2) + var(2, 0)/4); th+=var(2, 0)){
				for(int k = 0; k < currentSector->landmarks->size(); k++){
					s_landmark* landmark = currentSector->landmarks->at(k);
					Matrix state(STATE_VARIABLES, 1);
					float distance = 0, angle = 0;
					state(0, 0) = x;
					state(1, 0) = y;
					state(2, 0) = th;
					landmarkObservation(state, landmark, distance, angle);

					if(distance < zklt(2*k, 1)){
						zklt(2*k, 1) = distance;
					} else if(distance > zklt(2*k, 2)){
						zklt(2*k, 2) = distance;
					}

					if(angle < zklt(2*k + 1, 1)){
						zklt(2*k + 1, 1) = angle;
					} else if(angle > zklt(2*k + 1, 2)){
						zklt(2*k + 1, 2) = angle;
					}
				}
			}
		}
	}
	
	for(int i = 0; i < zklt.rows_size(); i++){
		Matrix results(1, TRAP_VERTEX);
		results(0, 0) = zklt(i, 0);
		results(0, 1) = zklt(i, 1);
		results(0, 2) = zklt(i, 2);
		results(0, 3) = zklt(i, 3);

		if((i%2) != 0){
			results = normalizeAngles(results);
			obsWONoise.push_back(new fuzzy::trapezoid("", results(0, 0), results(0, 1), results(0, 2), results(0, 3)));

			results(0, 0) += kalmanFuzzy->at(W_INDEX + 1)->getVertexA();
			results(0, 1) += kalmanFuzzy->at(W_INDEX + 1)->getVertexB();
			results(0, 2) += kalmanFuzzy->at(W_INDEX + 1)->getVertexC();
			results(0, 3) += kalmanFuzzy->at(W_INDEX + 1)->getVertexD();

			results = normalizeAngles(results);

		} else {
			results = results.sort_cols();
			obsWONoise.push_back(new fuzzy::trapezoid("", results(0, 0), results(0, 1), results(0, 2), results(0, 3)));

			results(0, 0) += kalmanFuzzy->at(W_INDEX)->getVertexA();
			results(0, 1) += kalmanFuzzy->at(W_INDEX)->getVertexB();
			results(0, 2) += kalmanFuzzy->at(W_INDEX)->getVertexC();
			results(0, 3) += kalmanFuzzy->at(W_INDEX)->getVertexD();
		}
		
		obsWithNoise.push_back(new fuzzy::trapezoid("", results(0, 0), results(0, 1), results(0, 2), results(0, 3)));
	}
}

void GeneralController::landmarkObservation(Matrix Xk, s_landmark* landmark, float& distance, float& angle){
	distance = std::sqrt(std::pow(landmark->xpos - Xk(0, 0), 2) + std::pow(landmark->ypos - Xk(1, 0), 2));
	angle = std::atan2(landmark->ypos - Xk(1, 0), landmark->xpos - Xk(0, 0)) - Xk(2, 0);
}

Matrix GeneralController::normalizeAngles(Matrix trap){
	Matrix tempTrap = trap;
	Matrix result(1, TRAP_VERTEX);

	for(int j = 0; j < TRAP_VERTEX; j++){
		if(tempTrap(0, j) > M_PI){
			tempTrap(0, j) = tempTrap(0, j) - 2 * M_PI;
		} else if(tempTrap(0, j) < -M_PI){
			tempTrap(0, j) = tempTrap(0, j) + 2 * M_PI;	
		}
	}
    
    result = tempTrap.sort_cols();

	return result;
}

Matrix GeneralController::denormalizeAngles(Matrix trap, int mode){
	Matrix tempTrap = trap;
	Matrix result(TRAP_VERTEX, 1);

	if(isThirdQuadrant(tempTrap(0, 0)) &&
       isSecondQuadrant(tempTrap(0, 1)) &&
       isSecondQuadrant(tempTrap(0, 2)) &&
       isSecondQuadrant(tempTrap(0, 3))){
		tempTrap(0, 0) = tempTrap(0, 0) + 2 * M_PI;
		
		result = tempTrap.sort_cols();

    } else if(isThirdQuadrant(tempTrap(0, 0)) &&
              isThirdQuadrant(tempTrap(0, 1)) &&
              isSecondQuadrant(tempTrap(0, 2)) &&
              isSecondQuadrant(tempTrap(0, 3))){
    	if(mode != 1){
			tempTrap(0, 0) = tempTrap(0, 0) + 2 * M_PI;
			tempTrap(0, 1) = tempTrap(0, 1) + 2 * M_PI;
		} else {
			tempTrap(0, 2) = tempTrap(0, 2) - 2 * M_PI;
			tempTrap(0, 3) = tempTrap(0, 3) - 2 * M_PI;
		}
		result = tempTrap.sort_cols();

    } else if(isThirdQuadrant(tempTrap(0, 0)) &&
              isThirdQuadrant(tempTrap(0, 1)) &&
              isThirdQuadrant(tempTrap(0, 2)) &&
              isFouthQuadrant(tempTrap(0, 3))){
		tempTrap(0, 3) = tempTrap(0, 3) - 2 * M_PI;
		result = tempTrap.sort_cols();
	} else {
		result = trap;
	}
	

	return result;
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

void GeneralController::stopRobotTracking(){
	if(keepRobotTracking == YES){
		keepRobotTracking = MAYBE;
		std::cout << "Stopping robot tracking thread" << std::endl;
		pthread_cancel(trackThread);
		keepRobotTracking = NO;
		//while(keepRobotTracking != NO) Sleep(100);
		std::cout << "Stopped robot tracking thread" << std::endl;
	}
}

void GeneralController::beginVideoStreaming(int videoDevice){
	pthread_t t1;
	stopVideoStreaming();

	
	vc = cv::VideoCapture(videoDevice);
	/*if(videoDevice == 0){
		vcSecond = cv::VideoCapture(2);

	}*/

	if(vc.isOpened()){
		std::cout << "Streaming from camera device: " << videoDevice << std::endl;
		pthread_create(&t1, NULL, streamingThread, (void *)(this));
	} else {
		vc.release();
		std::cout << "Could not open device: " << videoDevice << std::endl;
	}	
	 

}

void GeneralController::stopVideoStreaming(){
	if(streamingActive == YES){
		streamingActive = MAYBE;
		std::cout << "Stopping video streaming" << std::endl;
		while(streamingActive != NO) Sleep(100);
		vc.release();
	}
}

void* GeneralController::streamingThread(void* object){
	GeneralController* self = (GeneralController*)object;
	self->streamingActive = YES;

	cv::Stitcher stitcher = cv::Stitcher::createDefault(true);
	cv::Stitcher::Status status;

	cv::Mat frame;
	std::vector<uchar> buff;
	std::vector<int> params = vector<int>(2);
	params[0] = CV_IMWRITE_JPEG_QUALITY;
	params[1] = 80;
	
	UDPClient* udp_client = new UDPClient(self->getClientIPAddress(), self->udpPort);
	while(ros::ok() && self->streamingActive == YES){
		self->vc >> frame;
		cv::imencode(".jpg", frame, buff, params);
		udp_client->sendData(&buff[0], buff.size());
		Sleep(30);
	}
	udp_client->closeConnection();
	self->vc.release();
	if(self->vcSecond.isOpened()){
		self->vcSecond.release();
	}
	self->streamingActive = NO;
	return NULL;
}


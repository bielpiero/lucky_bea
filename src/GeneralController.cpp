#include <stdlib.h>
#include <stdio.h>
#include "GeneralController.h"


const float GeneralController::LASER_MAX_RANGE = 11.6;
const float GeneralController::MIN_RAND = -1;
const float GeneralController::MAX_RAND = 1;

GeneralController::GeneralController(ros::NodeHandle nh_){
	this->nh = nh_;
	
	this->maestroControllers = new SerialPort();
	
	this->keepSpinning = true;
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
	
	P = Matrix(3, 3);
	Q = Matrix(3, 3);
	R = Matrix(3, 3);
	spdUDPClient = NULL;

	xmlFaceFullPath = ros::package::getPath(PACKAGE_NAME) + XML_FILE_PATH;
	xmlSectorsFullPath = ros::package::getPath(PACKAGE_NAME) + XML_FILE_SECTORS_PATH;
	xmlRobotConfigFullPath = ros::package::getPath(PACKAGE_NAME) + XML_FILE_ROBOT_CONFIG_PATH;

	navSector = NULL;
	loadRobotConfig();
	loadSector(0);
	cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 1);
	cmd_goto_pub = nh.advertise<geometry_msgs::Pose2D>("/RosAria/cmd_goto", 1);
	pose2d_pub = nh.advertise<geometry_msgs::Pose2D>("/RosAria/cmd_set_pose", 1);
}


GeneralController::~GeneralController(void){
	stopDynamicGesture();
	stopVideoStreaming();
	stopRobotTracking();
	stopCurrentTour();
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
		stopRobotTracking();
		stopCurrentTour();
	}
}
void GeneralController::OnMsg(char* cad,int length){//callback for client and server
	cad[length] = 0;
	unsigned char function = *(cad++);
	std::string local_buffer_out = "";
	std::string gestures = "";
	std::string servo_positions = "";
	std::ostringstream number_converter;
	
	unsigned char port = 0;
	unsigned char card_id = 0;

	int servo_position = 0;
	int face_id, k=0;
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
			std::cout << "Command 0x08. Moving from CardId: " << (int)card_id << " Servo: " << (int)port << " To Position: " << servo_position << std::endl;
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
		case 0x13:
			getPositions(cad, x, y, theta);
			setRobotPosition(x, y, theta);
			break;
		case 0x14:
			getPositions(cad, x, y, theta);
			goToPosition(x, y, theta);
			break;
		case 0x20:
			getNumberOfCamerasAvailable(cameraCount);
			number_converter << cameraCount;
			local_buffer_out = number_converter.str();
			SendMsg(0x20, (char*)local_buffer_out.c_str(), (int)local_buffer_out.length()); 
			break;
		case 0x21:
			getCameraDevicePort(cad, videoDevice, udpPort);
			beginVideoStreaming(videoDevice);
			break;
		case 0x22:
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

void GeneralController::getPositions(char* cad, float& x, float& y, float& theta){
	char* current_number;
	int values[6];
	int index = 0;
	current_number = std::strtok(cad, ",");

	while(current_number != NULL){
		int cValue = std::atoi(current_number);
		values[index++] = (float)((float)cValue/1000.0);
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
		values[index++] = (float)((float)cValue/1000.0);
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
	std::cout << gestures << std::endl;
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
					setServoSpeed(card_id, servo_id, speed);
					setServoAcceleration(card_id, servo_id, acceleration);
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
    usleep(10000);
}

void GeneralController::setServoSpeed(unsigned char card_id, unsigned char servo_id, int speed){
    this->maestroControllers->setSpeed(card_id, servo_id, speed);
    usleep(10000);
}

void GeneralController::setServoAcceleration(unsigned char card_id, unsigned char servo_id, int acceleration){
    this->maestroControllers->setAcceleration(card_id, servo_id, acceleration);
    usleep(10000);
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
	if(root_node != NULL)
	{
		for (xml_node<> * repetir_node = root_node->first_node("Repetir"); repetir_node; repetir_node = repetir_node->next_sibling()){

			std::string idGesture(repetir_node->first_attribute("idGesto")->value());
			if(idGesture == node->id_gesto)
			{
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

	while(node->object->continue_dynamic_thread)
	{
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
	xml_document<> doc;
    xml_node<>* root_node;
	
	std::string buffer_str = "";
	
    std::ifstream the_file(xmlSectorsFullPath.c_str());
	
    std::vector<char> buffer((std::istreambuf_iterator<char>(the_file)), std::istreambuf_iterator<char>());
	buffer.push_back('\0');

	doc.parse<0>(&buffer[0]);
	root_node = doc.first_node(XML_ELEMENT_SECTORS_STR);

	if(navSector != NULL){
		delete navSector->landmarks;
		delete navSector->features;
		delete navSector->sites;
		delete navSector;
	}
	navSector = new s_sector;
	navSector->sitesCyclic = false;
	if(root_node != NULL){
		for (xml_node<> * sector_node = root_node->first_node(XML_ELEMENT_SECTOR_STR); sector_node; sector_node = sector_node->next_sibling()){	

			int xmlSectorId = atoi(sector_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value());
			if(xmlSectorId == sectorId){
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
					tempLandmark->var = atoi(landmark_node->first_attribute(XML_ATTRIBUTE_VARIANCE_STR)->value());
					tempLandmark->xpos = atoi(landmark_node->first_attribute(XML_ATTRIBUTE_X_POSITION_STR)->value());
					tempLandmark->ypos = atoi(landmark_node->first_attribute(XML_ATTRIBUTE_Y_POSITION_STR)->value());

					navSector->landmarks->push_back(tempLandmark);
				}

				xml_node<>* features_root_node = sector_node->first_node(XML_ELEMENT_FEATURES_STR);
				for(xml_node<>* features_node = features_root_node->first_node(XML_ELEMENT_FEATURE_STR); features_node; features_node = features_node->next_sibling()){
					s_feature* tempFeature = new s_feature;

					tempFeature->id = atoi(features_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value());
					tempFeature->name = std::string(features_node->first_attribute(XML_ATTRIBUTE_NAME_STR)->value());
					tempFeature->var = atoi(features_node->first_attribute(XML_ATTRIBUTE_VARIANCE_STR)->value());
					tempFeature->xpos = atoi(features_node->first_attribute(XML_ATTRIBUTE_X_POSITION_STR)->value());
					tempFeature->ypos = atoi(features_node->first_attribute(XML_ATTRIBUTE_Y_POSITION_STR)->value());

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
					tempSite->var = atoi(site_node->first_attribute(XML_ATTRIBUTE_VARIANCE_STR)->value());
					tempSite->tsec = atoi(site_node->first_attribute(XML_ATTRIBUTE_TIME_STR)->value());
					tempSite->xpos = atoi(site_node->first_attribute(XML_ATTRIBUTE_X_POSITION_STR)->value());
					tempSite->ypos = atoi(site_node->first_attribute(XML_ATTRIBUTE_Y_POSITION_STR)->value());

					navSector->sites->push_back(tempSite);
				}
				
			}
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
	robotConfig->navParams->processNoise = new s_position;
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
	th_zone->x1 = (float)atoi(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X1_STR)->value()) / 1000;
	th_zone->x2 = (float)atoi(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X2_STR)->value()) / 1000;
	th_zone->x3 = (float)atoi(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X3_STR)->value()) / 1000;
	th_zone->x4 = (float)atoi(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X4_STR)->value()) / 1000;

	robotConfig->navParams->initialPosition->xZone = x_zone;
	robotConfig->navParams->initialPosition->yZone = y_zone;
	robotConfig->navParams->initialPosition->thZone = th_zone;
	

	xml_node<>* process_noise_root_node = nav_root_node->first_node(XML_ELEMENT_PROCESS_NOISE_STR);
	pos_root_node = process_noise_root_node->first_node(XML_ELEMENT_POS_X_ZONE_STR);
	s_trapezoid* x1_zone = new s_trapezoid;
	x1_zone->x1 = (float)atoi(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X1_STR)->value()) / 100;
	x1_zone->x2 = (float)atoi(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X2_STR)->value()) / 100;
	x1_zone->x3 = (float)atoi(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X3_STR)->value()) / 100;
	x1_zone->x4 = (float)atoi(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X4_STR)->value()) / 100;

	pos_root_node = process_noise_root_node->first_node(XML_ELEMENT_POS_Y_ZONE_STR);
	s_trapezoid* y1_zone = new s_trapezoid;
	y1_zone->x1 = (float)atoi(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X1_STR)->value()) / 100;
	y1_zone->x2 = (float)atoi(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X2_STR)->value()) / 100;
	y1_zone->x3 = (float)atoi(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X3_STR)->value()) / 100;
	y1_zone->x4 = (float)atoi(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X4_STR)->value()) / 100;

	pos_root_node = process_noise_root_node->first_node(XML_ELEMENT_POS_TH_ZONE_STR);
	s_trapezoid* th1_zone = new s_trapezoid;
	th1_zone->x1 = (float)atoi(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X1_STR)->value()) / 1000;
	th1_zone->x2 = (float)atoi(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X2_STR)->value()) / 1000;
	th1_zone->x3 = (float)atoi(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X3_STR)->value()) / 1000;
	th1_zone->x4 = (float)atoi(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X4_STR)->value()) / 1000;

	robotConfig->navParams->processNoise->xZone = x1_zone;
	robotConfig->navParams->processNoise->yZone = y1_zone;
	robotConfig->navParams->processNoise->thZone = th1_zone;


	xml_node<>* observation_noise_root_node = nav_root_node->first_node(XML_ELEMENT_OBSERV_NOISE_STR);
	pos_root_node = observation_noise_root_node->first_node(XML_ELEMENT_POS_D_ZONE_STR);
	s_trapezoid* d_zone = new s_trapezoid;
	d_zone->x1 = (float)atoi(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X1_STR)->value()) / 100;
	d_zone->x2 = (float)atoi(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X2_STR)->value()) / 100;
	d_zone->x3 = (float)atoi(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X3_STR)->value()) / 100;
	d_zone->x4 = (float)atoi(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X4_STR)->value()) / 100;

	pos_root_node = observation_noise_root_node->first_node(XML_ELEMENT_POS_TH_ZONE_STR);
	s_trapezoid* th2_zone = new s_trapezoid;
	th2_zone->x1 = (float)atoi(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X1_STR)->value()) / 1000;
	th2_zone->x2 = (float)atoi(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X2_STR)->value()) / 1000;
	th2_zone->x3 = (float)atoi(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X3_STR)->value()) / 1000;
	th2_zone->x4 = (float)atoi(pos_root_node->first_attribute(XML_ATTRIBUTE_TRAP_X4_STR)->value()) / 1000;

	robotConfig->navParams->observationNoise->dZone = d_zone;
	robotConfig->navParams->observationNoise->thZone = th2_zone;

	
	the_file.close();
}

void GeneralController::moveRobot(float lin_vel, float angular_vel){
	geometry_msgs::Twist twist_msg;
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
		twist_msg.linear.x = lin_vel;
		twist_msg.linear.y = 0;
		twist_msg.linear.z = 0;
		twist_msg.angular.x = 0;
		twist_msg.angular.y = 0;
		twist_msg.angular.z = angular_vel;
		cmd_vel_pub.publish(twist_msg);
	} else {
		twist_msg.linear.x = 0;
		twist_msg.linear.y = 0;
		twist_msg.linear.z = 0;
		twist_msg.angular.x = 0;
		twist_msg.angular.y = 0;
		twist_msg.angular.z = 0;
		cmd_vel_pub.publish(twist_msg);
	}
	Sleep(100);
	ros::spinOnce();
	keepSpinning = true;
}

void GeneralController::setRobotPosition(float x, float y, float theta){
	Matrix Xk(3);
	
	Xk(0, 0) = x;
	Xk(1, 0) = y;
	Xk(2, 0) = theta;
	
	setRobotPosition(Xk);	
}

void GeneralController::setRobotPosition(Matrix Xk){
	geometry_msgs::Pose2D msg;
	
	msg.x = Xk(0, 0);
	msg.y = Xk(1, 0);
	msg.theta = Xk(2, 0);
	pose2d_pub.publish(msg);
	
	Sleep(100);
	ros::spinOnce();
}

void GeneralController::goToPosition(float x, float y, float theta){
	geometry_msgs::Pose2D msg;
	Matrix Xk(3);
	
	Xk(0, 0) = x;
	Xk(1, 0) = y;
	Xk(2, 0) = theta;

	msg.x = Xk(0, 0);
	msg.y = Xk(1, 0);
	msg.theta = Xk(2, 0);
	cmd_goto_pub.publish(msg);
	Sleep(100);
	ros::spinOnce();
}

void GeneralController::bumperStateCallback(const rosaria::BumperState::ConstPtr& bumpers){
	char* bump = new char[256];

	frontBumpersOk = true;
	rearBumpersOk = true;

	for (int i = 0; i < bumpers->front_bumpers.size(); i++){
		if (bumpers->front_bumpers[i] && frontBumpersOk){
			frontBumpersOk = false;
		}
	}

	for (int i = 0; i < bumpers->rear_bumpers.size(); i++){
		if (bumpers->rear_bumpers[i] && rearBumpersOk){
			rearBumpersOk = false;
		}
	}

	sprintf(bump, "$BUMPERS|%d,%d,%d,%d,%d,%d|%d,%d,%d,%d,%d,%d",bumpers->front_bumpers[0], bumpers->front_bumpers[1], bumpers->front_bumpers[2], bumpers->front_bumpers[3], bumpers->front_bumpers[4], bumpers->front_bumpers[5],
			bumpers->rear_bumpers[0], bumpers->rear_bumpers[1], bumpers->rear_bumpers[2], bumpers->rear_bumpers[3], bumpers->rear_bumpers[4], bumpers->rear_bumpers[5]);
	int dataLen = strlen(bump);
	if(spdUDPClient != NULL){
		//spdUDPClient->sendData((unsigned char*)bump, dataLen);
	}
}

void GeneralController::poseStateCallback(const nav_msgs::Odometry::ConstPtr& pose){
	float q3 = pose->pose.pose.orientation.z; //quaternion vector component z
	float q0 = pose->pose.pose.orientation.w; //quaternion scalar component
	
	float theta = std::atan2((2*(q0 * q3)), (1 - 2 * (std::pow(q3, 2))));
	
	robotEncoderPosition(0, 0) = pose->pose.pose.position.x;
	robotEncoderPosition(1, 0) = pose->pose.pose.position.y;
	robotEncoderPosition(2, 0) = theta;
	
	robotVelocity(0, 0) = pose->twist.twist.linear.x;
	robotVelocity(1, 0) = pose->twist.twist.angular.z;
	
	char* bump = new char[256];
	sprintf(bump, "$POSE_VEL|%.4f,%.4f,%.4f|%.4f,%.4f",robotEncoderPosition(0, 0), robotEncoderPosition(1, 0), robotEncoderPosition(2, 0), robotVelocity(0, 0), robotVelocity(1, 0));
	int dataLen = strlen(bump);
	if(spdUDPClient != NULL){
		spdUDPClient->sendData((unsigned char*)bump, dataLen);
	}
	
	Sleep(100);
}

//void GeneralController::batteryStateCallback(const std_msgs::Float32::ConstPtr& battery){
	//TODO: When Available.
//}

void GeneralController::sonarStateCallback(const sensor_msgs::PointCloud::ConstPtr& sonar){

}

void GeneralController::sonarPointCloud2StateCallback(const sensor_msgs::PointCloud2::ConstPtr& sonar){

}

void GeneralController::batteryVoltageCallback(const std_msgs::Float64::ConstPtr& battery){

}

void GeneralController::batteryRechargeStateCallback(const std_msgs::Int8::ConstPtr& battery){
	if(!setChargerPosition && ((int)battery->data) > 0){
		if(navSector != NULL){
			
			for(int i = 0; i< navSector->features->size(); i++){
				if(navSector->features->at(i)->name == SEMANTIC_FEATURE_CHARGER_STR){
					setChargerPosition = true;
					float xpos = (float)navSector->features->at(i)->xpos/100;
					float ypos = (float)navSector->features->at(i)->ypos/100;
					Sleep(100);
					setRobotPosition(xpos, ypos, M_PI/2);
				}
			}
		}
	}
}

void GeneralController::goalAchievementStateCallback(const std_msgs::Int8::ConstPtr& hasAchieved){
	if(((int)hasAchieved->data) == 0){
		this->hasAchievedGoal = false;
	} else {
		this->hasAchievedGoal = true;
	}

}

void GeneralController::laserScanStateCallback(const sensor_msgs::LaserScan::ConstPtr& laser){
	float range = MAX_RAND - MIN_RAND;
	float angle_min = laser->angle_min;
	float angle_max = laser->angle_max;
	float angle_increment = laser->angle_increment;
	std::vector<float> data = laser->ranges;
	std::vector<float> dataIntensities = laser->intensities;

	std::vector<int> dataIndices = stats::findIndicesHigherThan(dataIntensities, 0);
	
	landmarks.clear();
	
	std::vector<float> dataMean;
	std::vector<float> dataAngles;
	for(int i = 1; i < dataIndices.size(); i++){
		if(((dataIndices[i] - dataIndices[i - 1]) <= 2) && (i != (dataIndices.size() - 1))){
			dataMean.push_back(data[dataIndices[i - 1]]);
			dataAngles.push_back((angle_min + ((float)dataIndices[i - 1] * angle_increment)));
		} else {
			Matrix temp = Matrix(2, 1);
			if(i != (dataIndices.size() - 1)){
				dataMean.push_back(data[dataIndices[i - 1]]);
				dataAngles.push_back((angle_min + ((float)dataIndices[i - 1] * angle_increment)));
			} else {
				dataMean.push_back(data[dataIndices[i]]);
				dataAngles.push_back((angle_min + ((float)dataIndices[i] * angle_increment)));
			}
			
			float distMean = stats::expectation(dataMean);
			float angleMean = stats::expectation(dataAngles);
			
			//temp(0, 0) = distMean * cos(angleMean);
			//temp(1, 0) = distMean * sin(angleMean);
			temp(0, 0) = distMean;
			temp(1, 0) = angleMean;
			landmarks.push_back(temp);
			dataMean.clear();
			dataAngles.clear();
		}
	}
	Sleep(100);
}

void GeneralController::laserPointCloudStateCallback(const sensor_msgs::PointCloud::ConstPtr& laser){

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
		float xpos = (float)self->navSector->sites->at(goalIndex)->xpos/100;
		float ypos = (float)self->navSector->sites->at(goalIndex)->ypos/100;
		Sleep(100);
		self->goToPosition(xpos, ypos, 0.0);
		goalIndex++;
		if(goalIndex == self->navSector->sites->size()) goalIndex = 0;
		while(!self->hasAchievedGoal && self->keepTourAlive == YES) Sleep(100);
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

	float minXY = -0.5;
	float maxXY = 0.5;
	float minTh = -0.02618;
	float maxTh = 0.02618;
	float spacing = 0.1;

	fuzzy::trapezoid* xxKK = new fuzzy::trapezoid("Xx(k|k)", robotEncoderPosition(0, 0) + robotConfig->navParams->initialPosition->xZone->x1, robotEncoderPosition(0, 0) + robotConfig->navParams->initialPosition->xZone->x2, robotEncoderPosition(0, 0) + robotConfig->navParams->initialPosition->xZone->x3, robotEncoderPosition(0, 0) + robotConfig->navParams->initialPosition->xZone->x4);
	
	fuzzy::trapezoid* xyKK = new fuzzy::trapezoid("Xy(k|k)", robotEncoderPosition(1, 0) + robotConfig->navParams->initialPosition->yZone->x1, robotEncoderPosition(1, 0) + robotConfig->navParams->initialPosition->yZone->x2, robotEncoderPosition(1, 0) + robotConfig->navParams->initialPosition->yZone->x3, robotEncoderPosition(1, 0) + robotConfig->navParams->initialPosition->yZone->x4);	
	
	fuzzy::trapezoid* xThKK = new fuzzy::trapezoid("XTh(k|k)", robotEncoderPosition(2, 0) + robotConfig->navParams->initialPosition->thZone->x1, robotEncoderPosition(2, 0) + robotConfig->navParams->initialPosition->thZone->x2, robotEncoderPosition(2, 0) + robotConfig->navParams->initialPosition->thZone->x3, robotEncoderPosition(2, 0) + robotConfig->navParams->initialPosition->thZone->x4);	
	
	fuzzy::trapezoid* vxK1 = new fuzzy::trapezoid("Vx(k + 1)", robotConfig->navParams->processNoise->xZone->x1, robotConfig->navParams->processNoise->xZone->x2, robotConfig->navParams->processNoise->xZone->x3, robotConfig->navParams->processNoise->xZone->x4);
	
	fuzzy::trapezoid* vyK1 = new fuzzy::trapezoid("Vy(k + 1)", robotConfig->navParams->processNoise->yZone->x1, robotConfig->navParams->processNoise->yZone->x2, robotConfig->navParams->processNoise->yZone->x3, robotConfig->navParams->processNoise->yZone->x4);
	
	fuzzy::trapezoid* vThK1 = new fuzzy::trapezoid("VTh(k + 1)", robotConfig->navParams->processNoise->thZone->x1, robotConfig->navParams->processNoise->thZone->x2, robotConfig->navParams->processNoise->thZone->x3, robotConfig->navParams->processNoise->thZone->x4);	
	
	fuzzy::trapezoid* wdK1 = new fuzzy::trapezoid("Wd(k + 1)", robotConfig->navParams->observationNoise->dZone->x1, robotConfig->navParams->observationNoise->dZone->x2, robotConfig->navParams->observationNoise->dZone->x3, robotConfig->navParams->observationNoise->dZone->x4);
	
	fuzzy::trapezoid* wThK1 = new fuzzy::trapezoid("WTh(k + 1)", robotConfig->navParams->observationNoise->thZone->x1, robotConfig->navParams->observationNoise->thZone->x2, robotConfig->navParams->observationNoise->thZone->x3, robotConfig->navParams->observationNoise->thZone->x4);	
	
	kalmanFuzzy->push_back(xxKK);
	kalmanFuzzy->push_back(xyKK);
	kalmanFuzzy->push_back(xThKK);
	
	kalmanFuzzy->push_back(vxK1);
	kalmanFuzzy->push_back(vyK1);
	kalmanFuzzy->push_back(vThK1);
	
	kalmanFuzzy->push_back(wdK1);
	kalmanFuzzy->push_back(wThK1);

	uX = fuzzy::fstats::uncertainty(xxKK->getVertexA(), xxKK->getVertexB(), xxKK->getVertexC(), xxKK->getVertexD());
	uY = fuzzy::fstats::uncertainty(xyKK->getVertexA(), xyKK->getVertexB(), xyKK->getVertexC(), xyKK->getVertexD());
	uTh = fuzzy::fstats::uncertainty(xThKK->getVertexA(), xThKK->getVertexB(), xThKK->getVertexC(), xThKK->getVertexD());
	
	P(0, 0) = uX; 	P(0, 1) = 0; 	P(0, 2) = 0;
	P(1, 0) = 0; 	P(1, 1) = uY;	P(1, 2) = 0;
	P(2, 0) = 0;	P(2, 1) = 0;	P(2, 2) = uTh;
	
	// Variances and Covariances Matrix of Process noise Q

	uX = fuzzy::fstats::uncertainty(vxK1->getVertexA(), vxK1->getVertexB(), vxK1->getVertexC(), vxK1->getVertexD());
	uY = fuzzy::fstats::uncertainty(vyK1->getVertexA(), vyK1->getVertexB(), vyK1->getVertexC(), vyK1->getVertexD());
	uTh = fuzzy::fstats::uncertainty(vThK1->getVertexA(), vThK1->getVertexB(), vThK1->getVertexC(), vThK1->getVertexD());
	
	Q(0, 0) = uX; 	Q(0, 1) = 0; 	Q(0, 2) = 0;
	Q(1, 0) = 0; 	Q(1, 1) = uY;	Q(1, 2) = 0;
	Q(2, 0) = 0;	Q(2, 1) = 0;	Q(2, 2) = uTh;

	Matrix matXk(1, TRAP_VERTEX);
	matXk(0, 0) = xThKK->getVertexA();
	matXk(0, 1) = xThKK->getVertexB();
	matXk(0, 2) = xThKK->getVertexC();
	matXk(0, 3) = xThKK->getVertexD();

	matXk = normalizeAngles(matXk);
	xThKK->setVertexA(matXk(0, 0));
	xThKK->setVertexB(matXk(0, 1));
	xThKK->setVertexC(matXk(0, 2));
	xThKK->setVertexD(matXk(0, 3));

}

void GeneralController::trackRobot(){
	pthread_t trackThread;
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
	Matrix pk1;
	Matrix Hk;
	Matrix Pk = self->P;
	Matrix Xk;
	
	while(ros::ok() && self->keepRobotTracking == YES){
		// 1 - Prediction
		Xk = self->robotEncoderPosition;

		pk1 = Pk;
		Pk = Ak * pk1 * ~Ak;
		
		// 2 - Observation
		Hk = Matrix(2 * self->navSector->landmarks->size(), STATE_VARIABLES);
		self->R =  0.5 * Matrix::eye(2 * self->navSector->landmarks->size());

		for(int i = 0, zIndex = 0; i < self->navSector->landmarks->size(); i++, zIndex += 2){
			Hk(zIndex, 0) = -((self->navSector->landmarks->at(i)->xpos/100) - Xk(0, 0))/std::sqrt(std::pow((self->navSector->landmarks->at(i)->xpos/100) - Xk(0, 0), 2) + std::pow((self->navSector->landmarks->at(i)->ypos/100) - Xk(1, 0),2));
			Hk(zIndex, 1) = -((self->navSector->landmarks->at(i)->ypos/100) - Xk(1, 0))/std::sqrt(std::pow((self->navSector->landmarks->at(i)->xpos/100) - Xk(0, 0), 2) + std::pow((self->navSector->landmarks->at(i)->ypos/100) - Xk(1, 0),2));
			Hk(zIndex, 2) = 0;

			Hk(zIndex + 1, 0) = ((self->navSector->landmarks->at(i)->ypos/100) - Xk(1, 0))/(std::pow((self->navSector->landmarks->at(i)->xpos/100) - Xk(0, 0), 2) + std::pow((self->navSector->landmarks->at(i)->ypos/100) - Xk(1, 0),2));
			Hk(zIndex + 1, 1) = -((self->navSector->landmarks->at(i)->xpos/100) - Xk(0, 0))/(std::pow((self->navSector->landmarks->at(i)->xpos/100) - Xk(0, 0), 2) + std::pow((self->navSector->landmarks->at(i)->ypos/100) - Xk(1, 0),2));
			Hk(zIndex + 1, 2) = -1;
		}

		std::vector<fuzzy::trapezoid*> zkl = self->getObservationsTrapezoids();

		for(int i = 0; i < zkl.size(); i++){
			std::cout << "Zkl Trap " << i  << ": (";
			std::cout << zkl.at(i)->getVertexA() << ", ";
			std::cout << zkl.at(i)->getVertexB() << ", ";
			std::cout << zkl.at(i)->getVertexC() << ", ";
			std::cout << zkl.at(i)->getVertexD() << ")" << std::endl;
		}
										
		Matrix Sk = Hk * Pk * ~Hk + self->R;
		Matrix Wk = Pk * ~Hk * !Sk;		
		
		Matrix yk(2 * self->navSector->landmarks->size(), TRAP_VERTEX);
		// 3 - Matching
		std::cout << "Seen landmarks: " << self->landmarks.size() << std::endl;
		for (int i = 0; i < self->landmarks.size(); i++){
			Matrix l = self->landmarks.at(i);
			std::cout << "landmarks {d: " << l(0, 0) << ", a: " << l(1, 0) << "}" << std::endl;

			for (int j = 0; j < zkl.size(); j+=2){
				float mdDistance = zkl.at(j)->evaluate(l(0, 0));

				Matrix tempZkl(1, TRAP_VERTEX);
				tempZkl(0, 0) = zkl.at(j+1)->getVertexA();
				tempZkl(0, 1) = zkl.at(j+1)->getVertexB();
				tempZkl(0, 2) = zkl.at(j+1)->getVertexC();
				tempZkl(0, 3) = zkl.at(j+1)->getVertexD();

				tempZkl = self->denormalizeAngles(tempZkl);

				fuzzy::trapezoid* tempTrap = new fuzzy::trapezoid("", tempZkl(0, 0), tempZkl(0, 1), tempZkl(0, 2), tempZkl(0, 3));
				float mdAngle = tempTrap->evaluate(l(1, 0));
	
				if(mdDistance >= alpha && mdAngle >= alpha){
					std::cout << "Matched landmark: {d: " << j << ", a: " << j+1 << "}" << std::endl;
					Matrix tempY(1, TRAP_VERTEX);
					tempY(0, 0) = l(0,0) - zkl.at(j)->getVertexA();
					tempY(0, 1) = l(0,0) - zkl.at(j)->getVertexB();
					tempY(0, 2) = l(0,0) - zkl.at(j)->getVertexC();
					tempY(0, 3) = l(0,0) - zkl.at(j)->getVertexD();
					tempY.sort();

					for (int k = 0; k < tempY.cols_size(); k++){
						yk(j, k) = tempY(0, k);
					}

					tempY(0, 0) = l(1,0) - tempZkl(0, 0);
					tempY(0, 1) = l(1,0) - tempZkl(0, 1);
					tempY(0, 2) = l(1,0) - tempZkl(0, 2);
					tempY(0, 3) = l(1,0) - tempZkl(0, 3);
					tempY = self->normalizeAngles(tempY);

					for (int k = 0; k < tempY.cols_size(); k++){
						yk(j+1, k) = tempY(0, k);
					}
				}
			}
		}
		// 4 - Correction
		fuzzy::trapezoid *trapX = self->kalmanFuzzy->at(X_INDEX);
		fuzzy::trapezoid *trapY = self->kalmanFuzzy->at(X_INDEX + 1);
		fuzzy::trapezoid *trapTh = self->kalmanFuzzy->at(X_INDEX + 2);
		
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

		std::cout << "trap Xk:" << std::endl << matXk;
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

		matXk = matXk + Wk * yk;
		matThk(0, 0) = matXk(2, 0);
		matThk(0, 1) = matXk(2, 1);
		matThk(0, 2) = matXk(2, 2);
		matThk(0, 3) = matXk(2, 3);

		matThk = self->normalizeAngles(matThk);

		matXk = matXk.sort_cols();

		matXk(2, 0) = matThk(0, 0);
		matXk(2, 1) = matThk(0, 1);
		matXk(2, 2) = matThk(0, 2);
		matXk(2, 3) = matThk(0, 3);

		float xkn = fuzzy::fstats::expectation(matXk(0, 0), matXk(0, 1), matXk(0, 2), matXk(0, 3));
		float ykn = fuzzy::fstats::expectation(matXk(1, 0), matXk(1, 1), matXk(1, 2), matXk(1, 3));
		float thkn = fuzzy::fstats::expectation(matThk(0, 0), matThk(0, 1), matThk(0, 2), matThk(0, 3));


		self->kalmanFuzzy->at(X_INDEX) = new fuzzy::trapezoid("Xx(k|k)", matXk(0, 0), matXk(0, 1), matXk(0, 2), matXk(0, 3));
		self->kalmanFuzzy->at(X_INDEX + 1) = new fuzzy::trapezoid("Xy(k|k)", matXk(1, 0), matXk(1, 1), matXk(1, 2), matXk(1, 3));
		self->kalmanFuzzy->at(X_INDEX + 2) = new fuzzy::trapezoid("XTh(k|k)", matThk(0, 0), matThk(0, 1), matThk(0, 2), matThk(0, 3));
		
		std::cout << "new position is: {x: " << xkn <<  ", y: " << ykn << ", theta: " << thkn << "}" << std::endl;
		Pk = (Matrix::eye(3) - Wk * Hk) * Pk;
		self->setRobotPosition(xkn, ykn, thkn);
		std::cout << "Position Xk: " << std::endl << matXk;
		self->keepRobotTracking = NO;

	}
	self->keepRobotTracking = NO;
	return NULL;
}

std::vector<fuzzy::trapezoid*> GeneralController::getObservationsTrapezoids(){
	std::vector<fuzzy::trapezoid*> result;
	
	fuzzy::trapezoid *trapX = kalmanFuzzy->at(X_INDEX);
	fuzzy::trapezoid *trapY = kalmanFuzzy->at(X_INDEX + 1);
	fuzzy::trapezoid *trapTh = kalmanFuzzy->at(X_INDEX + 2);

	Matrix zklt(2 * navSector->landmarks->size(), 16);
	int index = 0;
	float distance = 0, angle = 0;
	for(int i = 0; i < TRAP_VERTEX; i++){
		bool changed = false;
		Matrix tempZkl1(2 * navSector->landmarks->size(), 1);
		Matrix tempZkl2(2 * navSector->landmarks->size(), 1);
		float xi = 0;
		switch(i){
			case 0:
				xi = trapX->getVertexA();
				break;	
			case 1:
				xi = trapX->getVertexB();
				break;
			case 2:
				xi = trapX->getVertexC();
				break;
			case 3:
				xi = trapX->getVertexD();
				break;
		}
		if( i == 0 || i == 3){
			float yi = 0;
			for(int j = 0; j < TRAP_VERTEX; j+=3){
				switch(j){
					case 0:
						yi = trapY->getVertexA();
						break;	
					case 3:
						yi = trapY->getVertexD();
						break;
				}
				Matrix state(3);
				state(0, 0) = xi; state(1, 0) = yi; state(2, 0) = trapTh->getVertexA();
				for(int k = 0; k < navSector->landmarks->size(); k++){
					landmarkObservation(state, navSector->landmarks->at(k), distance, angle);
					tempZkl1(2*k, 0) = distance;
					tempZkl1(2*k + 1, 0) = angle;
				}
				
				state(0, 0) = xi; state(1, 0) = yi; state(2, 0) = trapTh->getVertexD();
				for(int k = 0; k < navSector->landmarks->size(); k++){
					landmarkObservation(state, navSector->landmarks->at(k), distance, angle);
					tempZkl2(2*k, 0) = distance;
					tempZkl2(2*k + 1, 0) = angle;
				}
				if(i == 0 && !changed){ 
					changed = true; 
					index = 0; 
				} else if(!changed) { 
					changed = true;
					index = 4; 
				}
				for(int j = 0; j < zklt.rows_size(); j++){
					zklt(j, index) = tempZkl1(j, 0);
					zklt(j, index + 1) = tempZkl2(j, 0);
				}
				index+=2;
				

			}
		} else {
			float yi = 0;
			for(int j = 1; j < TRAP_VERTEX; j+=2){
				switch(j){
					case 1:
						yi = trapY->getVertexB();
						break;	
					case 2:
						yi = trapY->getVertexC();
						break;
				}
				Matrix state(3);
				state(0, 0) = xi; state(1, 0) = yi; state(2, 0) = trapTh->getVertexB();
				for(int k = 0; k < navSector->landmarks->size(); k++){
					landmarkObservation(state, navSector->landmarks->at(k), distance, angle);
					tempZkl1(2*k, 0) = distance;
					tempZkl1(2*k + 1, 0) = angle;
				}
				
				state(0, 0) = xi; state(1, 0) = yi; state(2, 0) = trapTh->getVertexC();
				for(int k = 0; k < navSector->landmarks->size(); k++){
					landmarkObservation(state, navSector->landmarks->at(k), distance, angle);
					tempZkl2(2*k, 0) = distance;
					tempZkl2(2*k + 1, 0) = angle;
				}

				if(i == 1 && !changed){ 
					changed = true; 
					index = 8; 
				} else if(!changed) { 
					changed = true;
					index = 12; 
				}

				for(int j = 0; j < zklt.rows_size(); j++){
					zklt(j, index) = tempZkl1(j, 0);
					zklt(j, index + 1) = tempZkl2(j, 0);
				}
				index+=2;
			}
		}

	}

	for(int i = 0; i < zklt.rows_size(); i++){
		fuzzy::trapezoid *trapNoise;
		std::vector<float> currentFirsts;
		std::vector<float> currentSeconds;

		for(int j = 0; j < zklt.cols_size(); j++){	
			if(j < zklt.cols_size()/2){
				currentFirsts.push_back(zklt(i, j));
			} else {
				currentSeconds.push_back(zklt(i, j));	
			}
		}

		Matrix results(1, TRAP_VERTEX);
		results(0, 0) = stats::min(currentFirsts);
		results(0, 1) = stats::min(currentSeconds);
		results(0, 2) = stats::max(currentSeconds);
		results(0, 3) = stats::max(currentFirsts);
		
		if((i%2) != 0){
			results(0, 0) += kalmanFuzzy->at(W_INDEX + 1)->getVertexA();
			results(0, 1) += kalmanFuzzy->at(W_INDEX + 1)->getVertexB();
			results(0, 2) += kalmanFuzzy->at(W_INDEX + 1)->getVertexC();
			results(0, 3) += kalmanFuzzy->at(W_INDEX + 1)->getVertexD();

			results = normalizeAngles(results);

		} else {
			results(0, 0) += kalmanFuzzy->at(W_INDEX)->getVertexA();
			results(0, 1) += kalmanFuzzy->at(W_INDEX)->getVertexB();
			results(0, 2) += kalmanFuzzy->at(W_INDEX)->getVertexC();
			results(0, 3) += kalmanFuzzy->at(W_INDEX)->getVertexD();

			results = results.sort();
		}
		

		result.push_back(new fuzzy::trapezoid("", results(0, 0), results(0, 1), results(0, 2), results(0, 3)));

	}
	
	return result;
}

void GeneralController::landmarkObservation(Matrix Xk, s_landmark* landmark, float& distance, float& angle){
	distance = std::sqrt(std::pow((landmark->xpos/100) - Xk(0, 0), 2) + std::pow((landmark->ypos/100) - Xk(1, 0), 2));
	angle = std::atan2((landmark->ypos/100)- Xk(1, 0), (landmark->xpos/100) - Xk(0, 0)) - Xk(2, 0);
}

Matrix GeneralController::normalizeAngles(Matrix trap){
	Matrix tempTrap = trap;
	Matrix result(1, TRAP_VERTEX);

	for(int j = 0; j < TRAP_VERTEX; j++){
		if(tempTrap(0, j) > M_PI){
			tempTrap(0, j) = tempTrap(0, j) - 2 * M_PI;
		}
		if(tempTrap(0, j) < -M_PI){
			tempTrap(0, j) = tempTrap(0, j) + 2 * M_PI;	
		}
	}

	if(((tempTrap(0, 0) > (M_PI / 2)) && (tempTrap(0, 0) < M_PI)) &&
		((tempTrap(0, 1) > -M_PI) && (tempTrap(0, 1) < (-M_PI / 2))) &&
		((tempTrap(0, 2) > -M_PI) && (tempTrap(0, 2) < (-M_PI / 2))) &&
		((tempTrap(0, 3) > -M_PI) && (tempTrap(0, 3) < (-M_PI / 2)))){
		
		Matrix tempSort(1, 3);
		tempSort(0, 0) = tempTrap(0, 1);
		tempSort(0, 1) = tempTrap(0, 2);
		tempSort(0, 2) = tempTrap(0, 3);

		tempSort = tempSort.sort_cols();
		result(0, 0) = tempTrap(0, 0);
		result(0, 1) = tempSort(0, 0);
		result(0, 2) = tempSort(0, 1);
		result(0, 3) = tempSort(0, 2);

	} else if(((tempTrap(0, 0) > -M_PI) && (tempTrap(0, 0) < (-M_PI / 2))) &&
		((tempTrap(0, 1) > -M_PI) && (tempTrap(0, 1) < (-M_PI / 2))) &&
		((tempTrap(0, 2) > -M_PI) && (tempTrap(0, 2) < (-M_PI / 2))) &&
		((tempTrap(0, 3) < M_PI) && (tempTrap(0, 3) > (M_PI / 2)))){

		Matrix tempSort(1, 3);
		tempSort(0, 0) = tempTrap(0, 0);
		tempSort(0, 1) = tempTrap(0, 1);
		tempSort(0, 2) = tempTrap(0, 2);

		tempSort = tempSort.sort_cols();
		result(0, 0) = tempTrap(0, 0);
		result(0, 1) = tempSort(0, 0);
		result(0, 2) = tempSort(0, 1);
		result(0, 3) = tempSort(0, 2);

			
	} else if(((tempTrap(0, 0) > (M_PI / 2)) && (tempTrap(0, 0) < M_PI)) &&
		((tempTrap(0, 1) > (M_PI / 2)) && (tempTrap(0, 1) < M_PI)) &&
		((tempTrap(0, 2) > (M_PI / 2)) && (tempTrap(0, 2) < M_PI)) &&
		((tempTrap(0, 3) > -M_PI) && (tempTrap(0, 3) < (-M_PI / 2)))){
		Matrix tempSort(1, 3);
		tempSort(0, 0) = tempTrap(0, 0);
		tempSort(0, 1) = tempTrap(0, 1);
		tempSort(0, 2) = tempTrap(0, 2);

		tempSort = tempSort.sort_cols();
		result(0, 0) = tempSort(0, 0);
		result(0, 1) = tempSort(0, 1);
		result(0, 2) = tempSort(0, 2);
		result(0, 3) = tempTrap(0, 3);

	} else if(((tempTrap(0, 0) > -M_PI) && (tempTrap(0, 0) < (-M_PI / 2))) &&
		((tempTrap(0, 1) > M_PI) && (tempTrap(0, 1) < (M_PI / 2))) &&
		((tempTrap(0, 2) > M_PI) && (tempTrap(0, 2) < (M_PI / 2))) &&
		((tempTrap(0, 3) > M_PI) && (tempTrap(0, 3) < (M_PI / 2)))){

		Matrix tempSort(1, 3);
		tempSort(0, 0) = tempTrap(0, 1);
		tempSort(0, 1) = tempTrap(0, 2);
		tempSort(0, 2) = tempTrap(0, 3);

		tempSort = tempSort.sort_cols();
		result(0, 0) = tempSort(0, 0);
		result(0, 1) = tempSort(0, 1);
		result(0, 2) = tempSort(0, 2);
		result(0, 3) = tempTrap(0, 0);
			
	} else if(((tempTrap(0, 0) > -M_PI) && (tempTrap(0, 0) < (-M_PI / 2))) &&
		((tempTrap(0, 1) > -M_PI) && (tempTrap(0, 1) < (-M_PI / 2))) &&
		((tempTrap(0, 2) > M_PI) && (tempTrap(0, 2) < (M_PI / 2))) &&
		((tempTrap(0, 3) > M_PI) && (tempTrap(0, 3) < (M_PI / 2)))){

		Matrix tempSort(1, 2);
		tempSort(0, 0) = tempTrap(0, 0);
		tempSort(0, 1) = tempTrap(0, 1);
		tempSort = tempSort.sort_cols();
		result(0, 2) = tempSort(0, 0);
		result(0, 3) = tempSort(0, 1);


		tempSort(0, 0) = tempTrap(0, 2);
		tempSort(0, 1) = tempTrap(0, 3);
		tempSort = tempSort.sort_cols();
		result(0, 0) = tempSort(0, 0);
		result(0, 1) = tempSort(0, 1);


	} else if (((tempTrap(0, 0) < M_PI) && (tempTrap(0, 0) > (M_PI / 2))) &&
		((tempTrap(0, 1) < M_PI) && (tempTrap(0, 1) > (M_PI / 2))) &&
		((tempTrap(0, 2) > -M_PI) && (tempTrap(0, 2) < (-M_PI / 2))) &&
		((tempTrap(0, 3) > -M_PI) && (tempTrap(0, 3) < (-M_PI / 2)))){

		Matrix tempSort(1, 2);
		tempSort(0, 0) = tempTrap(0, 0);
		tempSort(0, 1) = tempTrap(0, 1);
		tempSort = tempSort.sort_cols();
		result(0, 0) = tempSort(0, 0);
		result(0, 1) = tempSort(0, 1);


		tempSort(0, 0) = tempTrap(0, 2);
		tempSort(0, 1) = tempTrap(0, 3);
		tempSort = tempSort.sort_cols();
		result(0, 2) = tempSort(0, 0);
		result(0, 3) = tempSort(0, 1);
	} else {
		result = tempTrap.sort_cols();
	}
	

	return result;
}

Matrix GeneralController::denormalizeAngles(Matrix trap){
	Matrix tempTrap = trap;
	Matrix result(TRAP_VERTEX, 1);

	if((tempTrap(0, 0) > 0) && (tempTrap(0, 1) < 0) && (tempTrap(0, 2) < 0) && (tempTrap(0, 3) < 0)){
		tempTrap(0, 0) = tempTrap(0, 0) - 2 * M_PI;
		result = tempTrap.sort_cols();

	} else if((tempTrap(0, 0) > 0) && (tempTrap(0, 1) > 0) && (tempTrap(0, 2) < 0) && (tempTrap(0, 3) < 0)){
		tempTrap(0, 0) = tempTrap(0, 0) - 2 * M_PI;
		tempTrap(0, 1) = tempTrap(0, 1) - 2 * M_PI;
		result = tempTrap.sort_cols();

	} else if((tempTrap(0, 0) > 0) && (tempTrap(0, 1) > 0) && (tempTrap(0, 2) > 0) && (tempTrap(0, 3) < 0)){
		tempTrap(0, 3) = tempTrap(0, 3) + 2 * M_PI;
		
		result = tempTrap.sort_cols();
	} else {
		result = trap;
	}
	

	return result;
}

void GeneralController::stopRobotTracking(){
	if(keepRobotTracking == YES){
		keepRobotTracking = MAYBE;
		std::cout << "Stopping Robot Tracking" << std::endl;
		while(keepRobotTracking != NO) Sleep(100);
	}
}

void GeneralController::beginVideoStreaming(int videoDevice){
	pthread_t t1;
	stopVideoStreaming();

	try{
		vc = cv::VideoCapture(videoDevice);
	} catch(cv::Exception& e){
		std::cout << "Exception caught: " << e.what() << std::endl;
	}

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

void GeneralController::getNumberOfCamerasAvailable(int& count){
	int value = 0;
	bool firstCrash = false;
	for(int i = 0; i < 10 && !firstCrash; i++)
	{
		/*cv::VideoCapture vc(i);
		if(vc.isOpened()){
			value++;
			vc.release();
		} else{
			firstCrash = true;
		}*/
	}
	count = value;
}

void* GeneralController::streamingThread(void* object){
	GeneralController* self = (GeneralController*)object;
	self->streamingActive = YES;
	
	cv::Mat frame;
	std::vector<uchar> buff;
	std::vector<int> params = vector<int>(2);
	params[0] = CV_IMWRITE_JPEG_QUALITY;
	params[1] = 95;
	
	UDPClient* udp_client = new UDPClient(self->getClientIPAddress(), self->udpPort);
	while(ros::ok() && self->streamingActive == YES){
		self->vc >> frame;
		cv::imencode(".jpg", frame, buff, params);
		udp_client->sendData(&buff[0], buff.size());
		Sleep(30);
	}
	udp_client->closeConnection();
	self->vc.release();
	self->streamingActive = NO;
	return NULL;
}


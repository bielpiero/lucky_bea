#include <stdlib.h>
#include <stdio.h>
#include "GeneralController.h"


const float GeneralController::LASER_MAX_RANGE = 11.6;
const float GeneralController::MIN_RAND = -1;
const float GeneralController::MAX_RAND = 1;

GeneralController::GeneralController(ros::NodeHandle nh_)
{
	this->nh = nh_;
	
	this->maestroControllers = new SerialPort();
	
	this->keepSpinning = true;
	this->bumpersOk = true;
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
	cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 1);
	pose2d_pub = nh.advertise<geometry_msgs::Pose2D>("/RosAria/pose_2D", 1);
}


GeneralController::~GeneralController(void)
{
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
	double lin_vel = 0, ang_vel = 0;
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
			break;
		case 0x12:
			stopRobotTracking();
			break;
		case 0x13:
			getPositions(cad, x, y, theta);
			setRobotPosition(x, y, theta);
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
	current_number = strtok(cad, ",");

	while(current_number != NULL){
		values[index++] = atoi(current_number);
		current_number = strtok(NULL, ",");
	}
	x = values[0];
	y = values[1];
	theta = values[2];
}

void GeneralController::getCameraDevicePort(char* cad, int& device, int& port){
	char* current_number;
	int values[6];
	int index = 0;
	current_number = strtok(cad, ":");
	//
	while(current_number != NULL){
		values[index++] = atoi(current_number);
		current_number = strtok(NULL, ":");
	}
	device = values[0];
	port = values[1];
}

void GeneralController::getPololuInstruction(char* cad, unsigned char& card_id, unsigned char& servo_id, int& value){
	char* current_number;
	int values[6];
	int index = 0;
	current_number = strtok(cad, ",");
	//
	while(current_number != NULL){
		values[index++] = atoi(current_number);
		current_number = strtok(NULL, ",");
	}
	card_id = (unsigned char)values[0];
	servo_id = (unsigned char)values[1];
	value = values[2];
}

void GeneralController::getVelocities(char* cad, double& lin_vel, double& ang_vel){
	char* current_number;
	double values[6];
	int index = 0;
	current_number = strtok(cad, ",");
	//
	while(current_number != NULL){
		values[index++] = atof(current_number);
		current_number = strtok(NULL, ",");
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

void GeneralController::moveRobot(double lin_vel, double angular_vel){
	geometry_msgs::Twist twist_msg;
	
	if (bumpersOk)
	{
		twist_msg.linear.x = lin_vel;
		twist_msg.linear.y = 0;
		twist_msg.linear.z = 0;
		twist_msg.angular.x = 0;
		twist_msg.angular.y = 0;
		twist_msg.angular.z = angular_vel;
		cmd_vel_pub.publish(twist_msg);
	}
	else
	{
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

}

void GeneralController::bumperStateCallback(const rosaria::BumperState::ConstPtr& bumpers){
	char* bump = new char[256];

	bumpersOk = true;

	for (int i = 0; i < bumpers->front_bumpers.size(); i++)
	{
		if (bumpers->front_bumpers[i] && bumpersOk)
		{
			bumpersOk = false;
		}
	}

	if (bumpersOk)
	{
		for (int i = 0; i < bumpers->rear_bumpers.size(); i++)
		{
			if (bumpers->rear_bumpers[i] && bumpersOk)
			{
				bumpersOk = false;
			}
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

void GeneralController::laserScanStateCallback(const sensor_msgs::LaserScan::ConstPtr& laser){
	float range = MAX_RAND - MIN_RAND;
	float angle_min = laser->angle_min;
	float angle_max = laser->angle_max;
	float angle_increment = laser->angle_increment;
	std::vector<float> data = laser->ranges;
	std::vector<int> dataIndices = stats::findIndicesLessThan(data, LASER_MAX_RANGE);
	
	landmarks.clear();
	
	std::vector<float> dataMean;
	std::vector<float> dataAngles;
	for(int i = 1; i < dataIndices.size(); i++){
		if(((dataIndices[i] - dataIndices[i - 1]) == 1) && (i != (dataIndices.size() - 1))){
			dataMean.push_back(data[dataIndices[i - 1]]);
			dataAngles.push_back((angle_min + ((float)dataIndices[i - 1] * angle_increment)));
		} else {
			Matrix temp = Matrix(3, 1);
			dataMean.push_back(data[dataIndices[i]]);
			
			float distMean = stats::expectation(dataMean);
			float angleMean = stats::expectation(dataAngles);
			
			temp(0, 0) = distMean * cos(angleMean);
			temp(1, 0) = distMean * sin(angleMean);
			temp(2, 1) = 0;
			landmarks.push_back(temp);
			dataMean.clear();
			dataAngles.clear();
		}
	}
	Sleep(100);
}

void GeneralController::laserPointCloudStateCallback(const sensor_msgs::PointCloud::ConstPtr& laser){

}

void GeneralController::initializeKalmanVariables(){
	
	float uX, uY, uTh, depXY, depXTh, depYTh;
	
	std::vector<float> sampleXY;
	std::vector<float> sampleTh;
	
	std::vector<float> evaluatedMFX;
	std::vector<float> evaluatedMFY;
	std::vector<float> evaluatedMFTh;

	float minXY = -0.5;
	float maxXY = 0.5;
	float minTh = -0.02618;
	float maxTh = 0.02618;
	float spacing = 0.1;

	fuzzy::trapezoid* xxKK = new fuzzy::trapezoid("Xx(k|k)", -0.45, -0.35, 0.4, 0.45);
	
	fuzzy::trapezoid* xyKK = new fuzzy::trapezoid("Xy(k|k)", -0.15, -0.10, 0.10, 0.15);	
	
	fuzzy::trapezoid* xThKK = new fuzzy::trapezoid("XTh(k|k)", -0.01309, -0.007855, 0.007855, 0.01309);	
	
	fuzzy::trapezoid* vxK1 = new fuzzy::trapezoid("Vx(k + 1)", -0.09, -0.05, 0.05, 0.07);
	
	fuzzy::trapezoid* vyK1 = new fuzzy::trapezoid("Vy(k + 1)", -0.09, -0.05, 0.05, 0.07);
	
	fuzzy::trapezoid* vThK1 = new fuzzy::trapezoid("VTh(k + 1)", -0.02309, -0.001855, 0.001855, 0.002309);	
	
	fuzzy::trapezoid* wxK1 = new fuzzy::trapezoid("Wx(k + 1)", -0.09, -0.05, 0.05, 0.07);

	fuzzy::trapezoid* wyK1 = new fuzzy::trapezoid("Wy(k + 1)", -0.09, -0.05, 0.05, 0.07);
	
	fuzzy::trapezoid* wThK1 = new fuzzy::trapezoid("WTh(k + 1)", -0.02309, -0.001855, 0.001855, 0.002309);	
	
	fuzzy::trapezoid* zxK1K = new fuzzy::trapezoid("Zx(k + 1)", -0.45, -0.35, 0.4, 0.45);
	
	fuzzy::trapezoid* zyK1K = new fuzzy::trapezoid("Zy(k + 1)", -0.15, -0.10, 0.10, 0.15);	
	
	fuzzy::trapezoid* zThK1K = new fuzzy::trapezoid("ZTh(k + 1)", -0.01309, -0.007855, 0.007855, 0.01309);
	
	
	kalmanFuzzy->push_back(xxKK);
	kalmanFuzzy->push_back(xyKK);
	kalmanFuzzy->push_back(xThKK);
	
	kalmanFuzzy->push_back(vxK1);
	kalmanFuzzy->push_back(vyK1);
	kalmanFuzzy->push_back(vThK1);
	
	kalmanFuzzy->push_back(wxK1);
	kalmanFuzzy->push_back(wyK1);
	//kalmanFuzzy->push_back(wThK1);
	
	//kalmanFuzzy->push_back(zxK1K);
	//kalmanFuzzy->push_back(zyK1K);
	//kalmanFuzzy->push_back(zThK1K);	
		
	float iterations = (maxXY - minXY) / spacing;
	for (int i = 0; i <= (int)iterations; i++){
		sampleXY.push_back(minXY + ((float)i * spacing));
	}
	
	spacing = 0.01;
	iterations = (maxTh - minTh) / spacing;
	for (int i = 0; i <= (int)iterations; i++){
		sampleTh.push_back(minTh + ((float)i * spacing));
	}
	
	// Variances and Covariances Matrix of Estimation P
	evaluatedMFX = fuzzy::fstats::evaluateMF(kalmanFuzzy->at(X_INDEX), sampleXY);
	evaluatedMFY = fuzzy::fstats::evaluateMF(kalmanFuzzy->at(X_INDEX + 1), sampleXY);
	evaluatedMFTh = fuzzy::fstats::evaluateMF(kalmanFuzzy->at(X_INDEX + 2), sampleTh);
	
	uX = fuzzy::fstats::uncertainty(evaluatedMFX, sampleXY);
	uY = fuzzy::fstats::uncertainty(evaluatedMFY, sampleXY);
	uTh = fuzzy::fstats::uncertainty(evaluatedMFTh, sampleTh);
	
	depXY = fuzzy::fstats::dependency(evaluatedMFX, sampleXY, evaluatedMFY, sampleXY);

	P(0, 0) = uX; 		P(0, 1) = depXY; 	P(0, 2) = 0;
	P(1, 0) = depXY; 	P(1, 1) = uY;		P(1, 2) = 0;
	P(2, 0) = 0;		P(2, 1) = 0;		P(2, 2) = uTh;
	
	// Variances and Covariances Matrix of Process noise Q
	evaluatedMFX = fuzzy::fstats::evaluateMF(kalmanFuzzy->at(V_INDEX), sampleXY);
	evaluatedMFY = fuzzy::fstats::evaluateMF(kalmanFuzzy->at(V_INDEX + 1), sampleXY);
	evaluatedMFTh = fuzzy::fstats::evaluateMF(kalmanFuzzy->at(V_INDEX + 2), sampleTh);

	uX = fuzzy::fstats::uncertainty(evaluatedMFX, sampleXY);
	uY = fuzzy::fstats::uncertainty(evaluatedMFY, sampleXY);
	uTh = fuzzy::fstats::uncertainty(evaluatedMFTh, sampleTh);
	
	depXY = fuzzy::fstats::dependency(evaluatedMFX, sampleXY, evaluatedMFY, sampleXY);
	
	Q(0, 0) = uX; 		Q(0, 1) = depXY; 	Q(0, 2) = 0;
	Q(1, 0) = depXY; 	Q(1, 1) = uY;		Q(1, 2) = 0;
	Q(2, 0) = 0;		Q(2, 1) = 0;		Q(2, 2) = uTh;

	
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
	
	srand(time(NULL));
	
	self->keepRobotTracking = YES;
	
	Matrix Ak = Matrix::eye(3);
	Matrix pk1;
	Matrix Hk;
	Matrix zk;
	Matrix Pk = self->P;
	Matrix Xk;
	
	while(ros::ok() && self->keepRobotTracking == YES){
		// 1 - Prediction
		Xk = self->robotEncoderPosition;
		for(int i = 0; i < STATE_VARIABLES; i++){
			fuzzy::trapezoid *trapa = self->kalmanFuzzy->at(X_INDEX + i);
			fuzzy::trapezoid *trapb = self->kalmanFuzzy->at(V_INDEX + i);
			//self->kalmanFuzzy[i] = *trapb + *(Ak(i, i) * (*trapa));
		}
		std::cout << "X(k + 1|k): " << std::endl << Xk;
		pk1 = Pk;
		Pk = Ak * pk1 * Ak.transpose() + self->Q;
		std::cout << "New Pk(k+1|k): " << std::endl << Pk;
		
		// 2 - Observation
		if(self->landmarks.size() > 0){
			zk = Matrix(2 * self->landmarks.size(), 1);
			self->R =  0.001 * Matrix::eye(2 * self->landmarks.size());
			Hk = Matrix(2 * self->landmarks.size(), STATE_VARIABLES);
			
			for(int i = 0; i < self->landmarks.size(); i++){
				float noise_x = ((float)rand() / ((float)RAND_MAX/(self->MAX_RAND - self->MIN_RAND))) + self->MIN_RAND;
				float noise_y = ((float)rand() / ((float)RAND_MAX/(self->MAX_RAND - self->MIN_RAND))) + self->MIN_RAND;
								
				zk(2 * i, 0) = ((self->landmarks.at(i)(0, 0) - Xk(0, 0)) * std::cos(Xk(2, 0))) + ((self->landmarks.at(i)(1, 0) - Xk(1, 0)) * std::sin(Xk(2, 0))) + noise_x;
				zk((2 * i) + 1, 0) = ((self->landmarks.at(i)(0, 0) - Xk(0, 0)) * -std::sin(Xk(2, 0))) + ((self->landmarks.at(i)(1, 0) - Xk(1, 0)) * std::cos(Xk(2, 0))) + noise_y;
							
				
				Hk(2 * i, 0) = -std::cos(Xk(2, 0));
				Hk(2 * i, 1) = -std::sin(Xk(2, 0));
				Hk(2 * i, 2) = ((self->landmarks.at(i)(0, 0) - Xk(0, 0)) * -std::sin(Xk(2, 0))) + ((self->landmarks.at(i)(1, 0) - Xk(1, 0)) * std::cos(Xk(2, 0)));
				
				Hk(2 * i + 1, 0) = std::sin(Xk(2, 0));
				Hk(2 * i + 1, 1) = -std::cos(Xk(2, 0));
				Hk(2 * i + 1, 2) = ((self->landmarks.at(i)(0, 0) - Xk(0, 0)) * -std::cos(Xk(2, 0))) + ((self->landmarks.at(i)(1, 0) - Xk(1, 0)) * -std::sin(Xk(2, 0)));
			}	
		} else {
			zk = Matrix(2, 1);
			self->R = 0.001 * Matrix::eye(2);
			Hk = Matrix(2, 3);
		}
		std::cout << "Hx: " << std::endl << Hk;
		std::cout << "Matrix R: " << std::endl << self->R;
		std::cout << "Position zk: " << std::endl << zk;

		std::vector<fuzzy::trapezoid*> zkl = self->getStateTrapezoids(Hk);
		
		for(int i = 0; i < zkl.size(); i++){
			std::cout << "Zkl Trap " << i + 1 << ": (";
			std::cout << zkl.at(i)->getVertexA() << ", ";
			std::cout << zkl.at(i)->getVertexB() << ", ";
			std::cout << zkl.at(i)->getVertexC() << ", ";
			std::cout << zkl.at(i)->getVertexD() << ")" << std::endl;
		}
								
		Matrix Sk = Hk * Pk * ~Hk + self->R;
		Matrix Wk = Pk * ~Hk * !Sk;		
		
		std::cout << "Sk: " << std::endl << Sk;
		std::cout << "Wk: " << std::endl << Wk;
		
		// 3 - Matching
		//std::cout << "Evaluated Zk:";
		//std::vector<float> evaluatedMF(self->landmarks.size());
		//for(int i = 0; i < self->landmarks.size(); i++){		
			//evaluatedMF.push_back(fuzzy::fstats::evaluateMF(self->kalmanFuzzy->at(11)->getMFByIndex(0), zk(i, 0)));
			//std::cout << evaluatedMF[0] << "\t";
		//}
		//std::cout << std::endl;
		//std::cout << "Position zx: " << std::endl << zk;

		// 4 - Correction
		
		fuzzy::trapezoid *trapX = self->kalmanFuzzy->at(X_INDEX);
		fuzzy::trapezoid *trapY = self->kalmanFuzzy->at(X_INDEX + 1);
		fuzzy::trapezoid *trapTh = self->kalmanFuzzy->at(X_INDEX + 2);
		
		Matrix matXTrap(1, 4);
		matXTrap(0, 0) = trapX->getVertexA();
		matXTrap(0, 1) = trapX->getVertexB();
		matXTrap(0, 2) = trapX->getVertexC();
		matXTrap(0, 3) = trapX->getVertexD();
		
		Matrix matYTrap(1, 4);
		matYTrap(0, 0) = trapY->getVertexA();
		matYTrap(0, 1) = trapY->getVertexB();
		matYTrap(0, 2) = trapY->getVertexC();
		matYTrap(0, 3) = trapY->getVertexD();
		
		Matrix matThTrap(1, 4);
		matThTrap(0, 0) = trapTh->getVertexA();
		matThTrap(0, 1) = trapTh->getVertexB();
		matThTrap(0, 2) = trapTh->getVertexC();
		matThTrap(0, 3) = trapTh->getVertexD();
		
		for(int i = 0; i < zkl.size(); i++){
			Matrix zklt(1, 4);
			zklt(0, 0) = zkl.at(i)->getVertexA();
			zklt(0, 1) = zkl.at(i)->getVertexB();
			zklt(0, 2) = zkl.at(i)->getVertexC();
			zklt(0, 3) = zkl.at(i)->getVertexD();
			Matrix yk = zk(i, 0) - zklt;
			matXTrap = matXTrap + Wk * yk;
		}
		Matrix yk;
		
		//state(0, 0) = xi; state(1, 0) = yi; state(2, 0) = trapTh->getVertexA();
		Xk = Xk + Wk * yk;
		Pk = (Matrix::eye(3) - Wk * Hk) * Pk;
		std::cout << "New Pk(k+1|k+1): " << std::endl << Pk;
		//self->setRobotPosition(Xk);
		std::cout << "Position Xk: " << std::endl << Xk;

	}
	self->keepRobotTracking = NO;
	return NULL;
}

std::vector<fuzzy::trapezoid*> GeneralController::getStateTrapezoids(Matrix m){
	std::vector<fuzzy::trapezoid*> result;
	
	fuzzy::trapezoid *trapX = kalmanFuzzy->at(X_INDEX);
	fuzzy::trapezoid *trapY = kalmanFuzzy->at(X_INDEX + 1);
	fuzzy::trapezoid *trapTh = kalmanFuzzy->at(X_INDEX + 2);
	Matrix zklt(m.rows_size(), 16);
	int index = 0;
	for(int i = 0; i < TRAP_VERTEX; i++){
		Matrix tempZkl1;
		Matrix tempZkl2;
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
				tempZkl1 = m * state;
				
				state(0, 0) = xi; state(1, 0) = yi; state(2, 0) = trapTh->getVertexD();
				tempZkl2 = m * state;
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
				tempZkl1 = m * state;
				
				state(0, 0) = xi; state(1, 0) = yi; state(2, 0) = trapTh->getVertexC();
				tempZkl2 = m * state;
			}
		}

		for(int j = 0; j < zklt.rows_size(); j++){
			zklt(j, index) = tempZkl1(j, 0);
			zklt(j, index + 1) = tempZkl2(j, 0);
		}
		index+=2;
	}


	for(int i = 0; i < zklt.rows_size(); i++){
		Matrix currentSorted = zklt(i).sort();

		fuzzy::trapezoid* tempTrap = new fuzzy::trapezoid("", currentSorted(0, 0), currentSorted(0, 3),
									currentSorted(0, 11), currentSorted(0, 15));
		result.push_back(tempTrap);
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

	videoCapture.open(videoDevice);
	if(videoCapture.isOpened()){
		std::cout << "Streaming from camera device: " << videoDevice << std::endl;
		pthread_create(&t1, NULL, streamingThread, (void *)(this));
	} else {
		std::cout << "Could not open device: " << videoDevice << std::endl;
	}	
	 

}

void GeneralController::stopVideoStreaming(){
	if(streamingActive == YES){
		streamingActive = MAYBE;
		std::cout << "Stopping video streaming" << std::endl;
		while(streamingActive != NO) Sleep(100);
	}
}

void GeneralController::getNumberOfCamerasAvailable(int& count){
	int value = 0;
	bool firstCrash = false;
	for(int i = 0; i < 10 && !firstCrash; i++)
	{
		cv::VideoCapture vc(i);
		if(vc.isOpened()){
			value++;
			vc.release();
		} else{
			firstCrash = true;
		}
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
	params[1] = 85;
	
	UDPClient* udp_client = new UDPClient(self->getClientIPAddress(), self->udpPort);
	while(ros::ok() && self->streamingActive == YES){
		self->videoCapture >> frame;
		cv::imencode(".jpg", frame, buff, params);
		udp_client->sendData(&buff[0], buff.size());
	}
	udp_client->closeConnection();
	self->videoCapture.release();
	self->streamingActive = NO;
	return NULL;
}


#include <stdlib.h>
#include <stdio.h>
#include "GeneralController.h"


GeneralController::GeneralController(ros::NodeHandle nh_)
{
	this->nh = nh_;
	
	this->maestroControllers = new SerialPort();
	
	this->keepSpinning = true;
	this->bumpersOk = true;
	this->streamingActive = NO;
	xmlFaceFullPath = ros::package::getPath(PACKAGE_NAME) + XML_FILE_PATH;
	cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 1);
}


GeneralController::~GeneralController(void)
{
}
void GeneralController::Define(void *_Dlg)
{
	//pDlg = _Dlg;
}
void GeneralController::OnConnection()//callback for client and server
{
	if(IsConnected())
	{
		std::cout << "Client "<< this->getClientIPAddress() << " is Connected to Doris..." << std::endl;
	}
	else
	{
		std::cout << "Disconnected..." << std::endl;
		StopDynamicGesture();
		stopVideoStreaming();
	}
}
void GeneralController::OnMsg(char* cad,int length)//callback for client and server
{
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
	
	switch (function)
	{
		case 0x00:
			std::cout << "Command 0x00. Static Faces Requested" << std::endl;
			GetGestures("0", gestures);
			SendMsg(0x00, (char*)gestures.c_str(), (int)(gestures.length())); 
			break;
		case 0x01:
			std::cout << "Command 0x01. Dynamic Faces Requested" << std::endl;
			GetGestures("1", gestures);
			SendMsg(0x01, (char*)gestures.c_str(), (int)(gestures.length()));
			break;
		case 0x02:
			std::cout << "Command 0x02. Saving New Static Face" << std::endl;
			SaveGesture(cad, 0);
			break;
		case 0x03:
			std::cout << "Command 0x03. Saving New Dynamic Face" << std::endl;
			SaveGesture(cad, 1);
			break;
		case 0x04:
			std::cout << "Command 0x04. Modifying Static Face" << std::endl;
			ModifyGesture(cad, 0);
			break;
		case 0x05:
			std::cout << "Command 0x05. Modifying Dynamic Face" << std::endl;
			ModifyGesture(cad, 1);
			break;
		case 0x06:
			std::cout << "Command 0x06. Removing Face" << std::endl;
			RemoveGesture(cad);
			break;
		case 0x07:
			std::cout << "Command 0x07. Setting Gesture Id: " << cad << std::endl;
			SetGesture(cad);
			break;
		case 0x08:
            GetPololuInstruction(cad, card_id, port, servo_position);            
			std::cout << "Command 0x08. Moving from CardId: " << (int)card_id << " Servo: " << (int)port << " To Position: " << servo_position << std::endl;
			SetServoPosition(card_id, port, servo_position);
			break;
		case 0x09:
			std::cout << "Command 0x09. Sending current positions" << std::endl;
			//SendServoPositions(servo_positions);
			//SendMsg(0x09, (char*)servo_positions.c_str(), (int)(servo_positions.length()));
			break;
		case 0x0A:
			std::cout << "Command 0x0A. Stopping any Dynamic Face" << std::endl;
			StopDynamicGesture();
			break;
		case 0x10:
			GetVelocities(cad, lin_vel, ang_vel);
			moveRobot(lin_vel, ang_vel);
			break;
		case 0x20:
			GetNumberOfCamerasAvailable(cameraCount);
			number_converter << cameraCount;
			local_buffer_out = number_converter.str();
			SendMsg(0x20, (char*)local_buffer_out.c_str(), (int)local_buffer_out.length()); 
			break;
		case 0x21:
			videoDevice = atoi(cad);
			beginVideoStreaming(videoDevice);
			break;
		case 0x22:
			stopVideoStreaming();
			break;
		default:
			std::cout << "Command Not Recognized.." << std::endl;
			break;
			
	}
	
}

void GeneralController::GetPololuInstruction(char* cad, unsigned char& card_id, unsigned char& servo_id, int& value)
{
	char* current_number;
	int values[6];
	int index = 0;
	current_number = strtok(cad, ",");
	//
	while(current_number != NULL)
	{
		values[index++] = atoi(current_number);
		current_number = strtok(NULL, ",");
	}
	card_id = (unsigned char)values[0];
	servo_id = (unsigned char)values[1];
	value = values[2];
}

void GeneralController::GetVelocities(char* cad, double& lin_vel, double& ang_vel)
{
	char* current_number;
	double values[6];
	int index = 0;
	current_number = strtok(cad, ",");
	//
	while(current_number != NULL)
	{
		values[index++] = atof(current_number);
		current_number = strtok(NULL, ",");
	}
	lin_vel = values[0];
	ang_vel = values[1];
}

void GeneralController::GetGestures(std::string type, std::string& gestures)
{
    xml_document<> doc;
    xml_node<>* root_node;
	
	std::string buffer_str = "";
	
    std::ifstream the_file(xmlFaceFullPath.c_str());
	
    std::vector<char> buffer((std::istreambuf_iterator<char>(the_file)), std::istreambuf_iterator<char>());
	buffer.push_back('\0');
	
	doc.parse<0>(&buffer[0]);
	root_node = doc.first_node(XML_STATIC_GESTURES_STR);
	if(root_node != NULL)
	{
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

void GeneralController::SaveGesture(std::string token, int type)
{
    if(type == 0){
	s_motor motor_positions[SERVOS_COUNT];
	int i = 0;
	std::string gesture_name(strtok((char*)token.c_str(), "|"));
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
	SaveStaticGesture(gesture_name, motor_positions);
    }
}

void GeneralController::ModifyGesture(std::string token, int type)
{
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
	ModifyStaticGesture(gesture_id, gesture_name, motor_positions);
    }
}

void GeneralController::ModifyStaticGesture(std::string gesture_id, std::string name, s_motor servos[])
{
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

void GeneralController::SaveStaticGesture(std::string name, s_motor servos[])
{
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

void GeneralController::SaveDynamicGesture(std::string name, s_motor servos[])
{

}

void GeneralController::RemoveGesture(std::string face_id)
{
    xml_document<> doc;
    xml_node<>* root_node;
	
	std::string buffer_str = "";
	
    std::ifstream the_file(xmlFaceFullPath.c_str());
    std::vector<char> buffer((std::istreambuf_iterator<char>(the_file)), std::istreambuf_iterator<char>());
	buffer.push_back('\0');
	
	doc.parse<0>(&buffer[0]);
	
	root_node = doc.first_node(XML_STATIC_GESTURES_STR);
	if(root_node != NULL)
	{
        xml_node<>* node_to_find;
        bool found = false;
		for (xml_node<> * gesto_node = root_node->first_node(XML_ELEMENT_GESTURE_STR); gesto_node && !found; gesto_node = gesto_node->next_sibling()){	

			std::string id(gesto_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value());
					if(id == face_id)
					{
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

void GeneralController::SetGesture(std::string face_id)
{
    xml_document<> doc;
    xml_node<>* root_node;
	
	std::string buffer_str = "";
	
    std::ifstream the_file(xmlFaceFullPath.c_str());
    std::vector<char> buffer((std::istreambuf_iterator<char>(the_file)), std::istreambuf_iterator<char>());
	buffer.push_back('\0');
	
	doc.parse<0>(&buffer[0]);
	StopDynamicGesture();
	root_node = doc.first_node(XML_STATIC_GESTURES_STR);
	if(root_node != NULL)
	{
		for (xml_node<> * gesto_node = root_node->first_node(XML_ELEMENT_GESTURE_STR); gesto_node; gesto_node = gesto_node->next_sibling()){

			std::string id(gesto_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value());

			std::string tipo(gesto_node->first_attribute(XML_ATTRIBUTE_TYPE_STR)->value());
			
			if (face_id == id)
			{
				for(xml_node<> * motor_node = gesto_node->first_node(XML_ELEMENT_MOTOR_STR); motor_node; motor_node = motor_node->next_sibling())
				{
									unsigned char card_id = (unsigned char)atoi(motor_node->first_attribute(XML_ATTRIBUTE_CARD_ID_STR)->value());
					unsigned char servo_id = (unsigned char)atoi(motor_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value());
					int position = atoi(motor_node->first_attribute(XML_ATTRIBUTE_POSITION_STR)->value());
					int speed = atoi(motor_node->first_attribute(XML_ATTRIBUTE_SPEED_STR)->value());
					int acceleration = atoi(motor_node->first_attribute(XML_ATTRIBUTE_ACCELERATION_STR)->value());
					
					SetServoPosition(card_id, servo_id, position);
					SetServoSpeed(card_id, servo_id, speed);
					SetServoAcceleration(card_id, servo_id, acceleration);
				}	
					
				if(tipo == "1")
				{
					std::cout << "This is a dynamic face" << std::endl;
					dynamic_face_info *data = new dynamic_face_info;
					data->object = this;
					data->id_gesto = face_id;
					
					pthread_t t1;
					continue_dynamic_thread = true;
					
					pthread_create(&t1,NULL, GeneralController::DynamicFaceThread, (void *)(data));
				}
			}
		}
	}
	the_file.close();
}

void GeneralController::SetServoPosition(unsigned char card_id, unsigned char servo_id, int position)
{
    this->maestroControllers->setTarget(card_id, servo_id, position);
    usleep(10000);
}

void GeneralController::SetServoSpeed(unsigned char card_id, unsigned char servo_id, int speed)
{
    this->maestroControllers->setSpeed(card_id, servo_id, speed);
    usleep(10000);
}

void GeneralController::SetServoAcceleration(unsigned char card_id, unsigned char servo_id, int acceleration)
{
    this->maestroControllers->setAcceleration(card_id, servo_id, acceleration);
    usleep(10000);
}

void GeneralController::StopDynamicGesture()
{
	continue_dynamic_thread = false;
}

void* GeneralController::DynamicFaceThread(void* object)
{
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
				
				node->object->SetServoPosition(card_id, servo_id, position);
				node->object->SetServoSpeed(card_id, servo_id, speed);
				node->object->SetServoAcceleration(card_id, servo_id, acceleration);
			}
			
			usleep(1000*atoi(selected_dynamic_face.secuences[i].tsec.c_str()));
		}
	}
}

void GeneralController::moveRobot(double lin_vel, double angular_vel){
	geometry_msgs::Twist twist_msg;
	
	//keepSpinning = false;
	//Sleep(200);

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

void GeneralController::bumperStateCallback(const rosaria::BumperState::ConstPtr& bumpers){

}

void GeneralController::poseStateCallback(const nav_msgs::Odometry::ConstPtr& pose){
	
}

void GeneralController::batteryStateCallback(const std_msgs::Float32::ConstPtr& battery){

}

void GeneralController::sonarStateCallback(const sensor_msgs::PointCloud::ConstPtr& sonar){

}

void GeneralController::batteryVoltageCallback(const std_msgs::Float64::ConstPtr& battery){

}

void GeneralController::beginVideoStreaming(int videoDevice)
{
	
	pthread_t t1;
	stopVideoStreaming();
	//videoCapture = cv::VideoCapture(videoDevice);
	videoCapture.open(videoDevice);
	if(videoCapture.isOpened())
	{
		std::cout << "Streaming from camera device: " << videoDevice << std::endl;
		pthread_create(&t1, NULL, streamingThread, (void *)(this));
	}
	else
	{
		std::cout << "Could not open device: " << videoDevice << std::endl;
	}	
	 

}

void GeneralController::stopVideoStreaming()
{
	if(streamingActive == YES)
	{
		streamingActive = MAYBE;
		std::cout << "Stopping video streaming" << std::endl;
		while(streamingActive != NO) Sleep(100);
	}
}

void GeneralController::GetNumberOfCamerasAvailable(int& count){
	int value = 0;
	bool firstCrash = false;
	for(int i = 0; i < 10 && !firstCrash; i++)
	{
		cv::VideoCapture vc(i);
		if(vc.isOpened())
		{
			value++;
		}
		else{
			firstCrash = true;
		}
	}
	count = value;
}

void* GeneralController::streamingThread(void* object)
{
	
	GeneralController* self = (GeneralController*)object;
	self->streamingActive = YES;
	
	
	cv::Mat frame;
	std::vector<uchar> buff;
	std::vector<int> params = vector<int>(2);
	params[0] = CV_IMWRITE_JPEG_QUALITY;
	params[1] = 90;
	
	UDPClient* udp_client = new UDPClient(self->getClientIPAddress(), 14005);
	while(ros::ok() && self->streamingActive == YES)
	{
		self->videoCapture >> frame;
		cv::imencode(".jpg", frame, buff, params);
		udp_client->sendData(&buff[0], buff.size());
	}
	udp_client->closeConnection();
	self->videoCapture.release();
	self->streamingActive = NO;
	return NULL;
}


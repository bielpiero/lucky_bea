#include "RNGesturesTask.h"

RNGesturesTask::RNGesturesTask(const GeneralController* gn, const char* name, const char* description) : RNRecurrentTask(gn, name, description){
	this->gn = (GeneralController*)gn;
	this->maestroController = this->gn->getMaestroController();
	gestures = new std::vector<FaceGesture*>();
	
    xml_document<> doc;
    xml_node<> * root_node;

    // Read the xml file into a vector
    ifstream theFile (XML_GESTURE_FILE_PATH);
    vector<char> buffer((istreambuf_iterator<char>(theFile)), istreambuf_iterator<char>());
    buffer.push_back('\0');

     // Parse the buffer using the xml file parsing library into doc
    doc.parse<0>(&buffer[0]);

    // Find our root node
    root_node = doc.first_node(XML_STATIC_GESTURES_STR);

     for (xml_node<> * Gesto_node = root_node->first_node(XML_ELEMENT_GESTURE_STR); Gesto_node; Gesto_node = Gesto_node->next_sibling()){
        FaceGesture* gesture = new FaceGesture();
        gesture->setName(std::string(Gesto_node->first_attribute(XML_ATTRIBUTE_NAME_STR)->value()));
        gesture->setId(std::string(Gesto_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value()));
        gesture->setType(std::string(Gesto_node->first_attribute(XML_ATTRIBUTE_TYPE_STR)->value()));

        for(xml_node<> * state_node = Gesto_node->first_node(XML_ELEMENT_FRAME_STR); state_node; state_node = state_node->next_sibling()){
        	FaceFrame* state = new FaceFrame();
        	state->setId(std::string(state_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value()));

	        for(xml_node<> * Motor_node = state_node->first_node(XML_ELEMENT_MOTOR_STR); Motor_node; Motor_node = Motor_node->next_sibling()){ //we store all the variables for the motors in a matrix of strings
	       		FaceMotor* motor = new FaceMotor();
	       		if(strcmp(Motor_node->first_attribute(XML_ATTRIBUTE_CARD_ID_STR)->value(), XML_FACE_LABEL) == 0){
	       			motor->setCardId(this->gn->getFaceId());
	       		} else if(strcmp(Motor_node->first_attribute(XML_ATTRIBUTE_CARD_ID_STR)->value(), XML_NECK_LABEL) == 0){
	       			motor->setCardId(this->gn->getNeckId());
	       		}
	            motor->setId(std::atoi(Motor_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value()));
	            motor->setPos(std::atoi(Motor_node->first_attribute(XML_ATTRIBUTE_POSITION_STR)->value()));
	            motor->setSpeed(std::atoi(Motor_node->first_attribute(XML_ATTRIBUTE_SPEED_STR)->value()));
	            motor->setAcceleration(std::atoi(Motor_node->first_attribute(XML_ATTRIBUTE_ACCELERATION_STR)->value()));
	            state->addMotor(motor);
	        }
	        gesture->addFrame(state);
	    }
        gestures->push_back(gesture);
    }
    theFile.close();
}

RNGesturesTask::~RNGesturesTask(){
	for (int i = 0; i <  gestures->size(); i++){
        delete  gestures->at(i);
    }
   
    delete gestures;
}

void RNGesturesTask::task(){
	// Iterate over the gestures
	if(this->gestureId != ""){
		std::ostringstream bufferOut_str;
		for (int i = 0; i < gestures->size(); i++){
	       if(gestureId == gestures->at(i)->getId()){ //compares if the name of the gesture is the one we are looking for
	            // Iterate over the motors
	       		std::string currentCardId = "";
		       		//bufferOut_str << "{\"Gesture\":\"" << gestures->at(i)->getName() << "\",\"Id\":\"" << gestures->at(i)->getId() << "\",\"Cards\":[";
	       		for (int j = 0; j < gestures->at(i)->framesSize(); j++){
		           	for(int k = 0; k < gestures->at(i)->frameAt(j)->motorsSize(); k++){ //we store all the variables for the motors in a string matrix
		                //////////esto es lo que hay que enviar
		                uint8_t card_id = gestures->at(i)->frameAt(j)->motorAt(k)->getCardId();
		                uint8_t servo_id = gestures->at(i)->frameAt(j)->motorAt(k)->getId();
		                uint16_t position = gestures->at(i)->frameAt(j)->motorAt(k)->getPos();
		                uint16_t speed_t = gestures->at(i)->frameAt(j)->motorAt(k)->getSpeed();
		                uint16_t acc_t = gestures->at(i)->frameAt(j)->motorAt(k)->getAcceleration();
		                
						this->maestroController->setTarget(card_id, servo_id, position);
						this->maestroController->setSpeed(card_id, servo_id, speed_t);
						this->maestroController->setAcceleration(card_id, servo_id, acc_t);
		            }
		        }
	            //bufferOut_str << "]}]}";
	        }
	    }
	    this->gestureId = "";
	}
	    

}

void RNGesturesTask::setGesture(std::string gestureId){
    this->gestureId = gestureId;
}

void RNGesturesTask::getGestures(std::string& jsonResponse){

	std::ostringstream bufferOut_str;
	bufferOut_str.clear();
    
	for (int i = 0; i < gestures->size(); i++){
		
			bufferOut_str << gestures->at(i)->getId() << "," << gestures->at(i)->getName() << "|";
		
	}
	jsonResponse = bufferOut_str.str();
}


void RNGesturesTask::onKilled(){
	
}

void RNGesturesTask::saveGesture(std::string token){
    /*if(type == 0){
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
    }*/
}

void RNGesturesTask::modifyGesture(std::string token){
    /*if(type == 0){
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
    }*/
}

void RNGesturesTask::modifyStaticGesture(std::string gesture_id, std::string name, s_motor servos[]){
	/*xml_document<> doc;
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
	}  */
	     
}

void RNGesturesTask::saveStaticGesture(std::string name, s_motor servos[]){
	/*xml_document<> doc;
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
	}*/
	
}

void RNGesturesTask::removeGesture(std::string face_id){
    /*xml_document<> doc;
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
	}*/
	
}
#include "RNGesturesTask.h"

RNGesturesTask::RNGesturesTask(SerialPort* maestroController, const char* name, const char* description) : RNRecurrentTask(name, description){
	this->maestroController = maestroController;
	gestures = new std::vector<Gesture*>();
	
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
        Gesture* gesture = new Gesture();
        gesture->setName(std::string(Gesto_node->first_attribute(XML_ATTRIBUTE_NAME_STR)->value()));
        gesture->setId(std::string(Gesto_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value()));
        gesture->setType(std::string(Gesto_node->first_attribute(XML_ATTRIBUTE_TYPE_STR)->value()));


        for(xml_node<> * Motor_node = Gesto_node -> first_node(XML_ELEMENT_MOTOR_STR); Motor_node; Motor_node = Motor_node->next_sibling()){ //we store all the variables for the motors in a matrix of strings
       		Motor* motor = new Motor();
            motor->setCardId(std::string(Motor_node->first_attribute(XML_ATTRIBUTE_CARD_ID_STR)->value()));
            motor->setId(std::string(Motor_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value()));
            motor->setPos(std::string(Motor_node->first_attribute(XML_ATTRIBUTE_POSITION_STR)->value()));
            motor->setSpeed(std::string(Motor_node->first_attribute(XML_ATTRIBUTE_SPEED_STR)->value()));
            motor->setAcceleration(std::string(Motor_node->first_attribute(XML_ATTRIBUTE_ACCELERATION_STR)->value()));
            gesture->addMotor(motor);
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
	       		bufferOut_str << "{\"Gesture\":\"" << gestures->at(i)->getName() << "\",\"Id\":\"" << gestures->at(i)->getId() << "\",\"Cards\":[";
	           	for(int j = 0; j < gestures->at(i)->motorsSize(); j++){ //we store all the variables for the motors in a string matrix
	                //////////esto es lo que hay que enviar
	                std::string cardId = gestures->at(i)->motorAt(j)->getCardId();
	                std::string idMotor = gestures->at(i)->motorAt(j)->getId();
	                std::string pos = gestures->at(i)->motorAt(j)->getPos();
	                std::string speed = gestures->at(i)->motorAt(j)->getSpeed();
	                std::string acceleration = gestures->at(i)->motorAt(j)->getAcceleration();

	                if(currentCardId != ""){
						if(currentCardId != cardId){
							currentCardId = cardId;
							bufferOut_str << "]},{\"CardId\":\"" << cardId << "\",\"Motors\":[";
						}
					} else {
						if(currentCardId != cardId){
							currentCardId = cardId;
							bufferOut_str << "{\"CardId\":\"" << cardId << "\",\"Motors\":[";
						}
					}
					bufferOut_str << "{\"MotorId\":\"" << idMotor << "\",\"Position\":\"" << pos << "\"}";
					if((j + 1) < gestures->at(i)->motorsSize()){
						std::string card_id_next = gestures->at(i)->motorAt(j + 1)->getId();
						if(card_id_next == cardId){
							bufferOut_str << ",";	
						}
					}
					unsigned char card_id = (unsigned char)cardId.at(0);
					unsigned char servo_id = (unsigned char)idMotor.at(0);
					unsigned char position = (unsigned char)pos.at(0);
					//this->maestroControllers->setTarget(card_id, servo_id, position);
					//this->maestroControllers->setSpeed(card_id, servo_id, speed);
					//this->maestroControllers->setAcceleration(card_id, servo_id, acceleration);
	            }
	            bufferOut_str << "]}]}";
	        }
	    }
	    this->gestureId = "";
	}
	    

}

void RNGesturesTask::setGesture(std::string gestureId){
    this->gestureId = gestureId;
}

void RNGesturesTask::onKilled(){
	
}
#include "RNArmTask.h"

RNArmTask::RNArmTask(const GeneralController* gn, const char* name, const char* description) : RNRecurrentTask(gn, name, description){
	this->gn = (GeneralController*)gn;
	this->maestroController = this->gn->getMaestroController();
	gestures = new std::vector<ArmGesture*>();
	
    xml_document<> doc;
    xml_node<> * root_node;

    // Read the xml file into a vector
    ifstream theFile (XML_ARM_GESTURE_FILE_PATH);
    vector<char> buffer((istreambuf_iterator<char>(theFile)), istreambuf_iterator<char>());
    buffer.push_back('\0');

     // Parse the buffer using the xml file parsing library into doc
    doc.parse<0>(&buffer[0]);

    // Find our root node
    root_node = doc.first_node(XML_STATIC_GESTURES_STR);

     for (xml_node<> * Gesto_node = root_node->first_node(XML_ELEMENT_GESTURE_STR); Gesto_node; Gesto_node = Gesto_node->next_sibling()){
        ArmGesture* gesture = new ArmGesture();
        gesture->setName(std::string(Gesto_node->first_attribute(XML_ATTRIBUTE_NAME_STR)->value()));
        gesture->setId(std::string(Gesto_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value()));
        gesture->setType(std::string(Gesto_node->first_attribute(XML_ATTRIBUTE_TYPE_STR)->value()));
        
        for(xml_node<> * state_node = Gesto_node->first_node(XML_ELEMENT_FRAME_STR); state_node; state_node = state_node->next_sibling()){
        	ArmFrame* state = new ArmFrame();
        	state->setId(std::string(state_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value()));

	        for(xml_node<> * Motor_node = state_node->first_node(XML_ELEMENT_MOTOR_STR); Motor_node; Motor_node = Motor_node->next_sibling()){ //we store all the variables for the motors in a matrix of strings
	       		ArmMotor* motor = new ArmMotor();
	            motor->setId(std::string(Motor_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value()));
	            motor->setDegrees(std::string(Motor_node->first_attribute(XML_ATTRIBUTE_DEGREES_STR)->value()));
	            motor->setDevice(std::string(Motor_node->first_attribute(XML_ATTRIBUTE_DEVICE_STR)->value()));
	            state->addMotor(motor);
	        }
	        gesture->addState(state);
	        
	    }
        gestures->push_back(gesture);
    }
    theFile.close();

    brazo = new Arm(maestroController, DEVICENAME, PROTOCOL_VERSION);
    if (brazo->openPort()){
		brazo->setPortBaudrate(BAUDRATE);
		//BAUDRATE DE LA MANO?

		//PROGRAMA
		brazo->init();
	}
}

RNArmTask::~RNArmTask(){
	for (int i = 0; i <  gestures->size(); i++){
        delete  gestures->at(i);
    }
   
    delete gestures;
    //brazo->shutdown(); 
    //brazo->closePort();
    //delete brazo;
}

void RNArmTask::task(){
	if(gestureId != ""){
		
		ArmGesture* gesture = NULL;
		for (int i = 0; i <  gestures->size(); i++){
			if(gestureId == gestures->at(i)->getId()){
				gesture = gestures->at(i);
			}
		}
		if(gesture){
			brazo->predifinedMovement(gesture);
		}
		gestureId = "";
	}
}

void RNArmTask::setGesture(std::string gestureId){
    this->gestureId = gestureId;
}

void RNArmTask::moveSingleMotor(int id, int angle){
    brazo->singleMotor(id, angle);
}

void RNArmTask::moveAllMotors(std::vector<uint16_t> motors){
    brazo->bulkControl(motors);
}

void RNArmTask::getGestures(int type, std::string& jsonResponse){

	std::ostringstream bufferOut_str;
	bufferOut_str.clear();
    
	for (int i = 0; i < gestures->size(); i++){
		//if(type == atoi(gestures->at(i)->getType().c_str())){
			bufferOut_str << gestures->at(i)->getId() << "," << gestures->at(i)->getName() << "|";
		//}
	}
	jsonResponse = bufferOut_str.str();
}
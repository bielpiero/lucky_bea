#include "RNDialogsTask.h"

RNDialogsTask::RNDialogsTask(const GeneralController* gn, const char* name, const char* description) : RNRecurrentTask(gn, name, description){
    this->gn = (GeneralController*)gn;
    srand (time(NULL));// Initialize random seed
    inputMessages = new std::vector<InputMessage*>();
    outputMessages = new std::vector<OutputMessage*>();

    this->tts = this->gn->getTTS();
    this->state = "3";
    id_input = "121"; //(XML parameter id) It initialized to help with the "121" case
    last_id_input = "200";
    xml_document<> doc;
    xml_node<> * root_node;
    // Read the xml file into a vector
    std::string fullPath = RNUtils::getApplicationPath() + XML_DIALOGS_FILE_PATH;
    ifstream theFile (fullPath.c_str());
    vector<char> buffer(( istreambuf_iterator<char>( theFile ) ), istreambuf_iterator<char>());
    buffer.push_back('\0');
    // Parse the buffer using the xml file parsing library into doc
    doc.parse<0>(&buffer[0]);
    root_node = doc.first_node("inputmessages");
    for (xml_node<> * message_node = root_node->first_node("message"); message_node; message_node = message_node->next_sibling()){
        InputMessage* msg = new InputMessage();
        msg->setId(std::string(message_node->first_attribute("id")->value()));
        msg->setType(std::string(message_node->first_attribute("type")->value()));
        msg->setLang(std::string(message_node->first_attribute("lang")->value()));
        msg->setComplete(std::string(message_node->first_attribute("compl")->value()));
        msg->setText(std::string(message_node->first_attribute("request")->value()));
        inputMessages->push_back(msg);
    }

    root_node = doc.first_node("outputmessages");
    for (xml_node<> * message_node = root_node->first_node("message"); message_node; message_node = message_node->next_sibling()){
        OutputMessage* msg = new OutputMessage();
        msg->setId(std::string(message_node->first_attribute("id")->value()));
        msg->setType(std::string(message_node->first_attribute("type")->value()));
        msg->setState(std::string(message_node->first_attribute("est")->value()));
        msg->setLang(std::string(message_node->first_attribute("lang")->value()));
        msg->setText(std::string(message_node->first_attribute("response")->value()));
        outputMessages->push_back(msg);
    }
    theFile.close();
}

RNDialogsTask::~RNDialogsTask(){
    delete tts;
    RNUtils::printLn("Deleted tts...");

    for (int i = 0; i < outputMessages->size(); i++){
        delete outputMessages->at(i);
    }
    delete outputMessages;

    for (int i = 0; i < inputMessages->size(); i++){
        delete inputMessages->at(i);
    }
    delete inputMessages;
}

void RNDialogsTask::task(){
	
    //XML parameters
    if(inputMessage != ""){
        std::string id_input_aux; //To manage the id of the possible response
        

        //Variables for when the phrase is not complete
        
        responses.clear();
        
        int coincidences = 0; //To count the number of coincidences between the input message and the xml request
        int coincidences_aux = 0; //Auxiliar variable to know if there is another phrase that has more coincidences
        
        std::string s; //Used to divide the request into words
        //REPETITION
                    
        for (int i = 0; i < inputMessages->size() and (id_input == "121"); i++){
            if( inputMessages->at(i)->getComplete() == "0" ){ //if complete is 0, the phrase is complete
                if(inputMessage == inputMessages->at(i)->getText()){ //compares the phrase we say with the requests
                    id_input = inputMessages->at(i)->getId();
                    if(last_id_input == id_input){
                        id_input = "122";
                    }
                }
            } else if(inputMessages->at(i)->getComplete() == "1" ){ //if complete is 1, the phrase is not complete
                std::string s = inputMessages->at(i)->getText();
                std::string str = std::string(inputMessage);

                std::vector<std::string> splittedRequest = RNUtils::split((char*)s.c_str(), " ");
                std::vector<std::string> splittedInputMessage = RNUtils::split((char*)str.c_str(), " ");
                coincidences = 0;

                for(int p = 0; p < splittedRequest.size(); p++){
                    for(int q = 0; q < splittedInputMessage.size(); q++){
                        if(splittedRequest.at(p) == splittedInputMessage.at(q)){
                            coincidences++;
                        }

                    }
                }

                if (coincidences == splittedRequest.size()){ //if we have found all the words of the request in the input message
                    
                    id_input_aux = inputMessages->at(i)->getId();

                    if (coincidences > coincidences_aux){ //To compare if there has been a better match
                        id_input = id_input_aux;
                        coincidences_aux = coincidences;
                    }
                }
            }
        }
        // ENVIAR id_input A GN
        gn->setTextInputIdToEmotion(id_input);
        // OUTPUT MESSAGES MANAGEMENT
        inputMessage = "";   
        
    }
               
    
}

void RNDialogsTask::setInputMessage(std::string inputMessage){
    this->inputMessage = inputMessage;
}

void RNDialogsTask::setState(std::string state){
    std::string response;
    this->state = state;
    for (int i = 0; i < outputMessages->size(); i++){
        if(id_input == outputMessages->at(i)->getId()){
            if(this->state == outputMessages->at(i)->getState()){//We search for the responses that have the state Doris is in
                responses.push_back(outputMessages->at(i));
            }
        }
    }
    //Still output messages management
    //The management of the output is different if there is only one possible answer or if there are plenty
    //if(responses.size() > 1){ //If there is more than one possible answer we use random to select one
        //We use random between the number of possible answers
    int index = rand() % responses.size();
    printf("Index Output Message %d\n", index);
    response = responses.at(index)->getText();
    tts->textToViseme(response);
    if(id_input != "122"){
        last_id_input = id_input;
    }
    id_input = "121";
        //printf("\n%s. ", response);
    //} else if(number_states == 1){ //If there is only one possible answer we just take it (it's the first one in the matrix)
    //    response = aux_response[0][3];
        //printf("\n%s. ", response);
    //} 
}
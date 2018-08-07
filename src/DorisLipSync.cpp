
#include "DorisLipSync.h"

DorisLipSync::DorisLipSync (SerialPort* mc, int faceId) {
	std::setlocale(LC_ALL, "es_ES");
	xmlLipSyncFullPath = RNUtils::getApplicationPath() + XML_FILE_LIP_SYNC_PATH;
	xmlVisemesCodesFullPath = RNUtils::getApplicationPath() + XML_FILE_VISEMES_CODES_PATH;
	this->mc = mc;
	this->faceId = faceId;
	tts = new TextToSpeech;
	tts->setDefaultConfiguration();

	gestures = new std::vector<FaceGesture*>();
	
    xml_document<> doc;
    xml_node<> * root_node;

    // Read the xml file into a vector
    ifstream theFile (xmlLipSyncFullPath.c_str());
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

	       		motor->setCardId(this->faceId);
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

char localeToLower(char c) {        
    return std::tolower(c, std::locale());    
}

std::string DorisLipSync::textNorm(const std::string str){
	std::string someString(str);
	std::transform(someString.begin(), someString.end(), someString.begin(), localeToLower);
	return someString;
}

std::string DorisLipSync::removeExtraChars(const std::string str, char symbol){
	std::string someString(str);
	someString.erase(std::remove(someString.begin(), someString.end(), symbol), someString.end());
	return someString;
}

std::string DorisLipSync::syllableToViseme(const std::string syllable){

	using namespace rapidxml;
	xml_document<> doc;
	xml_node<> * root_node;
	std::string iterViseme;
	std::string viseme = "";
	std::string actualSyllable;

	bool found = false;

	std::ifstream dorisVisemes (xmlVisemesCodesFullPath.c_str());
	vector<char> buffer((istreambuf_iterator<char>(dorisVisemes)), istreambuf_iterator<char>());
	buffer.push_back('\0');
	doc.parse<0>(&buffer[0]);

	root_node = doc.first_node(XML_ELEMENT_VISEMES_STR);

	for (xml_node<> * viseme_node = root_node->first_node(XML_ELEMENT_VISEME_STR); viseme_node and not found; viseme_node = viseme_node->next_sibling()){
		iterViseme = std::string(viseme_node->first_attribute(XML_ATTRIBUTE_NAME_STR)->value());
		viseme = iterViseme.substr(1, 2);
		actualSyllable = iterViseme.substr(3, iterViseme.length() - 2);
		found = actualSyllable == syllable;
		
	}

	return viseme;

}

void DorisLipSync::textToViseme(const std::string str){

////////////////////////////
// Comunication TTS object
	
	RNUtils::printLn("Text To Say: %s", str.c_str());
	std::string wtxt;
	wtxt.assign(str.begin(), str.end());

	std::string text = textNorm(wtxt);

	text = removeExtraChars(text, 63);
	text = removeExtraChars(text, 168);
	text = removeExtraChars(text, 33);
	text = removeExtraChars(text, 173);
	text = removeExtraChars(text, 44);
	text = removeExtraChars(text, 46);

	std::vector<std::string> syllables;
	std::string actualSyllable="";

	getSyllables(text, &syllables);
	
	//std::cout<<"<---Number of syllables--->\n"<<numSil<<std::endl;

////////////////////////////
//Text to speech
	tts->setString(str);
//__________________

	for(unsigned int i = 0; i < syllables.size(); i++){
		std::string fixedSyllable = fixSyllable(syllables.at(i));
		//std::cout << fixedSyllable << std::endl;
		actualSyllable = syllableToViseme(fixedSyllable);
		float timer = (timeSync(syllables.at(i).length(), 250));
		selectMotion(std::atoi(actualSyllable.c_str()), timer);
	}
	setViseme("5");
}

std::string DorisLipSync::fixSyllable(std::string syllable){
	std::string syl = "";
	std::locale loc("es_ES.utf8");
	for (unsigned int i = 0; i < syllable.length(); i++){
		if(syllable.at(i) == -61){
			switch(syllable.at(i + 1)){
				case -77: case -109:
					syl += 'O';
					break;
				case -83: case -115:
					syl += 'I';
					break;
				case -95: case -127:
					syl += 'A';
					break;
				case -87: case -119:
					syl += 'E';
				case -70: case -102: case -68: case -100:
					syl += 'U';
					break;
				case -79: // �
				case -111: // �
					syl += -111;
					break;
			}
			i++;
		} else {
			syl += std::toupper(syllable.at(i), loc);
		}
    	
	}

	return syl;
}

float DorisLipSync::timeSync(int numOfLetters, int actualspeakingRate){
	float sampleDuration;
	float timetoSync;

	sampleDuration = ((150 * 2.023) / (actualspeakingRate));
	timetoSync= (((sampleDuration / 19) * numOfLetters) * 1000000);

	//std::cout<<"<---NLetters:------------->\n" << numOfLetters << std::endl;
	//std::cout<<"<---Sync Time------------->\n" << timetoSync << std::endl;

	return timetoSync;
}

int DorisLipSync::speakingRate(const double& emotion){

	int selectVel = -1;

	if (emotion==0) { // Felicidad
		selectVel = 200;
	}

	if (emotion==0) {// Tristeza
		selectVel = 60;
    //75
	}

	if (emotion==0) {// Sorpresa
		selectVel = 200;
	}

	if (emotion==0) {// Furia
		selectVel = 250;
	}

	if (emotion==0) {// Disgusto
		selectVel = 250;
	}

	if (emotion==0) {// Miedo
		selectVel = 220;
	}

	if (emotion==0) {// Neutro
		selectVel = 200;
	}

	return selectVel;
}

int DorisLipSync::amplitudeWave(const double& emotion){

	int selectVol = -1;

	if (emotion==0) {// Felicidad
		selectVol = 16;
	}

	if (emotion==0) {// Tristeza
		selectVol = 10;
	}

	if (emotion==0) {// Sorpresa
		selectVol= 16;
	}

	if (emotion==0) {// Furia
		selectVol= 16;
	}

	if (emotion==0) {// Disgusto
		selectVol= 16;
	}

	if (emotion==0) {// Miedo
		selectVol = 12;
	}

	return selectVol;
}

int DorisLipSync::configureEmicTwo(const double& emotion, const std::string language){

	std::string lx;

	std::string setVoice = "N2";                /*Beautiful Betty*/
	std::string setParser = "P1";                /*P0 (DECtalk), P1 (Epson)*/

	int speaking = speakingRate(emotion);
	int audioVolume = amplitudeWave(emotion);


	if (language == ENGLISH_LANG_STR) {
		lx="L0";
	}
	
    /*L1 para espa�ol castellano*/
	if (language == SPANISH_LANG_STR) {
		lx="L2";
	}

	tts->setSpeakingRate(speaking);
	tts->setVolume(audioVolume);

	//std::cout << "<---Config EMIC 2----------->\n"<<std::endl;
	//std::cout << "Parser =  " << setParser << std::endl;
	//std::cout << "Voz =     " << setVoice << std::endl;
	//std::cout << "Lenguaje= " << lx << std::endl;
	//std::cout << "Velocidad=" << setspeakingRate <<std::endl;
	//std::cout << "Volumen = " << setAudioVolumen <<std::endl;

	    //Introducir la funcion de tranferencia de datos con libsub, una funcion por cada configuracion (Send & recive)


	return 0; // retornar el status de la tarjeta de TTS cuando responda :
}

void DorisLipSync::setViseme(std::string viseme_id){
	if(viseme_id != ""){
		std::ostringstream bufferOut_str;
		for (int i = 0; i < gestures->size(); i++){
	       if(viseme_id == gestures->at(i)->getId()){ //compares if the name of the gesture is the one we are looking for
	            // Iterate over the motors
	       		
	       		for (int j = 0; j < gestures->at(i)->framesSize(); j++){
		           	for(int k = 0; k < gestures->at(i)->frameAt(j)->motorsSize(); k++){ //we store all the variables for the motors in a string matrix
		                //////////esto es lo que hay que enviar
		                uint8_t card_id = gestures->at(i)->frameAt(j)->motorAt(k)->getCardId();
		                uint8_t servo_id = gestures->at(i)->frameAt(j)->motorAt(k)->getId();
		                uint16_t position = gestures->at(i)->frameAt(j)->motorAt(k)->getPos();
		                uint16_t speed_t = gestures->at(i)->frameAt(j)->motorAt(k)->getSpeed();
		                uint16_t acc_t = gestures->at(i)->frameAt(j)->motorAt(k)->getAcceleration();
		                
						this->mc->setTarget(card_id, servo_id, position);
						this->mc->setSpeed(card_id, servo_id, speed_t);
						this->mc->setAcceleration(card_id, servo_id, acc_t);
		            }
		        }
	        }
	    }
	}
}

void DorisLipSync::selectMotion(int visemeCod, float timetoSync){
	
	//std::cout << "<---Select Servos' Positions------>\n"<<std::endl;
	

	std::string id;
	
	switch(visemeCod)
	{
		case 1:
		id="0";
		setViseme(id);
		usleep(timetoSync);
		//std::cout<<"\n Visema A \n"<<std::endl;
		break;

		case 2:
		id="0";
		setViseme(id);
		usleep(timetoSync/2);
		id="5";
		setViseme(id);
		usleep(timetoSync/2);
		
		//std::cout<<"\n Visema A + BMPV \n"<<std::endl;

		break;

		case 3:
		id="5";
		setViseme(id);
		usleep(timetoSync/2);
		id="0";
		setViseme(id);
		usleep(timetoSync/2);
		
		//std::cout<<"\n Visema BMPV + A \n"<<std::endl;

		break;

		case 4:   	
		id="5";
		setViseme(id);
		usleep(timetoSync/2);
		id="1";
		setViseme(id);
		usleep(timetoSync/2);
		
		//std::cout<<"\n Visema BMPV + E \n"<<std::endl;      
		break;


		case 5:
		id="1";
		setViseme(id);
		usleep(timetoSync/2);
		id="0";
		setViseme(id);
		usleep(timetoSync/2);
		
		//std::cout<<"\n Visema E + A \n"<<std::endl;
		break;

		case 6:
		id="5";
		setViseme(id);
		usleep(timetoSync/2);
		id="2";
		setViseme(id);
		usleep(timetoSync/2);
		
		//std::cout<<"\n Visema BMPV + I \n"<<std::endl;
		break;

		case 7:
		id="5";
		setViseme(id);
		usleep(timetoSync/3);
		id="2";
		setViseme(id);
		usleep(timetoSync/3);
		id="0";
		setViseme(id);
		usleep(timetoSync/3);
		
		//std::cout<<"\n Visema BMPV + I + A \n"<<std::endl;

		break;

		case 8:
		id="5";
		setViseme(id);
		usleep(timetoSync/3);
		id="2";
		setViseme(id);
		usleep(timetoSync/3);
		id="1";
		setViseme(id);
		usleep(timetoSync/3);
		//std::cout<<"\n Visema BMPV + I + E \n"<<std::endl;
		break;      


		case 9:
		id="5";
		setViseme(id);
		usleep(timetoSync/3);
		id="1";
		setViseme(id);
		usleep(timetoSync/3);
		id="5";
		setViseme(id);
		usleep(timetoSync/3);
		
		//std::cout<<"\n Visema BMPV + E + BMPV \n"<<std::endl;     
		break;   

		case 10:
		id="5";
		setViseme(id);
		usleep(timetoSync/2);
		id="3";
		setViseme(id);
		usleep(timetoSync/2);
		//std::cout<<"\n Visema BMPV + O \n"<<std::endl; 
		break;    

		case 11:
		id="5";
		setViseme(id);
		usleep(timetoSync/3);
		id="3";
		setViseme(id);
		usleep(timetoSync/3);
		id="0";
		setViseme(id);
		usleep(timetoSync/3);
		
		//std::cout<<"\n Visema BMPV + 0 + A \n"<<std::endl;

		break;

		case 12:
		id="5";
		setViseme(id);
		usleep(timetoSync/3);
		id="3";
		setViseme(id);
		usleep(timetoSync/3);
		id="1";
		setViseme(id);
		usleep(timetoSync/3);
		
		//std::cout<<"\n Visema BMPV + 0 + E \n"<<std::endl;

		break;

		case 13:
		id="5";
		setViseme(id);
		usleep(timetoSync/3);
		id="3";
		setViseme(id);
		usleep(timetoSync/3);
		id="2";
		setViseme(id);
		usleep(timetoSync/3);
		
		//std::cout<<"\n Visema BMPV + 0 + I \n"<<std::endl;

		break;

		case 14:   	
		id="5";
		setViseme(id);
		usleep(timetoSync/2);
		id="4";
		setViseme(id);
		usleep(timetoSync/2);
		
		//std::cout<<"\n Visema BMPV + U \n"<<std::endl;      
		break;


		case 15:
		id="5";
		setViseme(id);
		usleep(timetoSync/3);
		id="4";
		setViseme(id);
		usleep(timetoSync/3);
		id="1";
		setViseme(id);
		usleep(timetoSync/3);
		
		//std::cout<<"\n Visema BMPV + U + E \n"<<std::endl;
		break;

		case 16:
		id="5";
		setViseme(id);
		usleep(timetoSync/3);
		id="4";
		setViseme(id);
		usleep(timetoSync/3);
		id="2";
		setViseme(id);
		usleep(timetoSync/3);
		
		//std::cout<<"\n Visema BMPV + U + I \n"<<std::endl;
		break;

		case 17:
		id="5";
		setViseme(id);
		usleep(timetoSync/3);
		id="4";
		setViseme(id);
		usleep(timetoSync/3);
		id="0";
		setViseme(id);
		usleep(timetoSync/3);
		
		//std::cout<<"\n Visema BMPV +U + I + A \n"<<std::endl;

		break;

		case 18:
		id="1";
		setViseme(id);
		usleep(timetoSync/2);
		id="0";
		setViseme(id);
		usleep(timetoSync/2);
		//std::cout<<"\n Visema E + A \n"<<std::endl;
		break;      


		case 19:
		id="1";
		setViseme(id);
		usleep(timetoSync);
		//std::cout<<"\n Visema E \n"<<std::endl;     
		break;   

		case 20:
		id="5";
		setViseme(id);
		usleep(timetoSync/2);
		id="3";
		setViseme(id);
		usleep(timetoSync/2);
		//std::cout<<"\n Visema BMPV + O \n"<<std::endl; 
		break; 


		case 21:
		id="2";
		setViseme(id);
		usleep(timetoSync/2);
		id="0";
		setViseme(id);
		usleep(timetoSync/2);

		//std::cout<<"\n Visema I + A \n"<<std::endl;

		break;

		case 22:
		id="2";
		setViseme(id);
		usleep(timetoSync/2);
		id="1";
		setViseme(id);
		usleep(timetoSync/2);

		//std::cout<<"\n Visema I + E \n"<<std::endl;

		break;

		case 23:
		id="2";
		setViseme(id);
		usleep(timetoSync/2);
		id="5";
		setViseme(id);
		usleep(timetoSync/2);

		//std::cout<<"\n Visema I + M \n"<<std::endl;

		break;

		case 24:

		id="2";
		setViseme(id);
		usleep(timetoSync/2);
		id="4";
		setViseme(id);
		usleep(timetoSync/2);

		//std::cout<<"\n Visema I + U \n"<<std::endl;     
		break;


		case 25:
		id="2";
		setViseme(id);
		usleep(timetoSync);
		//std::cout<<"\n Visema I \n"<<std::endl;
		break;

		case 26:
		id="3";
		setViseme(id);
		usleep(timetoSync/2);
		id="0";
		setViseme(id);
		usleep(timetoSync/2);

		//std::cout<<"\n Visema O + A \n"<<std::endl;  
		break;

		case 27:
		id="3";
		setViseme(id);
		usleep(timetoSync/2);
		id="1";
		setViseme(id);
		usleep(timetoSync/2);

		//std::cout<<"\n Visema O + E \n"<<std::endl;  

		break;

		case 28:
		id="3";
		setViseme(id);
		usleep(timetoSync/2);
		id="2";
		setViseme(id);
		usleep(timetoSync/2);

		//std::cout<<"\n Visema O + I \n"<<std::endl;     
		break;      


		case 29:
		id="3";
		setViseme(id);
		usleep(timetoSync/3);
		id="2";
		setViseme(id);
		usleep(timetoSync/3);
		id="0";
		setViseme(id);
		usleep(timetoSync/3);

		//std::cout<<"\n Visema O+ 1 + A \n"<<std::endl;    
		break;   

		case 30:
		id="3";
		setViseme(id);
		usleep(timetoSync/2);
		id="5";
		setViseme(id);
		usleep(timetoSync/2);
		//std::cout<<"\n Visema O + BMPV \n"<<std::endl;   
		break;       


		case 31:
		id="3";
		setViseme(id);
		usleep(timetoSync/2);

		//std::cout<<"\n Visema O \n"<<std::endl;
		break;

		case 32:
		id="4";
		setViseme(id);
		usleep(timetoSync/2);
		id="0";
		setViseme(id);
		usleep(timetoSync/2);
		//std::cout<<"\n Visema U + A \n"<<std::endl;

		break;

		case 33:
		id="4";
		setViseme(id);
		usleep(timetoSync/2);
		id="1";
		setViseme(id);
		usleep(timetoSync/2);

		//std::cout<<"\n Visema U + E \n"<<std::endl;

		break;

		case 34:

		id="4";
		setViseme(id);
		usleep(timetoSync/2);
		id="2";
		setViseme(id);
		usleep(timetoSync/2);

		//std::cout<<"\n Visema U + I \n"<<std::endl;     
		break;


		case 35:
		id="4";
		setViseme(id);
		usleep(timetoSync/3);
		id="2";
		setViseme(id);
		usleep(timetoSync/3);
		id="0";
		setViseme(id);
		usleep(timetoSync/3);
		//std::cout<<"\n Visema U+ 1 + A \n"<<std::endl; 
		break;

		case 36:
		id="4";
		setViseme(id);
		usleep(timetoSync/2);
		id="5";
		setViseme(id);
		usleep(timetoSync/2);

		//std::cout<<"\n Visema U + BMVP \n"<<std::endl;  
		break;

		case 37:
		id="5";
		setViseme(id);
		usleep(timetoSync/3);
		id="4";
		setViseme(id);
		usleep(timetoSync/3);
		id="3";
		setViseme(id);
		usleep(timetoSync/3);
		//std::cout<<"\n Visema BMVP+ U + O \n"<<std::endl; 

		break;

		case 38:
		id="4";
		setViseme(id);
		usleep(timetoSync/2);
		//std::cout<<"\n Visema U\n"<<std::endl;     
		break;      


		case 39:
		id="5";
		setViseme(id);
		usleep(timetoSync/3);
		id="2";
		setViseme(id);
		usleep(timetoSync/3);
		id="3";
		setViseme(id);
		usleep(timetoSync/3);
		//std::cout<<"\n Visema BMVP+ I + O \n"<<std::endl;     
		break;   

		case 40:
		id="5";
		setViseme(id);
		usleep(timetoSync/3);
		id="4";
		setViseme(id);
		usleep(timetoSync/3);
		id="2";
		setViseme(id);
		usleep(timetoSync/3);
		//std::cout<<"\n Visema BMVP+ U + I \n"<<std::endl;   
		break;       


		case 41:
		id="2";
		setViseme(id);
		usleep(timetoSync/2);
		id="3";
		setViseme(id);
		usleep(timetoSync/2);

		//std::cout<<"\n Visema I + O \n"<<std::endl;

		break;

		case 42:
		id="0";
		setViseme(id);
		usleep(timetoSync/2);
		id="1";
		setViseme(id);
		usleep(timetoSync/2);

		//std::cout<<"\n Visema A + E \n"<<std::endl;

		break;

		case 43:
		id="0";
		setViseme(id);
		usleep(timetoSync/2);
		id="2";
		setViseme(id);
		usleep(timetoSync/2);

		//std::cout<<"\n Visema A + I \n"<<std::endl;

		break;

		case 44:

		id="0";
		setViseme(id);
		usleep(timetoSync/2);
		id="3";
		setViseme(id);
		usleep(timetoSync/2);

		//std::cout<<"\n Visema A + O \n"<<std::endl;     
		break;


		case 45:
		id="0";
		setViseme(id);
		usleep(timetoSync/2);
		id="4";
		setViseme(id);
		usleep(timetoSync/2);
		//std::cout<<"\n Visema A + U \n"<<std::endl; 
		break;

		case 46:
		id="1";
		setViseme(id);
		usleep(timetoSync/2);
		id="2";
		setViseme(id);
		usleep(timetoSync/2);

		//std::cout<<"\n Visema E + I \n"<<std::endl;  
		break;

		case 47:
		id="1";
		setViseme(id);
		usleep(timetoSync/2);
		id="3";
		setViseme(id);
		usleep(timetoSync/2);
		//std::cout<<"\n Visema E + O \n"<<std::endl; 
		break;

		case 48:
		id="1";
		setViseme(id);
		usleep(timetoSync/2);
		id="4";
		setViseme(id);
		usleep(timetoSync/2);
		//std::cout<<"\n Visema E + U \n"<<std::endl;
		break;


		case 49:
		id="3";
		setViseme(id);
		usleep(timetoSync/2);
		id="4";
		setViseme(id);
		usleep(timetoSync/2);
		//std::cout<<"\n Visema O + U \n"<<std::endl;   
		break;   

		case 50:
		id="5";
		setViseme(id);
		usleep(timetoSync/3);
		id="1";
		setViseme(id);
		usleep(timetoSync/3);
		id="2";
		setViseme(id);
		usleep(timetoSync/3);
		//std::cout<<"\n Visema BMVP+ E + I \n"<<std::endl;   
		break; 


		case 51:
		id="5";
		setViseme(id);
		usleep(timetoSync/3);
		id="1";
		setViseme(id);
		usleep(timetoSync/3);
		id="3";
		setViseme(id);
		usleep(timetoSync/3);
		//std::cout<<"\n Visema BMVP+ E + 0 \n"<<std::endl;   
		break; 

		case 52:
		id="5";
		setViseme(id);
		usleep(timetoSync/3);
		id="1";
		setViseme(id);
		usleep(timetoSync/3);
		id="4";
		setViseme(id);
		usleep(timetoSync/3);
		//std::cout<<"\n Visema BMVP+ E + U \n"<<std::endl;   
		break; 



      //-------------CASO NEUTRO-------------------------/
		default:
		id="6";
		setViseme(id);
		usleep(timetoSync);
		//std::cout<<"enviar neutro"<<endl;
		break;
	}
}

/**************************************************************/
/* Returns an array with the start positions of the syllables */
/**************************************************************/

int DorisLipSync::getSyllables (std::string word, std::vector<std::string>* syllables) {
	std::vector<std::string> words = RNUtils::split(word, " ");

	// It looks for syllables in the word
	syllables->clear();
	for (int i = 0; i < words.size(); i++) {
		int end = 0;
		// Syllables consist of three parts: onSet, nucleus and coda
		for(int j = 0; j < words.at(i).length();){
			int start = j;
			onSet(words.at(i), start, &end);
			//std::wcout << L"set: " << words.at(i).substr(start, end - start) << std::endl;
			start = end;
			bool cont = nucleus(words.at(i), start, &end);
			//std::wcout << L"nucleus: " << words.at(i).substr(start, end - start) << std::endl;
			if(cont){
				start = end;
				if(start < words.at(i).length()){
					coda(words.at(i), start, &end);	
				}
				//std::wcout << "coda: " << words.at(i).substr(start, end - start) << std::endl;
			}
			//std::wcout << L"Syllable: " << words.at(i).substr(j, end - j) << std::endl;

			syllables->push_back(words.at(i).substr(j, end - j));
			j = end;
		}

		
	}
}

/************************************/
/* Determines whether hiatus exists */
/************************************/

bool DorisLipSync::hiatus (std::string vowels, int* size) {
	bool r = false;
	if(vowels.length() == 2){
		if((openVowel(vowels.at(0)) and openVowel(vowels.at(1))) or 
			(vowels.at(0) == 'i' and vowels.at(1) == 'i') or 
			(vowels.at(0) == 'u' and vowels.at(1) == 'u')){
			r = true;
		}
	} else if(vowels.length() == 3){
		if((openVowel(vowels.at(0)) and (vowels.at(1) == -61 and closedAcutedVowel(vowels.at(2)))) or
			((vowels.at(0) == -61 and closedAcutedVowel(vowels.at(1))) and openVowel(vowels.at(2)))){
			r = true;
		}
	}
	
	*size = vowels.length();
	return r;
}

/****************************************/
/* Determines whether triphthong exists */
/****************************************/

bool DorisLipSync::triphthong (std::string vowels, int* size){
	bool r = false;
	if(vowels.length() == 3){
		if(closedVowel(vowels.at(0)) and openVowel(vowels.at(1)) and closedVowel(vowels.at(2))){
			r = true;
		}
	} else if(vowels.length() == 4){
		if(closedVowel(vowels.at(0)) and (vowels.at(1) == -61 and openAcutedVowel(vowels.at(2))) and closedVowel(vowels.at(3))){
			r = true;
		}
	} else if(vowels.length() == 5){
		if((vowels.at(0) == -61 and (vowels.at(1) == -68 or vowels.at(1) == -100)) and (vowels.at(2) == -61 and openAcutedVowel(vowels.at(3))) and closedVowel(vowels.at(4))){
			r = true;
		}
	}

	*size = vowels.length();
	return r;
}


/********************************************************************/
/* Determines the onSet of the current syllable whose begins in pos */
/* and pos is changed to the follow position after end of onSet     */
/********************************************************************/

void DorisLipSync::onSet (std::string word, int start, int* end) {
	// Every initial consonant belongs to the onSet
	bool c1 = false, c2 = false;
	if(word.at(start) != -61){
		if(isConsonant(word.at(start)) or word.at(start) == 'y'){
			c1 = true;
			if(word.at(start) == 'g' or word.at(start) == 'k' or word.at(start) == 't' or word.at(start) == 'b' or word.at(start) == 'p' or word.at(start) == 'f'){
				if(word.at(start + 1) == 'r' or word.at(start + 1) == 'l'){
					c2 = true;
				}
			} else if(word.at(start) == 'g' or word.at(start) == 'k' or word.at(start) == 't' or word.at(start) == 'b' or word.at(start) == 'p' or word.at(start) == 'f' or word.at(start) == 'd'){
				if(word.at(start + 1) == 'r'){
					c2 = true;
				}
			} else if(word.at(start) == 'c'){
				if(word.at(start + 1) == 'h'){
					c2 = true;
				}
			} else if(word.at(start) == 'l'){
				if(word.at(start + 1) == 'l'){
					c2 = true;
				}
			} else if(word.at(start) == 'r'){
				if(word.at(start + 1) == 'r'){
					c2 = true;
				}
			}
		}
	} else if((start + 1) < word.length() and (word.at(start + 1) == -79 or word.at(start + 1) == -111)){
		c1 = true;
		c2 = true;
	}
	
	
	if(c1){
		*end = start + 1;
	}
	if(c2){
		*end = start + 2;
	}
	
}

/****************************************************************************/
/* Determines the nucleus of current syllable whose onSet ending on pos - 1 */
/* and changes pos to the follow position behind of nucleus                 */
/****************************************************************************/


bool DorisLipSync::nucleus (std::string word, int start, int* end) {
	bool r = false;
	std::string vowels;
	int i = start;
	int size;
	bool enough = false;

	while(i < word.length() and not enough){
		//std::cout << (int)word.at(i) << ", " << word.length() << std::endl;
		if(word.at(i) == -61){ // precedent for acute letter
			if((word.at(i + 1) != -79 and word.at(i + 1) != -111)){
				vowels += word.at(i);
				vowels += word.at(i + 1);
				i += 2;
			} else {
				enough = true;
			}
		} else if(not isConsonant(word.at(i))){
			vowels += word.at(i++);	
		} else {
			enough = true;
		}
	}

	if(triphthong(vowels, &size)){
		*end = *end + size;
		r = true;
	} else if(vowels.length() >= 2 and not hiatus(vowels, &size)){
		*end = *end + size;
		r = true;
	} else if(hiatus(vowels, &size)) {
		if(size > 2){
			size = 2;
		} else {
			size = 1;
		}
		*end = *end + size;
		r = false;
	} else {
		*end = *end + vowels.length();
		r = true;
	}
	
	return r;
}

/*****************************************************************************/
/* Determines the coda of the current syllable whose nucleus ends in pos - 1 */
/* and changes pos to follow position to the end of the coda                 */
/*****************************************************************************/


void DorisLipSync::coda (std::string word, int start, int* end) {
	bool incremented = false;
	//rule 6
	if(word.at(start) == 'b' or word.at(start) == 'd' or word.at(start) == 'k' or word.at(start) == 'n' or word.at(start) == 'l' or word.at(start) == 'r'){
		if((start + 1) <= (word.length() - 1)){
			if(word.at(start + 1) == 's'){
				if((start + 1) == (word.length() - 1)){
					*end = *end + 2;
					incremented = true;
				} else if((start + 2) < word.length()){
					if(openVowel(word.at(start + 2)) or closedVowel(word.at(start + 2)) or (word.at(start + 2) == -61 and (openAcutedVowel(word.at(start + 3)) or closedAcutedVowel(word.at(start + 3))))){
						*end = *end + 1;
						incremented = true;
					} else {
						*end = *end + 2;
						incremented = true;
					}
				}
			}
		}
	}

	//rule 7
	if(not incremented){
		if(word.at(start) == 's' or word.at(start) == 'z' or word.at(start) == 'x' or word.at(start) == 'c' or word.at(start) == 'm'){
			if((start + 1) < word.length()){
				if(word.at(start + 1) != -61 and isConsonant(word.at(start + 1))){
					*end = *end + 1;
					incremented = true;
				}
			} else if(start == (word.length() - 1)){
				*end = *end + 1;
				incremented = true;
			}
		} else if (word.at(start) == -61 and (word.at(start + 1) == -79 or word.at(start + 1) == -111)){
			if((start + 2) < word.length()){
				if(isConsonant(word.at(start + 2))){
					*end = *end + 2;
					incremented = true;
				}
			}
		} else if(word.at(start) == 'l'){
			if((start + 1) < word.length()){
				if(word.at(start + 1) == 'l'){
					if((start + 2) < word.length()){
						if(isConsonant(word.at(start + 2))){
							*end = *end + 2;
							incremented = true;
						} else {
							incremented = true;
						}
					}
				}
			} else if(start == (word.length() - 1)){
				*end = *end + 1;
				incremented = true;
			}
		}
	}
	
	
	//rule 8
	if(not incremented){
		if(word.at(start) == 'b' or word.at(start) == 'k'){
			if((start + 1) < word.length()){
				if(not (word.at(start + 1) == 'r' or word.at(start + 1) == 'l' or word.at(start + 1) == 's' or openVowel(word.at(start + 1)) or closedVowel(word.at(start + 1)) or word.at(start + 1))){
					*end = *end + 1;
					incremented = true;
				}
			} else if(start == (word.length() - 1)){
				*end = *end + 1;
				incremented = true;
			}
		}
	}
	
	//rule 9
	if(not incremented){
		if(word.at(start) == 'p' or word.at(start) == 'g' or word.at(start) == 'f'){
			if((start + 1) < word.length()){
				if(not (word.at(start + 1) == 'r' or word.at(start + 1) == 'l' or openVowel(word.at(start + 1)) or closedVowel(word.at(start + 1)) or (word.at(start + 1) == -61 and (openAcutedVowel(word.at(start + 2)) or closedAcutedVowel(word.at(start + 2)))))){
					*end = *end + 1;
					incremented = true;
				}
			}
		}
	}
	

	//rule 10
	if(not incremented){
		if(word.at(start) == 'd'){
			if((start + 1) < word.length()){
				if(not (word.at(start + 1) == 'r' or word.at(start + 1) == 's' or openVowel(word.at(start + 1)) or closedVowel(word.at(start + 1)) or (word.at(start + 1) == -61 and (openAcutedVowel(word.at(start + 2)) or closedAcutedVowel(word.at(start + 2)))))){
					*end = *end + 1;
					incremented = true;
				}
			} else if((start + 1) == word.length()){
				*end = *end + 1;
				incremented = true;
			}
		}
	}
	//rule 11
	if(not incremented){
		if(word.at(start) == 't'){
			if((start + 1) < word.length()){
				if(not (word.at(start + 1) == 'r' or openVowel(word.at(start + 1)) or closedVowel(word.at(start + 1)) or (word.at(start + 1) == -61 and (openAcutedVowel(word.at(start + 2)) or closedAcutedVowel(word.at(start + 2)))))){
					*end = *end + 1;
					incremented = true;
				}
			} else if((start + 1) == word.length()){
				*end = *end + 1;
				incremented = true;
			}
		}
	}

	//rule 12
	if(not incremented){
		if(word.at(start) == 'm' or word.at(start) == 'n' or word.at(start) == 'l' or word.at(start) == 'r'){
			if((start + 1) < word.length()){
				if(word.at(start) == 'r' and word.at(start + 1) == 'r'){
					incremented = true;
				} else if(not (word.at(start + 1) == 's' or openVowel(word.at(start + 1)) or closedVowel(word.at(start + 1)) or (word.at(start + 1) == -61 and (openAcutedVowel(word.at(start + 2)) or closedAcutedVowel(word.at(start + 2)))))){
					*end = *end + 1;
					incremented = true;
				}
			} else if(start == (word.length() - 1)){
				*end = *end + 1;
				incremented = true;
			}
		}
	}
	
}

/***************************************************************************/
/* Determines whether c is a open-vowel or close vowel with written accent */
/***************************************************************************/


bool DorisLipSync::openVowel (char vowel) {
	bool r = false;
	switch (vowel) {
		case 'a': case 'A':
		case 'e': case 'E':
		case 'o': case 'O':
		r = true;
		break;
		case -61: default:
		r = false;
		break;
	}
	return r;
}

bool DorisLipSync::openAcutedVowel(char vowel){
	bool r = false;
	switch (vowel) {
		case -95: case -127:		//á - Á
		case -87: case -119:		//é - É
		case -77: case -109:		//ó - Ó
		r = true;
		break;
		case -61: default:
		r = false;
		break;
	}
	return r;
}

bool DorisLipSync::closedVowel (char vowel){
	bool r = false;
	switch (vowel) {
		case 'i': case 'I':
		case 'u': case 'U':
		case 'y': case 'Y':
		r = true;
		break;
		case -61: default:
		r = false;
		break;
	}
	return r;
}

bool DorisLipSync::closedAcutedVowel(char vowel){
	bool r = false;
	switch (vowel) {
		case -83: case -115:
		case -70: case -102: case -68: case -100:
		r = true;
		break;
		case -61: default:
		r = false;
		break;
	}
	return r;
}

//**********************************/
/* Determines whether c is a vowel */
/***********************************/


bool DorisLipSync::isConsonant (char c) {
	return (not (openVowel(c) or closedVowel(c) or openAcutedVowel(c) or closedAcutedVowel(c)));
}
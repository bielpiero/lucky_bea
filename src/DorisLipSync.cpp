
#include "DorisLipSync.h"

DorisLipSync::DorisLipSync (SerialPort* mc, std::string packagePath) {
	lastWord = "";
	xmlLipSyncFullPath = packagePath + XML_FILE_LIP_SYNC_PATH;
	xmlVisemesCodesFullPath = packagePath + XML_FILE_VISEMES_CODES_PATH;
	this->mc = new SerialPort(*mc);

	tts = new TextToSpeech;
	tts->setDefaultConfiguration();
}

int DorisLipSync::numberOfSyllables (const char *word) {
	process (word);
	return numSyl;
}

std::vector<int> DorisLipSync::syllablePositions (const char *word) {
	process (std::string(word));
	return positions;
}

int DorisLipSync::stressedSyllable (const char *word) {
	process (std::string(word));
	return stressed;
}

char* DorisLipSync::cutData(char *text, int start, int stop){

	std::string actualSyllable="";


	for(int j = start; j <= stop; j++){
		actualSyllable+=text[j];}


		char *actualSyllablechar = new char[actualSyllable.length() + 1];
		strcpy(actualSyllablechar, actualSyllable.c_str());


		return actualSyllablechar;
}

char* DorisLipSync::textNorm(const char *str, char Symbol){
	std::string someString(str);
	std::transform(someString.begin(), someString.end(), someString.begin(), ::toupper);
	size_t posicion;
	while((posicion = someString.find(Symbol)) != std::string::npos){
		someString.erase(posicion, 1);}

		char *actualSyllablechar = new char[someString.length() + 1];
		strcpy(actualSyllablechar, someString.c_str());

		return actualSyllablechar;


}

char* DorisLipSync::syllableToViseme(char *syllable){

	using namespace rapidxml;
	xml_document<> doc;
	xml_node<> * root_node;
	char *iterViseme;
	char *viseme;
	char *actualSyllable;
	int compVisemeSillable;

	std::ifstream dorisVisemes (xmlVisemesCodesFullPath.c_str());
	vector<char> buffer((istreambuf_iterator<char>(dorisVisemes)), istreambuf_iterator<char>());
	buffer.push_back('\0');
	doc.parse<0>(&buffer[0]);

	root_node = doc.first_node(XML_ELEMENT_VISEMES_STR);

	for (xml_node<> * viseme_node = root_node->first_node(XML_ELEMENT_VISEME_STR); viseme_node; viseme_node = viseme_node->next_sibling())
	{
		iterViseme=viseme_node->first_attribute(XML_ATTRIBUTE_NAME_STR)->value();
		viseme = cutData(iterViseme, 1,2);
		actualSyllable = cutData(iterViseme, 3,(strlen(iterViseme)+1));
    	//std::cout <<viseme<<endl;
		compVisemeSillable = strcmp(actualSyllable, syllable);

		if (compVisemeSillable==0){
			break;
		}
	}

	return viseme;

}

void DorisLipSync::textToViseme(const char *str){

////////////////////////////
// Comunication TTS object
	
	RNUtils::printLn("Text To Say: %s", str);

	char* text = textNorm(str, '_');

	std::vector<int> pointerPositions;
	int storeSillableWeight;
	std::string actualSyllable="";
	int numSil;
	char  endText[]  = " a";
	strcat(text, endText);


	pointerPositions = syllablePositions(text);
	numSil = numberOfSyllables(text) - 1;

	//std::cout<<"<---Number of syllables--->\n"<<numSil<<std::endl;

////////////////////////////
//Text to speech
	tts->setString(str);
//__________________

	for(int i = 0; i < numSil; i++){
		storeSillableWeight = (pointerPositions.at(i + 1) - pointerPositions.at(i));
		int k = 0;


		for(int j = pointerPositions[i]; j < (storeSillableWeight + pointerPositions.at(i)); ){
			actualSyllable+=text[j];
			k++;

    // Change to char the string of the actual Syllable
			char *actualSyllablechar = new char[actualSyllable.length() + 1];
			strcpy(actualSyllablechar, actualSyllable.c_str());


			if (k == (storeSillableWeight)){
				actualSyllablechar = textNorm(actualSyllablechar,' ');
				actualSyllablechar = textNorm(actualSyllablechar,'?');
				actualSyllablechar = textNorm(actualSyllablechar,'¡');
				actualSyllablechar = textNorm(actualSyllablechar,'!');
				actualSyllablechar = textNorm(actualSyllablechar,'¿');
				actualSyllablechar = textNorm(actualSyllablechar,',');
				actualSyllablechar = textNorm(actualSyllablechar,'.');

				//std::cout << "\n\n<---------------------->\n" << std::endl;
				//std::cout << "<---Syllable----------->\n" << actualSyllablechar << std::endl;
				actualSyllablechar = syllableToViseme(actualSyllablechar);

				//std::cout << "<---Viseme Code----------->\n"<<actualSyllablechar << std::endl;


	        //Activar cuando cambie emociones
	        //char Emotion[]="Neutro"; 
	        //char *SRate = speakingRate("Neutro");
	        //char *SRateOnly=cutData(SRate, 1,4);
	        //int speakingRate = (atoi (SRateOnly));


				float timer = (timeSync(storeSillableWeight, 250));
	        //std::cout<<"<---tiempito----------->\n"<<speakingRate<<std::endl;
				selectMotion(actualSyllablechar, timer);  
			}

			j++;
		}

		actualSyllable="";

	}
	setViseme("5");
}

float  DorisLipSync::timeSync(int numOfLetters, int actualspeakingRate){
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

int DorisLipSync::configureEmicTwo(const double& emotion, const char *language){

	std::string lx;
	int selectLanguage;

	char setVoice[]=     "N2";                /*Beautiful Betty*/
	char setParser[]=    "P1";                /*P0 (DECtalk), P1 (Epson)*/

	int speaking = speakingRate(emotion);
	int audioVolume = amplitudeWave(emotion);


	selectLanguage= strcmp(language,"English");
	if (selectLanguage == 0) {
		lx="L0";
	}
	
	selectLanguage= strcmp(language, "Spanish");     /*L1 para español castellano*/
	if (selectLanguage == 0) {
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

void DorisLipSync::setViseme(string viseme_id){
	using namespace rapidxml;
	xml_document<> doc;
	xml_node<>* root_node;
	
	std::string buffer_str = "";
	std::ifstream the_file (xmlLipSyncFullPath.c_str());

	vector<char> buffer((istreambuf_iterator<char>(the_file)), istreambuf_iterator<char>());
	buffer.push_back('\0');
	
	doc.parse<0>(&buffer[0]);
	//stopDynamicGesture();
	root_node = doc.first_node(XML_STATIC_GESTURES_STR);
	if(root_node != NULL){
		for (xml_node<> * gesto_node = root_node->first_node(XML_ELEMENT_GESTURE_STR); gesto_node; gesto_node = gesto_node->next_sibling()){

			string id(gesto_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value());

			string tipo(gesto_node->first_attribute(XML_ATTRIBUTE_TYPE_STR)->value());
			
			if (viseme_id == id)
			{
				for(xml_node<> * motor_node = gesto_node->first_node(XML_ELEMENT_MOTOR_STR); motor_node; motor_node = motor_node->next_sibling()){
					unsigned char card_id = (unsigned char)atoi(motor_node->first_attribute(XML_ATTRIBUTE_CARD_ID_STR)->value());
					unsigned char servo_id = (unsigned char)atoi(motor_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value());
					int position = atoi(motor_node->first_attribute(XML_ATTRIBUTE_POSITION_STR)->value());
					//int speed = atoi(motor_node->first_attribute(XML_ATTRIBUTE_SPEED_STR)->value());
					//int acceleration = atoi(motor_node->first_attribute(XML_ATTRIBUTE_ACCELERATION_STR)->value());

					mc->setTarget(card_id, servo_id, position); 
					//std::cout << "Motor = " << servo_id<<std::endl;
					//std::cout << "Position = " << position<<std::endl;


				}	

			}
		}
	}
	the_file.close();
}

void DorisLipSync::selectMotion(char *visemeCod, float timetoSync){
	
	//std::cout << "<---Select Servos' Positions------>\n"<<std::endl;
	
	int codViseme = std::atoi (visemeCod);
	string id;
	
	switch(codViseme)
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

void DorisLipSync::process (std::string word) {
	if(word != lastWord){
		lastWord = word;
		syllablePositions (word);
	}
}

/**************************************************************/
/* Returns an array with the start positions of the syllables */
/**************************************************************/

void DorisLipSync::syllablePositions (std::string word) {
	wordLength      = word.length();
	stressedFound   = false;
	stressed        = 0;
	numSyl          = 0;
	letterAccent    = -1;

	// It looks for syllables in the word
	positions.clear();
	for (int i = 0; i < wordLength;) {
		positions.push_back(i);
		numSyl++;
		//positions [numSyl++] = i;  // It marks the beginning of the current syllable

		// Syllables consist of three parts: onSet, nucleus and coda

		onSet   (word, i);
		nucleus (word, i);
		coda    (word, i);

		if ((stressedFound) && (stressed == 0)) stressed = numSyl; // It marks the stressed syllable
	}

	// If the word has not written accent, the stressed syllable is determined
	// according to the stress rules

	if (!stressedFound) {
		if (numSyl < 2) stressed = numSyl;  // Monosyllables
		else {                              // Polysyllables
			char endLetter      = tolower(word.at(wordLength - 1));
			char previousLetter = tolower(word.at(wordLength - 2));

			if ((!isConsonant (endLetter) || (endLetter == 'y')) ||
				(((endLetter == 'n') || (endLetter == 's') || !isConsonant (previousLetter))))
				stressed = numSyl - 1;	// Stressed penultimate syllable
			else
				stressed = numSyl;		// Stressed last syllable
		}
	}
	positions.push_back(-1);
	//positions [numSyl] = -1;  // It marks the end of found syllables
}

/************************************/
/* Determines whether hiatus exists */
/************************************/


bool DorisLipSync::hiatus () {
	char accented = tolower (lastWord.at(letterAccent));

	if ((letterAccent > 1) &&  // hiatus is only possible if there is accent
		(tolower(lastWord.at(letterAccent - 1)) == 'u') &&
		(tolower(lastWord.at(letterAccent - 2)) == 'q'))
	    return false; // The 'u' letter belonging "qu" doesn't form hiatus

	// The central character of a hiatus must be a close-vowel with written accent

	if ((accented == 'í') || (accented == 'ì') || (accented == 'ú') || (accented == 'ù')) {

		if ((letterAccent > 0) && openVowel (lastWord.at(letterAccent - 1))) return true;

		if ((letterAccent < (wordLength - 1)) && openVowel (lastWord.at(letterAccent + 1))) return true;
	}

	return false;
}


/********************************************************************/
/* Determines the onSet of the current syllable whose begins in pos */
/* and pos is changed to the follow position after end of onSet     */
/********************************************************************/

void DorisLipSync::onSet (std::string pal, int &pos) {
	// Every initial consonant belongs to the onSet

	char lastConsonant = 'a';
	while ((pos < wordLength) && ((isConsonant (pal.at(pos))) && (tolower (pal.at(pos)) != 'y'))) {
		lastConsonant = tolower (pal.at(pos));
		pos++;
	}

	// (q | g) + u (example: queso, gueto)

	if (pos < wordLength - 1)
	// 
	{
		if (tolower (pal.at(pos)) == 'u') {
			if (lastConsonant == 'q') pos++;
			else
				if (lastConsonant == 'g') {
					char letter = tolower (pal.at(pos + 1));
					if ((letter == 'e') || (letter == 'é') ||
						(letter == 'i') || (letter == 'í')) pos ++;
				}
		}
		else { // The 'u' with diaeresis is added to the consonant
			if ((pal.at(pos) == 'ü') || (pal.at(pos) == 'Ü'))
				if (lastConsonant == 'g') pos++;
		}
	// eliminar llave
	}
}

/****************************************************************************/
/* Determines the nucleus of current syllable whose onSet ending on pos - 1 */
/* and changes pos to the follow position behind of nucleus                 */
/****************************************************************************/


void DorisLipSync::nucleus (std::string pal, int &pos) {
	int previous; // Saves the type of previous vowel when two vowels together exists
	              // 0 = open
	              // 1 = close with written accent
	              // 2 = close

	if (pos >= wordLength) return; // ¡¿Doesn't it have nucleus?!

	// Jumps a letter 'y' to the starting of nucleus, it is as consonant

	if (tolower(pal.at(pos)) == 'y') pos++;

	// First vowel

	if (pos < wordLength) {
		switch (pal.at(pos)) {
		// Open-vowel or close-vowel with written accent
			case 'á': case 'Á': case 'à': case 'À':
			case 'é': case 'É': case 'è': case 'È':
			case 'ó': case 'Ó': case 'ò': case 'Ò':
			letterAccent = pos;
			stressedFound   = true;
 		// Open-vowel
			case 'a': case 'A':
			case 'e': case 'E':
			case 'o': case 'O':
			previous = 0;
			pos++;
			break;
		// Close-vowel with written accent breaks some possible diphthong
			case 'í': case 'Í': case 'ì': case 'Ì':
			case 'ú': case 'Ú': case 'ù': case 'Ù': case 'ü': case 'Ü':
			letterAccent = pos;
			previous = 1;
			pos++;
			stressedFound = true;
			return;
			break;
		// Close-vowel
			case 'i': case 'I':
			case 'u': case 'U':
			previous = 2;
			pos++;
			break;
		}
	}

	// If 'h' has been inserted in the nucleus then it doesn't determine diphthong neither hiatus

	bool aitch = false;
	if (pos < wordLength) {
		if (tolower(pal.at(pos)) == 'h') {
			pos++;
			aitch = true;
		}
	}

	// Second vowel

	if (pos < wordLength) {
		switch (pal.at(pos)) {
		// Open-vowel with written accent
			case 'á': case 'Á': case 'à': case 'À':
			case 'é': case 'É': case 'è': case 'È':
			case 'ó': case 'Ó': case 'ò': case 'Ò':
			letterAccent = pos;
			if (previous != 0) {
				stressedFound    = true;
			}
		// Open-vowel
			case 'a': case 'A':
			case 'e': case 'E':
			case 'o': case 'O':
			if (previous == 0) {    // Two open-vowels don't form syllable
				if (aitch) pos--;
			return;
		}
		else {
			pos++;
		}

		break;

		// Close-vowel with written accent, can't be a triphthong, but would be a diphthong
		case 'í': case 'Í': case 'ì': case 'Ì':
		case 'ú': case 'Ú': case 'ù': case 'Ù':
		letterAccent = pos;

			if (previous != 0) {  // Diphthong
				stressedFound    = true;
				pos++;
			}
			else
				if (aitch) pos--;

			return;
		// Close-vowel
			case 'i': case 'I':
			case 'u': case 'U': case 'ü': case 'Ü':
			if (pos < wordLength - 1) { // ¿Is there a third vowel?
				if (!isConsonant (pal.at(pos+1))) {
					if (tolower(pal.at(pos - 1)) == 'h') pos--;
					return;
				}
			}

			// Two equals close-vowels don't form diphthong
			if (pal [pos] != pal.at(pos)) pos++;

			return;  // It is a descendent diphthong
			break;
		}
	}

	// Third vowel?

	if (pos < wordLength) {
		if ((tolower(pal.at(pos)) == 'i') || (tolower(pal.at(pos)) == 'u')) { // Close-vowel
			pos++;
			return;  // It is a triphthong
		}
	}
}

/*****************************************************************************/
/* Determines the coda of the current syllable whose nucleus ends in pos - 1 */
/* and changes pos to follow position to the end of the coda                 */
/*****************************************************************************/


void DorisLipSync::coda (std::string pal, int &pos) {
	if ((pos >= wordLength) || (!isConsonant (pal.at(pos))))
		return; // Syllable hasn't coda
	else {
		if (pos == wordLength - 1) // End of word
		{
			pos++;
			return;
		}

		// If there is only a consonant between vowels, it belongs to the following syllable

		if (!isConsonant (pal.at(pos + 1))) return;

		char c1 = tolower (pal.at(pos));
		char c2 = tolower (pal.at(pos + 1));

		// Has the syllable a third consecutive consonant?

		if ((pos < wordLength - 2)) {
			char c3 = tolower (pal.at(pos + 2));

			if (!isConsonant (c3)) { // There isn't third consonant
				// The groups ll, ch and rr begin a syllable

				if ((c1 == 'l') && (c2 == 'l')) return;
			if ((c1 == 'c') && (c2 == 'h')) return;
			if ((c1 == 'r') && (c2 == 'r')) return;

				// A consonant + 'h' begins a syllable, except for groups sh and rh
			if ((c1 != 's') && (c1 != 'r') &&
				(c2 == 'h'))
				return;

				// If the letter 'y' is preceded by the some
                //      letter 's', 'l', 'r', 'n' or 'c' then
				//      a new syllable begins in the previous consonant
                // else it begins in the letter 'y'

			if ((c2 == 'y')) {
				if ((c1 == 's') || (c1 == 'l') || (c1 == 'r') || (c1 == 'n') || (c1 == 'c'))
					return;

				pos++;
				return;
			}

				// groups: gl - kl - bl - vl - pl - fl - tl

			if ((((c1 == 'b')||(c1 == 'v')||(c1 == 'c')||(c1 == 'k')||
				(c1 == 'f')||(c1 == 'g')||(c1 == 'p')||(c1 == 't')) &&
				(c2 == 'l')
				)
				) {
				return;
		}

				// groups: gr - kr - dr - tr - br - vr - pr - fr

		if ((((c1 == 'b')||(c1 == 'v')||(c1 == 'c')||(c1 == 'd')||(c1 == 'k')||
			(c1 == 'f')||(c1 == 'g')||(c1 == 'p')||(c1 == 't')) &&
			(c2 == 'r')
			)
			) {
			return;
	}

	pos++;
	return;
}
			else { // There is a third consonant
				if ((pos + 3) == wordLength) { // Three consonants to the end, foreign words?
					if ((c2 == 'y')) {  // 'y' as vowel
						if ((c1 == 's') || (c1 == 'l') || (c1 == 'r') || (c1 == 'n') || (c1 == 'c'))
							return;
					}

					if (c3 == 'y') { // 'y' at the end as vowel with c2
						pos++;
				}
					else {	// Three consonants to the end, foreign words?
						pos += 3;
					}
					return;
				}

				if ((c2 == 'y')) { // 'y' as vowel
					if ((c1 == 's') || (c1 == 'l') || (c1 == 'r') || (c1 == 'n') || (c1 == 'c'))
						return;

					pos++;
					return;
				}

				// The groups pt, ct, cn, ps, mn, gn, ft, pn, cz, tz and ts begin a syllable
				// when preceded by other consonant

				if ((c2 == 'p') || (c3 == 't') ||
					(c2 == 'c') || (c3 == 't') ||
					(c2 == 'c') || (c3 == 'n') ||
					(c2 == 'p') || (c3 == 's') ||
					(c2 == 'm') || (c3 == 'n') ||
					(c2 == 'g') || (c3 == 'n') ||
					(c2 == 'f') || (c3 == 't') ||
					(c2 == 'p') || (c3 == 'n') ||
					(c2 == 'c') || (c3 == 'z') ||
					(c2 == 't') || (c3 == 's') ||
					(c2 == 't') || (c3 == 's'))
				{
					pos++;
					return;
				}

				if ((c3 == 'l') || (c3 == 'r') ||    // The consonantal groups formed by a consonant
				                                     // following the letter 'l' or 'r' cann't be
				                                     // separated and they always begin syllable
					((c2 == 'c') && (c3 == 'h')) ||  // 'ch'
					(c3 == 'y')) {                   // 'y' as vowel
					pos++;  // Following syllable begins in c2
			}
			else
					pos += 2; // c3 begins the following syllable
			}
		}
		else {
			if ((c2 == 'y')) return;

			pos +=2; // The word ends with two consonants
		}
	}
}

/***************************************************************************/
/* Determines whether c is a open-vowel or close vowel with written accent */
/***************************************************************************/


bool DorisLipSync::openVowel (char c) {
	switch (c) {
		case 'a': case 'á': case 'A': case 'Á': case 'à': case 'À':
		case 'e': case 'é': case 'E': case 'É': case 'è': case 'È':
		case 'í': case 'Í': case 'ì': case 'Ì':
		case 'o': case 'ó': case 'O': case 'Ó': case 'ò': case 'Ò':
		case 'ú': case 'Ú': case 'ù': case 'Ù':
		return true;
	}
	return false;
}

//**********************************/
/* Determines whether c is a vowel */
/***********************************/


bool DorisLipSync::isConsonant (char c) {
	switch (c) {
	// Open-vowel or close-vowel with written accent
		case 'a': case 'á': case 'A': case 'Á': case 'à': case 'À':
		case 'e': case 'é': case 'E': case 'É': case 'è': case 'È':
		case 'í': case 'Í': case 'ì': case 'Ì':
		case 'o': case 'ó': case 'O': case 'Ó': case 'ò': case 'Ò':
		case 'ú': case 'Ú': case 'ù': case 'Ù':
	// Close-vowel
		case 'i': case 'I':
		case 'u': case 'U':
		case 'ü': case 'Ü':
		return false;
		break;
	}

	return true;
}
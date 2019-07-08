#include "RNEmotionsTask.h"

RNEmotionsTask::RNEmotionsTask(const GeneralController* gn, const char* name, const char* description) : RNRecurrentTask(gn, name, description){
	this->gn = (GeneralController*)gn;
	impulses = new std::vector<Impulse*>();
    this->maestroController = this->gn->getMaestroController();
    spokenId = RN_NONE;
	currentState = new Impulse(50, 50, 50);

    xml_document<> doc;
    xml_node<> * root_node;
    
    std::string fullPath = RNUtils::getApplicationPath() + XML_IMPULSES_FILE_PATH;
    ifstream theFile (fullPath.c_str());
    vector<char> buffer(( istreambuf_iterator<char>( theFile ) ), istreambuf_iterator<char>());
    buffer.push_back('\0');
    
    doc.parse<0>(&buffer[0]);
    root_node = doc.first_node(XML_ELEMENT_IMPULSES_STR);
    for (xml_node<> * impulse_node = root_node->first_node(XML_ELEMENT_IMPULSE_STR); impulse_node; impulse_node = impulse_node->next_sibling()){
    	Impulse* imp = new Impulse();
        imp->setId(std::atoi(impulse_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value()));
        imp->setJoy(std::atof(impulse_node->first_attribute(XML_ATTRIBUTE_JOY_STR)->value()));
        imp->setFear(std::atof(impulse_node->first_attribute(XML_ATTRIBUTE_FEAR_STR)->value()));
        imp->setApproval(std::atof(impulse_node->first_attribute(XML_ATTRIBUTE_APPROVAL_STR)->value()));
        impulses->push_back(imp);
    }
    theFile.close();

    faceMotors = new FRMotorsList;

    fullPath = RNUtils::getApplicationPath() + XML_FACE_MOTORS_RANGES_FILE_PATH;
    theFile.open(fullPath);
    buffer = std::vector<char>(( istreambuf_iterator<char>( theFile ) ), istreambuf_iterator<char>());
    buffer.push_back('\0');
    doc.parse<0>(&buffer[0]);

    root_node = doc.first_node(XML_ELEMENT_FACE_STR);
    xml_node<> * node = root_node->first_node(XML_ELEMENT_MOUTH_STR);
    for (xml_node<> * motor_node = node->first_node(XML_ELEMENT_MOTOR_STR); motor_node; motor_node = motor_node->next_sibling()){
        FRMotor* mtr = new FRMotor;
        mtr->setId(std::atoi(motor_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value()));
        if(strcmp(motor_node->first_attribute(XML_ATTRIBUTE_CARD_ID_STR)->value(), XML_FACE_STR) == 0){
            mtr->setCardId(this->gn->getFaceId());
        } else if(strcmp(motor_node->first_attribute(XML_ATTRIBUTE_CARD_ID_STR)->value(), XML_NECK_STR) == 0){
            mtr->setCardId(this->gn->getNeckId());
        }
        mtr->setLow(std::atoi(motor_node->first_attribute(XML_ATTRIBUTE_LOW_STR)->value()));
        mtr->setHigh(std::atoi(motor_node->first_attribute(XML_ATTRIBUTE_HIGH_STR)->value()));
        mtr->setDescription(motor_node->first_attribute(XML_ATTRIBUTE_DESCRIPTION_STR)->value());
        faceMotors->addMouthMotor(mtr);
    }

    node = root_node->first_node(XML_ELEMENT_EYELIDS_STR);
    for (xml_node<> * motor_node = node->first_node(XML_ELEMENT_MOTOR_STR); motor_node; motor_node = motor_node->next_sibling()){
        FRMotor* mtr = new FRMotor;
        mtr->setId(std::atoi(motor_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value()));
        if(strcmp(motor_node->first_attribute(XML_ATTRIBUTE_CARD_ID_STR)->value(), XML_FACE_STR) == 0){
            mtr->setCardId(this->gn->getFaceId());
        } else if(strcmp(motor_node->first_attribute(XML_ATTRIBUTE_CARD_ID_STR)->value(), XML_NECK_STR) == 0){
            mtr->setCardId(this->gn->getNeckId());
        }
        mtr->setLow(std::atoi(motor_node->first_attribute(XML_ATTRIBUTE_LOW_STR)->value()));
        mtr->setHigh(std::atoi(motor_node->first_attribute(XML_ATTRIBUTE_HIGH_STR)->value()));
        mtr->setDescription(motor_node->first_attribute(XML_ATTRIBUTE_DESCRIPTION_STR)->value());
        faceMotors->addEyelidsMotor(mtr);
    }

    node = root_node->first_node(XML_ELEMENT_EYEBROWS_STR);
    for (xml_node<> * motor_node = node->first_node(XML_ELEMENT_MOTOR_STR); motor_node; motor_node = motor_node->next_sibling()){
        FRMotor* mtr = new FRMotor;
        mtr->setId(std::atoi(motor_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value()));
        if(strcmp(motor_node->first_attribute(XML_ATTRIBUTE_CARD_ID_STR)->value(), XML_FACE_STR) == 0){
            mtr->setCardId(this->gn->getFaceId());
        } else if(strcmp(motor_node->first_attribute(XML_ATTRIBUTE_CARD_ID_STR)->value(), XML_NECK_STR) == 0){
            mtr->setCardId(this->gn->getNeckId());
        }
        mtr->setLow(std::atoi(motor_node->first_attribute(XML_ATTRIBUTE_LOW_STR)->value()));
        mtr->setHigh(std::atoi(motor_node->first_attribute(XML_ATTRIBUTE_HIGH_STR)->value()));
        mtr->setDescription(motor_node->first_attribute(XML_ATTRIBUTE_DESCRIPTION_STR)->value());
        faceMotors->addEyebrowsMotor(mtr);
    }
    
    node = root_node->first_node(XML_ELEMENT_EYES_STR);
    for (xml_node<> * motor_node = node->first_node(XML_ELEMENT_MOTOR_STR); motor_node; motor_node = motor_node->next_sibling()){
        FRMotor* mtr = new FRMotor;
        mtr->setId(std::atoi(motor_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value()));
        if(strcmp(motor_node->first_attribute(XML_ATTRIBUTE_CARD_ID_STR)->value(), XML_FACE_STR) == 0){
            mtr->setCardId(this->gn->getFaceId());
        } else if(strcmp(motor_node->first_attribute(XML_ATTRIBUTE_CARD_ID_STR)->value(), XML_NECK_STR) == 0){
            mtr->setCardId(this->gn->getNeckId());
        }
        mtr->setLow(std::atoi(motor_node->first_attribute(XML_ATTRIBUTE_LOW_STR)->value()));
        mtr->setHigh(std::atoi(motor_node->first_attribute(XML_ATTRIBUTE_HIGH_STR)->value()));
        mtr->setDescription(motor_node->first_attribute(XML_ATTRIBUTE_DESCRIPTION_STR)->value());
        faceMotors->addEyesMotor(mtr);
    }

    node = root_node->first_node(XML_ELEMENT_NECK_STR);
    for (xml_node<> * motor_node = node->first_node(XML_ELEMENT_MOTOR_STR); motor_node; motor_node = motor_node->next_sibling()){
        FRMotor* mtr = new FRMotor;
        mtr->setId(std::atoi(motor_node->first_attribute(XML_ATTRIBUTE_ID_STR)->value()));
        if(strcmp(motor_node->first_attribute(XML_ATTRIBUTE_CARD_ID_STR)->value(), XML_FACE_STR) == 0){
            mtr->setCardId(this->gn->getFaceId());
        } else if(strcmp(motor_node->first_attribute(XML_ATTRIBUTE_CARD_ID_STR)->value(), XML_NECK_STR) == 0){
            mtr->setCardId(this->gn->getNeckId());
        }
        mtr->setLow(std::atoi(motor_node->first_attribute(XML_ATTRIBUTE_LOW_STR)->value()));
        mtr->setHigh(std::atoi(motor_node->first_attribute(XML_ATTRIBUTE_HIGH_STR)->value()));
        mtr->setDescription(motor_node->first_attribute(XML_ATTRIBUTE_DESCRIPTION_STR)->value());
        faceMotors->addNeckMotor(mtr);
    }
    
    
    theFile.close();
    initializeFuzzyEmotionSystem();
}

RNEmotionsTask::~RNEmotionsTask(){
    delete faceMotors;
}

void RNEmotionsTask::initializeFuzzyEmotionSystem(){
    emotionEngine = new fl::Engine;
    emotionEngine->setName("FuzzyEmotionSystem");
    emotionEngine->setDescription("");

    joyFS = new fl::InputVariable;
    joyFS->setName("JOY");
    joyFS->setDescription("");
    joyFS->setEnabled(true);
    joyFS->setRange(0, 100);
    //joyFS->setRange(-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
    joyFS->setLockValueInRange(false);
    joyFS->addTerm(new fl::Triangle("SAD", -std::numeric_limits<double>::infinity(), 25.000, 50.000)); 
    joyFS->addTerm(new fl::Triangle("NORMAL", 25.000, 50.000, 75.000)); 
    joyFS->addTerm(new fl::Triangle("HAPPY", 50.000, 75.000, std::numeric_limits<double>::infinity())); 
    emotionEngine->addInputVariable(joyFS);
    
    fearFS = new fl::InputVariable;
    fearFS->setName("FEAR");
    fearFS->setDescription("");
    fearFS->setEnabled(true);
    fearFS->setRange(0, 100);
    //fearFS->setRange(-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
    fearFS->setLockValueInRange(false);
    fearFS->addTerm(new fl::Triangle("AFFRAID", -std::numeric_limits<double>::infinity(), 25.000, 50.000)); 
    fearFS->addTerm(new fl::Triangle("NORMAL", 25.000, 50.000, 75.000)); 
    fearFS->addTerm(new fl::Triangle("CALM", 50.000, 75.000, std::numeric_limits<double>::infinity())); 
    emotionEngine->addInputVariable(fearFS);
    
    approvalFS = new fl::InputVariable;
    approvalFS->setName("APPROVAL");
    approvalFS->setDescription("");
    approvalFS->setEnabled(true);
    approvalFS->setRange(0, 100);
    //approvalFS->setRange(-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
    approvalFS->setLockValueInRange(false);
    approvalFS->addTerm(new fl::Triangle("DISLIKE", -std::numeric_limits<double>::infinity(), 25.000, 50.000)); 
    approvalFS->addTerm(new fl::Triangle("NORMAL", 25.000, 50.000, 75.000)); 
    approvalFS->addTerm(new fl::Triangle("LIKE", 50.000, 75.000, std::numeric_limits<double>::infinity())); 
    emotionEngine->addInputVariable(approvalFS);

    eyelidsFS = new fl::OutputVariable;
    eyelidsFS->setName("FACE_EYELIDS");
    eyelidsFS->setDescription("");
    eyelidsFS->setEnabled(true);
    eyelidsFS->setRange(-25.000, 125);
    eyelidsFS->setLockValueInRange(false);
    eyelidsFS->setAggregation(new fl::AlgebraicSum);
    eyelidsFS->setDefuzzifier(new fl::Centroid(100));
    eyelidsFS->setDefaultValue(0.0);
    eyelidsFS->setLockPreviousValue(false);
    eyelidsFS->addTerm(new fl::Triangle("CLOSED", -25.000, 0.000, 25.000)); 
    eyelidsFS->addTerm(new fl::Triangle("HALF_CLOSED", 0.000, 25.000, 50.000)); 
    eyelidsFS->addTerm(new fl::Triangle("NORMAL", 25.000, 50.000, 75.000));
    eyelidsFS->addTerm(new fl::Triangle("HALF_OPENED", 50.000, 75.000, 100.000)); 
    eyelidsFS->addTerm(new fl::Triangle("OPENED", 75.000, 100.000, 125.000)); 
    emotionEngine->addOutputVariable(eyelidsFS);
        
    mouthFS = new fl::OutputVariable;
    mouthFS->setName("FACE_MOUTH");
    mouthFS->setDescription("");
    mouthFS->setEnabled(true);
    mouthFS->setRange(-25, 125);
    mouthFS->setLockValueInRange(false);
    mouthFS->setAggregation(new fl::AlgebraicSum);
    mouthFS->setDefuzzifier(new fl::Centroid(100));
    mouthFS->setDefaultValue(0.0);
    mouthFS->setLockPreviousValue(false);
    mouthFS->addTerm(new fl::Triangle("SO_SAD", -25.000, 0.000, 25.000)); 
    mouthFS->addTerm(new fl::Triangle("SAD", 0.000, 25.000, 50.000)); 
    mouthFS->addTerm(new fl::Triangle("NORMAL", 25.000, 50.000, 75.000));
    mouthFS->addTerm(new fl::Triangle("SMILEY", 50.000, 75.000, 100.000)); 
    mouthFS->addTerm(new fl::Triangle("BIG_SMILEY", 75.000, 100.000, 125.000)); 
    emotionEngine->addOutputVariable(mouthFS);

    eyebrowsFS = new fl::OutputVariable;
    eyebrowsFS->setName("FACE_EYEBROWS");
    eyebrowsFS->setDescription("");
    eyebrowsFS->setEnabled(true);
    eyebrowsFS->setRange(0, 100);
    eyebrowsFS->setLockValueInRange(false);
    eyebrowsFS->setAggregation(new fl::AlgebraicSum);
    eyebrowsFS->setDefuzzifier(new fl::Centroid(100));
    eyebrowsFS->setDefaultValue(0.0);
    eyebrowsFS->setLockPreviousValue(false);
    eyebrowsFS->addTerm(new fl::Triangle("LOW", 0, 25.000, 50.000)); 
    eyebrowsFS->addTerm(new fl::Triangle("NORMAL", 25.000, 50.000, 75.000)); 
    eyebrowsFS->addTerm(new fl::Triangle("HIGH", 50.000, 75.000, 100)); 
    emotionEngine->addOutputVariable(eyebrowsFS);
           
    voiceRateFS = new fl::OutputVariable;
    voiceRateFS->setName("VOICE_RATE");
    voiceRateFS->setDescription("");
    voiceRateFS->setEnabled(true);
    voiceRateFS->setRange(0, 100);
    voiceRateFS->setLockValueInRange(false);
    voiceRateFS->setAggregation(new fl::AlgebraicSum);
    voiceRateFS->setDefuzzifier(new fl::Centroid(100));
    voiceRateFS->setDefaultValue(0.0);
    voiceRateFS->setLockPreviousValue(false);
    voiceRateFS->addTerm(new fl::Triangle("LOW", 0, 25.000, 50.000)); 
    voiceRateFS->addTerm(new fl::Triangle("NORMAL", 25.000, 50.000, 75.000)); 
    voiceRateFS->addTerm(new fl::Triangle("HIGH", 50.000, 75.000, 100)); 
    emotionEngine->addOutputVariable(voiceRateFS);

    responseModeFS = new fl::OutputVariable;
    responseModeFS->setName("VOICE_RESPONSE");
    responseModeFS->setDescription("");
    responseModeFS->setEnabled(true);
    responseModeFS->setRange(0, 4);
    responseModeFS->setLockValueInRange(false);
    responseModeFS->setAggregation(new fl::AlgebraicSum);
    responseModeFS->setDefuzzifier(new fl::Centroid(100));
    responseModeFS->setDefaultValue(0.0);
    responseModeFS->setLockPreviousValue(false);
    responseModeFS->addTerm(new fl::Triangle("BLUE", -1.000, 0.000, 1.000));
    responseModeFS->addTerm(new fl::Triangle("HYPOCRITE", 0.000, 1.000, 2.000));
    responseModeFS->addTerm(new fl::Triangle("NORMAL", 1.000, 2.000, 3.000));
    responseModeFS->addTerm(new fl::Triangle("EASY", 2.000, 3.000, 4.000));
    responseModeFS->addTerm(new fl::Triangle("ENTHUSIASTIC", 3.000, 4.000, 5.000));
    emotionEngine->addOutputVariable(responseModeFS);

    ruleBlock = new fl::RuleBlock;
	ruleBlock->setName("mamdani");
	ruleBlock->setDescription("");
	ruleBlock->setEnabled(true);
	ruleBlock->setConjunction(new fl::AlgebraicProduct);
	ruleBlock->setDisjunction(new fl::AlgebraicSum);
	ruleBlock->setImplication(new fl::AlgebraicProduct);
	ruleBlock->setActivation(new fl::General);

    ruleBlock->addRule(fl::Rule::parse("if JOY is SAD and FEAR is AFFRAID and APPROVAL is DISLIKE then FACE_EYELIDS is CLOSED and FACE_EYEBROWS is LOW and FACE_MOUTH is SO_SAD and VOICE_RATE is LOW and VOICE_RESPONSE is BLUE", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if JOY is SAD and FEAR is AFFRAID and APPROVAL is NORMAL then FACE_EYELIDS is CLOSED and FACE_EYEBROWS is NORMAL and FACE_MOUTH is SO_SAD and VOICE_RATE is NORMAL and VOICE_RESPONSE is BLUE", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if JOY is SAD and FEAR is AFFRAID and APPROVAL is LIKE then FACE_EYELIDS is HALF_CLOSED and FACE_EYEBROWS is HIGH and FACE_MOUTH is SO_SAD and VOICE_RATE is LOW and VOICE_RESPONSE is HYPOCRITE", emotionEngine));

    ruleBlock->addRule(fl::Rule::parse("if JOY is SAD and FEAR is NORMAL and APPROVAL is DISLIKE then FACE_EYELIDS is CLOSED and FACE_EYEBROWS is LOW and FACE_MOUTH is SAD and VOICE_RATE is LOW and VOICE_RESPONSE is BLUE", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if JOY is SAD and FEAR is NORMAL and APPROVAL is NORMAL then FACE_EYELIDS is CLOSED and FACE_EYEBROWS is NORMAL and FACE_MOUTH is SAD and VOICE_RATE is NORMAL and VOICE_RESPONSE is HYPOCRITE", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if JOY is SAD and FEAR is NORMAL and APPROVAL is LIKE then FACE_EYELIDS is HALF_CLOSED and FACE_EYEBROWS is NORMAL and FACE_MOUTH is SO_SAD and VOICE_RATE is LOW and VOICE_RESPONSE is HYPOCRITE", emotionEngine));

    ruleBlock->addRule(fl::Rule::parse("if JOY is SAD and FEAR is CALM and APPROVAL is DISLIKE then FACE_EYELIDS is HALF_CLOSED and FACE_EYEBROWS is LOW and FACE_MOUTH is SO_SAD and VOICE_RATE is LOW and VOICE_RESPONSE is BLUE", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if JOY is SAD and FEAR is CALM and APPROVAL is NORMAL then FACE_EYELIDS is HALF_CLOSED and FACE_EYEBROWS is NORMAL and FACE_MOUTH is SAD and VOICE_RATE is LOW and VOICE_RESPONSE is BLUE", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if JOY is SAD and FEAR is CALM and APPROVAL is LIKE then FACE_EYELIDS is HALF_CLOSED and FACE_EYEBROWS is NORMAL and FACE_MOUTH is SAD and VOICE_RATE is NORMAL and VOICE_RESPONSE is HYPOCRITE", emotionEngine));


    ruleBlock->addRule(fl::Rule::parse("if JOY is NORMAL and FEAR is AFFRAID and APPROVAL is DISLIKE then FACE_EYELIDS is NORMAL and FACE_EYEBROWS is LOW and FACE_MOUTH is SAD and VOICE_RATE is HIGH and VOICE_RESPONSE is HYPOCRITE", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if JOY is NORMAL and FEAR is AFFRAID and APPROVAL is NORMAL then FACE_EYELIDS is HALF_CLOSED and FACE_EYEBROWS is NORMAL and FACE_MOUTH is SAD and VOICE_RATE is NORMAL and VOICE_RESPONSE is NORMAL", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if JOY is NORMAL and FEAR is AFFRAID and APPROVAL is LIKE then FACE_EYELIDS is HALF_CLOSED and FACE_EYEBROWS is NORMAL and FACE_MOUTH is SAD and VOICE_RATE is NORMAL and VOICE_RESPONSE is NORMAL", emotionEngine));

    ruleBlock->addRule(fl::Rule::parse("if JOY is NORMAL and FEAR is NORMAL and APPROVAL is DISLIKE then FACE_EYELIDS is NORMAL and FACE_EYEBROWS is LOW and FACE_MOUTH is NORMAL and VOICE_RATE is LOW and VOICE_RESPONSE is HYPOCRITE", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if JOY is NORMAL and FEAR is NORMAL and APPROVAL is NORMAL then FACE_EYELIDS is NORMAL and FACE_EYEBROWS is NORMAL and FACE_MOUTH is NORMAL and VOICE_RATE is NORMAL and VOICE_RESPONSE is NORMAL", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if JOY is NORMAL and FEAR is NORMAL and APPROVAL is LIKE then FACE_EYELIDS is NORMAL and FACE_EYEBROWS is NORMAL and FACE_MOUTH is NORMAL and VOICE_RATE is HIGH and VOICE_RESPONSE is NORMAL", emotionEngine));

    ruleBlock->addRule(fl::Rule::parse("if JOY is NORMAL and FEAR is CALM and APPROVAL is DISLIKE then FACE_EYELIDS is NORMAL and FACE_EYEBROWS is LOW and FACE_MOUTH is NORMAL and VOICE_RATE is NORMAL and VOICE_RESPONSE is NORMAL", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if JOY is NORMAL and FEAR is CALM and APPROVAL is NORMAL then FACE_EYELIDS is NORMAL and FACE_EYEBROWS is HIGH and FACE_MOUTH is NORMAL and VOICE_RATE is NORMAL and VOICE_RESPONSE is EASY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if JOY is NORMAL and FEAR is CALM and APPROVAL is LIKE then FACE_EYELIDS is NORMAL and FACE_EYEBROWS is HIGH and FACE_MOUTH is SMILEY and VOICE_RATE is NORMAL and VOICE_RESPONSE is EASY", emotionEngine));


    ruleBlock->addRule(fl::Rule::parse("if JOY is HAPPY and FEAR is AFFRAID and APPROVAL is DISLIKE then FACE_EYELIDS is HALF_OPENED and FACE_EYEBROWS is NORMAL and FACE_MOUTH is SAD and VOICE_RATE is LOW and VOICE_RESPONSE is HYPOCRITE", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if JOY is HAPPY and FEAR is AFFRAID and APPROVAL is NORMAL then FACE_EYELIDS is HALF_OPENED and FACE_EYEBROWS is NORMAL and FACE_MOUTH is NORMAL and VOICE_RATE is LOW and VOICE_RESPONSE is NORMAL", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if JOY is HAPPY and FEAR is AFFRAID and APPROVAL is LIKE then FACE_EYELIDS is HALF_OPENED and FACE_EYEBROWS is NORMAL and FACE_MOUTH is NORMAL and VOICE_RATE is NORMAL and VOICE_RESPONSE is EASY", emotionEngine));

    ruleBlock->addRule(fl::Rule::parse("if JOY is HAPPY and FEAR is NORMAL and APPROVAL is DISLIKE then FACE_EYELIDS is HALF_OPENED and FACE_EYEBROWS is NORMAL and FACE_MOUTH is SAD and VOICE_RATE is NORMAL and VOICE_RESPONSE is NORMAL", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if JOY is HAPPY and FEAR is NORMAL and APPROVAL is NORMAL then FACE_EYELIDS is HALF_OPENED and FACE_EYEBROWS is HIGH and FACE_MOUTH is SMILEY and VOICE_RATE is NORMAL and VOICE_RESPONSE is EASY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if JOY is HAPPY and FEAR is NORMAL and APPROVAL is LIKE then FACE_EYELIDS is HALF_OPENED and FACE_EYEBROWS is HIGH and FACE_MOUTH is SMILEY and VOICE_RATE is HIGH and VOICE_RESPONSE is EASY", emotionEngine));


    ruleBlock->addRule(fl::Rule::parse("if JOY is HAPPY and FEAR is CALM and APPROVAL is DISLIKE then FACE_EYELIDS is OPENED and FACE_EYEBROWS is LOW and FACE_MOUTH is SMILEY and VOICE_RATE is NORMAL and VOICE_RESPONSE is EASY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if JOY is HAPPY and FEAR is CALM and APPROVAL is NORMAL then FACE_EYELIDS is OPENED and FACE_EYEBROWS is NORMAL and FACE_MOUTH is SMILEY and VOICE_RATE is NORMAL and VOICE_RESPONSE is EASY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if JOY is HAPPY and FEAR is CALM and APPROVAL is LIKE then FACE_EYELIDS is OPENED and FACE_EYEBROWS is HIGH and FACE_MOUTH is BIG_SMILEY and VOICE_RATE is HIGH and VOICE_RESPONSE is ENTHUSIASTIC", emotionEngine));


    emotionEngine->addRuleBlock(ruleBlock);
    printf("Reglas EMOTION FUZZY cargadas\n");
}

void RNEmotionsTask::getSystemInput(double* eyelids, double* eyelashes, double* mouth, double* responseMode, double* voiceRate){
    RNUtils::printLn("Emotion: %s", currentState->toString().c_str());
    this->joyFS->setValue(currentState->getJoy());  
    this->fearFS->setValue(currentState->getFear());
    this->approvalFS->setValue(currentState->getApproval());

    emotionEngine->process();

    *eyelids = this->eyelidsFS->getValue();
    *eyelashes = this->eyebrowsFS->getValue();
    *mouth = this->mouthFS->getValue();
    *voiceRate = this->voiceRateFS->getValue();
    *responseMode = this->responseModeFS->getValue();
}

void RNEmotionsTask::task(){ //This is supposed to be sensing Doris emotion by: speaking....
    //conditions by speaking
   if(spokenId != RN_NONE){
       bool found = false;
        for (int i = 0; i < impulses->size() and not found; i++){
            if(spokenId == impulses->at(i)->getId()){
                found = true;
                currentState->setJoy(impulses->at(i)->getJoy() + currentState->getJoy());
                currentState->setFear(impulses->at(i)->getFear() + currentState->getFear());
                currentState->setApproval(impulses->at(i)->getApproval() + currentState->getApproval());
            }
        }
        RNUtils::printLn("SpokenId: {%d}", spokenId);
        spokenId = RN_NONE;
        if(found){
            double eyelids, eyebrows, mouth, responseMode, voiceRate;
            getSystemInput(&eyelids, &eyebrows, &mouth, &responseMode, &voiceRate);

            RNUtils::printLn("Emotion: {EL: %lf, EB: %lf, Mouth: %lf, ResponseType: %lf, VoiceRate: %lf}", eyelids, eyebrows, mouth, responseMode, voiceRate);
            setDialogState(responseMode, voiceRate);
            setFace(eyelids, eyebrows, mouth);
        }
        // set face
        // modify speaking rate and volume.
    }
    

    
}

void RNEmotionsTask::setSpokenImpulseId(int id){
    this->spokenId = id;
}

void RNEmotionsTask::setDialogState(double responseMode, double voiceRate){ //Función para enviar parámetro state a la clase RNDialogsTask para que Doris responda en función de su estado de ánimo
    int type = static_cast<int>(responseMode);
    std::string state = std::to_string(type);
	gn->setEmotionsResult(state, "");
}

void RNEmotionsTask::setFace(double eyelids, double eyebrows, double mouth){ //Función para enviar parámetro id a RNGestureTask para indicar qué cara tiene que poner 
    for(int i = 0; i < faceMotors->mouthMotorsSize(); i++){
        double valor = RNUtils::linearInterpolator(mouth, PointXY(0, faceMotors->mouthMotorAt(i)->getLow()), PointXY(100, faceMotors->mouthMotorAt(i)->getHigh()));
        uint8_t card_id = faceMotors->mouthMotorAt(i)->getCardId();
        uint8_t servo_id = faceMotors->mouthMotorAt(i)->getId();
        uint16_t position = static_cast<uint16_t>(valor);
        this->maestroController->setTarget(card_id, servo_id, position);
    }

    for(int i = 0; i < faceMotors->eyelidsMotorsSize(); i++){
        double valor = RNUtils::linearInterpolator(mouth, PointXY(0, faceMotors->eyelidsMotorAt(i)->getLow()), PointXY(100, faceMotors->eyelidsMotorAt(i)->getHigh()));
        uint8_t card_id = faceMotors->eyelidsMotorAt(i)->getCardId();
        uint8_t servo_id = faceMotors->eyelidsMotorAt(i)->getId();
        uint16_t position = static_cast<uint16_t>(valor);
        this->maestroController->setTarget(card_id, servo_id, position);
    }

    for(int i = 0; i < faceMotors->eyebrowsMotorsSize(); i++){
        double valor = RNUtils::linearInterpolator(mouth, PointXY(0, faceMotors->eyebrowsMotorAt(i)->getLow()), PointXY(100, faceMotors->eyebrowsMotorAt(i)->getHigh()));
        uint8_t card_id = faceMotors->eyebrowsMotorAt(i)->getCardId();
        uint8_t servo_id = faceMotors->eyebrowsMotorAt(i)->getId();
        uint16_t position = static_cast<uint16_t>(valor);
        this->maestroController->setTarget(card_id, servo_id, position);
    }

    /*for(int i = 0; i < faceMotors->eyesMotorsSize(); i++){
        valor = RNUtils::linearInterpolator(mouth, PointXY(0, faceMotors->eyesMotorAt(i)->getLow()), PointXY(100, faceMotors->eyesMotorAt(i)->getHigh()));
    }*/

    /*for(int i = 0; i < faceMotors->neckMotorsSize(); i++){
        valor = RNUtils::linearInterpolator(mouth, PointXY(0, faceMotors->neckMotorAt(i)->getLow()), PointXY(100, faceMotors->neckMotorAt(i)->getHigh()));
    }*/


}
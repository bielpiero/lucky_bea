#include "RNEmotionsTask.h"

RNEmotionsTask::RNEmotionsTask(const GeneralController* gn, const char* name, const char* description) : RNRecurrentTask(gn, name, description){
	this->gn = (GeneralController*)gn;
	impulses = new std::vector<Impulse*>();

	currentState = new Impulse(50, 50, 50, 50, 50);

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
        imp->setAngry(std::atof(impulse_node->first_attribute(XML_ATTRIBUTE_ANGRY_STR)->value()));
        imp->setHappy(std::atof(impulse_node->first_attribute(XML_ATTRIBUTE_HAPPY_STR)->value()));
        imp->setCalm(std::atof(impulse_node->first_attribute(XML_ATTRIBUTE_CALM_STR)->value()));
        imp->setSad(std::atof(impulse_node->first_attribute(XML_ATTRIBUTE_SAD_STR)->value()));
        imp->setAfraid(std::atof(impulse_node->first_attribute(XML_ATTRIBUTE_AFRAID_STR)->value()));
        impulses->push_back(imp);
    }

    theFile.close();
    initializeFuzzyEmotionSystem();
}

RNEmotionsTask::~RNEmotionsTask(){

}

void RNEmotionsTask::initializeFuzzyEmotionSystem(){
    emotionEngine = new fl::Engine;
    emotionEngine->setName("FuzzyEmotionSystem");
    emotionEngine->setDescription("");

    angryFS = new fl::InputVariable;
    angryFS->setName("angryDialogInput");
    angryFS->setDescription("");
    angryFS->setEnabled(true);
    angryFS->setRange(-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
    angryFS->setLockValueInRange(false);
    angryFS->addTerm(new fl::Triangle("VeryLow", -std::numeric_limits<double>::infinity(), 20.000, 40.000)); 
    angryFS->addTerm(new fl::Triangle("Low", 20.000, 40.000, 60.000)); 
    angryFS->addTerm(new fl::Triangle("Normal", 40.000, 60.000, 80.000)); 
    angryFS->addTerm(new fl::Triangle("High", 60.000, 80.000, 100.000)); 
    angryFS->addTerm(new fl::Triangle("VeryHigh", 80.000, 100.000, std::numeric_limits<double>::infinity())); 
    emotionEngine->addInputVariable(angryFS);
    
    happyFS = new fl::InputVariable;
    happyFS->setName("happyDialogInput");
    happyFS->setDescription("");
    happyFS->setEnabled(true);
    happyFS->setRange(-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
    happyFS->setLockValueInRange(false);
    happyFS->addTerm(new fl::Triangle("VeryLow", -std::numeric_limits<double>::infinity(), 20.000, 40.000)); 
    happyFS->addTerm(new fl::Triangle("Low", 20.000, 40.000, 60.000)); 
    happyFS->addTerm(new fl::Triangle("Normal", 40.000, 60.000, 80.000)); 
    happyFS->addTerm(new fl::Triangle("High", 60.000, 80.000, 100.000)); 
    happyFS->addTerm(new fl::Triangle("VeryHigh", 80.000, 100.000, std::numeric_limits<double>::infinity())); 
    emotionEngine->addInputVariable(happyFS);
    
    calmFS = new fl::InputVariable;
    calmFS->setName("calmDialogInput");
    calmFS->setDescription("");
    calmFS->setEnabled(true);
    calmFS->setRange(-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
    calmFS->setLockValueInRange(false);
    calmFS->addTerm(new fl::Triangle("VeryLow", -std::numeric_limits<double>::infinity(), 20.000, 40.000)); 
    calmFS->addTerm(new fl::Triangle("Low", 20.000, 40.000, 60.000)); 
    calmFS->addTerm(new fl::Triangle("Normal", 40.000, 60.000, 80.000)); 
    calmFS->addTerm(new fl::Triangle("High", 60.000, 80.000, 100.000)); 
    calmFS->addTerm(new fl::Triangle("VeryHigh", 80.000, 100.000, std::numeric_limits<double>::infinity())); 
    emotionEngine->addInputVariable(calmFS);
        
    sadFS = new fl::InputVariable;
    sadFS->setName("sadDialogInput");
    sadFS->setDescription("");
    sadFS->setEnabled(true);
    sadFS->setRange(-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
    sadFS->setLockValueInRange(false);
    sadFS->addTerm(new fl::Triangle("VeryLow", -std::numeric_limits<double>::infinity(), 20.000, 40.000)); 
    sadFS->addTerm(new fl::Triangle("Low", 20.000, 40.000, 60.000)); 
    sadFS->addTerm(new fl::Triangle("Normal", 40.000, 60.000, 80.000)); 
    sadFS->addTerm(new fl::Triangle("High", 60.000, 80.000, 100.000)); 
    sadFS->addTerm(new fl::Triangle("VeryHigh", 80.000, 100.000, std::numeric_limits<double>::infinity())); 
    emotionEngine->addInputVariable(sadFS);
    
    afraidFS = new fl::InputVariable;
    afraidFS->setName("afraidDialogInput");
    afraidFS->setDescription("");
    afraidFS->setEnabled(true);
    afraidFS->setRange(-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
    afraidFS->setLockValueInRange(false);
    afraidFS->addTerm(new fl::Triangle("VeryLow", -std::numeric_limits<double>::infinity(), 20.000, 40.000)); 
    afraidFS->addTerm(new fl::Triangle("Low", 20.000, 40.000, 60.000)); 
    afraidFS->addTerm(new fl::Triangle("Normal", 40.000, 60.000, 80.000)); 
    afraidFS->addTerm(new fl::Triangle("High", 60.000, 80.000, 100.000)); 
    afraidFS->addTerm(new fl::Triangle("VeryHigh", 80.000, 100.000, std::numeric_limits<double>::infinity())); 
    emotionEngine->addInputVariable(afraidFS);
    
    emotionFS = new fl::OutputVariable;
    emotionFS->setName("outputEmotion");
    emotionFS->setDescription("");
    emotionFS->setEnabled(true);
    emotionFS->setRange(0, 120);
    emotionFS->setLockValueInRange(false);
    emotionFS->setAggregation(new fl::AlgebraicSum);
    emotionFS->setDefuzzifier(new fl::Centroid(100));
    emotionFS->setDefaultValue(0.0);
    emotionFS->setLockPreviousValue(false);

    emotionFS->addTerm(new fl::Triangle("HAPPY", 0.000, 20.000, 40.000));
    emotionFS->addTerm(new fl::Triangle("SAD", 20.000, 40.000, 60.000));
    emotionFS->addTerm(new fl::Triangle("CALM", 40.000, 60.000, 80.000));
    emotionFS->addTerm(new fl::Triangle("AFRAID", 60.000, 80.000, 100.000));
    emotionFS->addTerm(new fl::Triangle("ANGRY", 80.000, 100.000, 120.000));
    emotionEngine->addOutputVariable(emotionFS);

    ruleBlock = new fl::RuleBlock;
	ruleBlock->setName("mamdani");
	ruleBlock->setDescription("");
	ruleBlock->setEnabled(true);
	ruleBlock->setConjunction(new fl::AlgebraicProduct);
	ruleBlock->setDisjunction(new fl::AlgebraicSum);
	ruleBlock->setImplication(new fl::AlgebraicProduct);
	ruleBlock->setActivation(new fl::General);
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
   
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
   
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
   
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is VeryHigh then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
        
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
        
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));

    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
        
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
        
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryHigh and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));

    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is VeryHigh then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is VeryHigh then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is VeryHigh then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));  
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is VeryHigh then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
        
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
        
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is High and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
 
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));

    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));

    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));

    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is VeryHigh then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
        
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Normal and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));

    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));

    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));

    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is VeryHigh then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
       
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is ANGRY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));

    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
        
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is AFRAID", emotionEngine)); 
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is Low and happyDialogInput is VeryLow and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is ANGRY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryHigh and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is High and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
   
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Normal and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is VeryHigh and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is High and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is VeryHigh then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is Normal and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is VeryLow then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is VeryLow then outputEmotion is HAPPY", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is Normal then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is Low and sadDialogInput is VeryLow and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is Low and calmDialogInput is VeryLow and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is HAPPY", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is VeryHigh and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is High and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is Normal and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is CALM", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is VeryHigh and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is High then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is High and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is Normal then outputEmotion is SAD", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is Normal and afraidDialogInput is Low then outputEmotion is SAD", emotionEngine));
    
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is VeryHigh then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is High then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is Normal then outputEmotion is AFRAID", emotionEngine));
    ruleBlock->addRule(fl::Rule::parse("if angryDialogInput is VeryLow and happyDialogInput is VeryLow and calmDialogInput is Low and sadDialogInput is Low and afraidDialogInput is Low then outputEmotion is CALM", emotionEngine));
    emotionEngine->addRuleBlock(ruleBlock);
    printf("Reglas EMOTION FUZZY cargadas\n");
}

void RNEmotionsTask::getSystemInput(double* emotion){
    RNUtils::printLn("Emotion: %s", currentState->toString().c_str());
    this->angryFS->setValue(currentState->getAngry());  
    this->happyFS->setValue(currentState->getHappy());
    this->calmFS->setValue(currentState->getCalm());
    this->sadFS->setValue(currentState->getSad());
    this->afraidFS->setValue(currentState->getAfraid());
    emotionEngine->process();
    *emotion = this->emotionFS->getValue();
}

void RNEmotionsTask::task(){ //This is supposed to be sensing Doris emotion by: speaking....
    //conditions by speaking
   if(spokenId != RN_NONE){
       bool found = false;
        for (int i = 0; i < impulses->size() and not found; i++){
            if(spokenId == impulses->at(i)->getId()){
                found = true;
                currentState->setAngry(impulses->at(i)->getAngry() + currentState->getAngry());
                currentState->setHappy(impulses->at(i)->getHappy() + currentState->getHappy());
                currentState->setCalm(impulses->at(i)->getCalm() + currentState->getCalm());
                currentState->setSad(impulses->at(i)->getSad() + currentState->getSad());
                currentState->setAfraid(impulses->at(i)->getAfraid() + currentState->getAfraid());
            }
        }
        spokenId = RN_NONE;
        if(found){
            double emotion;
            getSystemInput(&emotion);
            RNUtils::printLn("Emotion: %lf", emotion);
            std::string state = setDialogState(emotion);
            std::string face = setFace(emotion);
            RNUtils::printLn("State: %s, Face: %s", state.c_str(), face.c_str());
            gn->setEmotionsResult(state, face);
        }
        // set face
        // modify speaking rate and volume.
    }
    

    
}

void RNEmotionsTask::setSpokenImpulseId(int id){
    this->spokenId = id;
}

std::string RNEmotionsTask::setDialogState(double outputEmotion){ //Función para enviar parámetro state a la clase RNDialogsTask para que Doris responda en función de su estado de ánimo
    std::string state = "";
	if (outputEmotion >= 80.0){//outputEmotion=="ANGRY";
		state = "4";
	}
	else if (outputEmotion >= 60.0 and outputEmotion < 80.0) {//outputEmotion=="AFRAID";
		state = "3";
	}
	else if (outputEmotion >= 40.0 and outputEmotion < 60.0) {//outputEmotion=="CALM";
		state = "2";
	}
	else if (outputEmotion >= 20.0 and outputEmotion < 40.0) {//outputEmotion=="SAD";
		state = "1";
	}
	else if (outputEmotion < 20.0) {//outputEmotion=="HAPPY";
		state = "0";
	}
	
	return state;
}

std::string RNEmotionsTask::setFace(double outputEmotion){ //Función para enviar parámetro id a RNGestureTask para indicar qué cara tiene que poner 
    std::string id;
	if (outputEmotion >= 80 ){//outputEmotion=="ANGRY";
		if (outputEmotion < 85){ 
			id = "13";
		} else if ((outputEmotion >= 85) and (outputEmotion < 90)){ 
			id = "12";
		} else if (outputEmotion >= 90 and outputEmotion < 95){
			id = "4";
		} else if (outputEmotion >= 95){ 
			id = "8";
		}
	} else if (outputEmotion < 80 and outputEmotion >= 60 ) { //outputEmotion=="AFRAID";
		if (outputEmotion < 65){ 
			id = "24";
		} else if (outputEmotion >= 65 and outputEmotion < 70){ 
			id = "25";
		} else if (outputEmotion >= 70 and outputEmotion < 75){ 
			id = "26";
		} else if (outputEmotion >= 75){ 
			id = "0";
		}
			
	} else if (outputEmotion < 60 and outputEmotion >= 40) { //outputEmotion=="CALM";
		if (outputEmotion < 45){ 
			id = "11"; 
		} else if (outputEmotion >= 45 and outputEmotion < 50){ 
			id = "29";
		} else if (outputEmotion >= 50 and outputEmotion < 55){ 
			id = "16";
		} else if (outputEmotion >= 55){ 
			id = "7";
		}
				
	} else if (outputEmotion < 40 and outputEmotion >= 20 ) { //outputEmotion=="SAD";
		if (outputEmotion < 25) { 
			id = "21"; 
		} else if (outputEmotion >= 25 and outputEmotion < 30){ 
			id = "20";
		} else if (outputEmotion >= 30 and outputEmotion < 35){ 
			id = "30";
		} else if (outputEmotion >= 35){ 
			id = "14";
		}
					
	} else if (outputEmotion < 20) { //outputEmotion=="HAPPY";
		if (outputEmotion < 5){  
			id = "1";
		} else if (outputEmotion >= 5 and outputEmotion < 10){ 
			id = "28"; 
		} else if (outputEmotion >= 10 and outputEmotion < 15){ 
			id = "3";
		} else if (outputEmotion >= 15){ 
			id = "10";
		}					
	}
    return id;
	// mandar a la clase GN
}
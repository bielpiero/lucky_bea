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
        imp->setAngry(std::atoi(impulse_node->first_attribute(XML_ATTRIBUTE_ANGRY_STR)->value()));
        imp->setHappy(std::atoi(impulse_node->first_attribute(XML_ATTRIBUTE_HAPPY_STR)->value()));
        imp->setCalm(std::atoi(impulse_node->first_attribute(XML_ATTRIBUTE_CALM_STR)->value()));
        imp->setSad(std::atoi(impulse_node->first_attribute(XML_ATTRIBUTE_SAD_STR)->value()));
        imp->setAfraid(std::atoi(impulse_node->first_attribute(XML_ATTRIBUTE_AFRAID_STR)->value()));
        impulses->push_back(imp);
    }

    theFile.close();
}

RNEmotionsTask::~RNEmotionsTask(){

}

void RNEmotionsTask::initializeFuzzyEmotionSystem(){
    emotionEngine = new fl::Engine;
    emotionEngine->setName("FuzzyEmotionSystem");
    emotionEngine->setDescription("");

    angryFS = new fl::InputVariable;
    angryFS->setName("AngryDialogInput");
    angryFS->setDescription("");
    angryFS->setEnabled(true);
    angryFS->setRange(-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
    angryFS->setLockValueInRange(false);
    angryFS->addTerm(new fl::Triangle("VL", -std::numeric_limits<double>::infinity(), 20.000, 40.000)); 
    angryFS->addTerm(new fl::Triangle("L", 20.000, 40.000, 60.000)); 
    angryFS->addTerm(new fl::Triangle("N", 40.000, 60.000, 80.000)); 
    angryFS->addTerm(new fl::Triangle("H", 60.000, 80.000, 100.000)); 
    angryFS->addTerm(new fl::Triangle("VH", 80.000, 100.000, std::numeric_limits<double>::infinity())); 
    emotionEngine->addInputVariable(angryFS);
    
    happyFS = new fl::InputVariable;
    happyFS->setName("happyDialogInput");
    happyFS->setDescription("");
    happyFS->setEnabled(true);
    happyFS->setRange(-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
    happyFS->setLockValueInRange(false);
    happyFS->addTerm(new fl::Triangle("VL", -std::numeric_limits<double>::infinity(), 20.000, 40.000)); 
    happyFS->addTerm(new fl::Triangle("L", 20.000, 40.000, 60.000)); 
    happyFS->addTerm(new fl::Triangle("N", 40.000, 60.000, 80.000)); 
    happyFS->addTerm(new fl::Triangle("H", 60.000, 80.000, 100.000)); 
    happyFS->addTerm(new fl::Triangle("VH", 80.000, 100.000, std::numeric_limits<double>::infinity())); 
    emotionEngine->addInputVariable(happyFS);
    
    calmFS = new fl::InputVariable;
    calmFS->setName("calmDialogInput");
    calmFS->setDescription("");
    calmFS->setEnabled(true);
    calmFS->setRange(-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
    calmFS->setLockValueInRange(false);
    calmFS->addTerm(new fl::Triangle("VL", -std::numeric_limits<double>::infinity(), 20.000, 40.000)); 
    calmFS->addTerm(new fl::Triangle("L", 20.000, 40.000, 60.000)); 
    calmFS->addTerm(new fl::Triangle("N", 40.000, 60.000, 80.000)); 
    calmFS->addTerm(new fl::Triangle("H", 60.000, 80.000, 100.000)); 
    calmFS->addTerm(new fl::Triangle("VH", 80.000, 100.000, std::numeric_limits<double>::infinity())); 
    emotionEngine->addInputVariable(calmFS);
    
    sadFS = new fl::InputVariable;
    sadFS->setName("sadDialogInput");
    sadFS->setDescription("");
    sadFS->setEnabled(true);
    sadFS->setRange(-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
    sadFS->setLockValueInRange(false);
    sadFS->addTerm(new fl::Triangle("VL", -std::numeric_limits<double>::infinity(), 20.000, 40.000)); 
    sadFS->addTerm(new fl::Triangle("L", 20.000, 40.000, 60.000)); 
    sadFS->addTerm(new fl::Triangle("N", 40.000, 60.000, 80.000)); 
    sadFS->addTerm(new fl::Triangle("H", 60.000, 80.000, 100.000)); 
    sadFS->addTerm(new fl::Triangle("VH", 80.000, 100.000, std::numeric_limits<double>::infinity())); 
    emotionEngine->addInputVariable(sadFS);
    
    afraidFS = new fl::InputVariable;
    afraidFS->setName("afraidDialogInput");
    afraidFS->setDescription("");
    afraidFS->setEnabled(true);
    afraidFS->setRange(-std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
    afraidFS->setLockValueInRange(false);
    afraidFS->addTerm(new fl::Triangle("VL", -std::numeric_limits<double>::infinity(), 20.000, 40.000)); 
    afraidFS->addTerm(new fl::Triangle("L", 20.000, 40.000, 60.000)); 
    afraidFS->addTerm(new fl::Triangle("N", 40.000, 60.000, 80.000)); 
    afraidFS->addTerm(new fl::Triangle("H", 60.000, 80.000, 100.000)); 
    afraidFS->addTerm(new fl::Triangle("VH", 80.000, 100.000, std::numeric_limits<double>::infinity())); 
    emotionEngine->addInputVariable(afraidFS);
    
    emotionFS->setName("outputEmotion");
    emotionFS->setDescription("");
    emotionFS->setEnabled(true);
    emotionFS->setRange(0.000, 100.000);
    emotionFS->setLockValueInRange(false);
    emotionFS->setAggregation(new fl::AlgebraicSum);
    emotionFS->setDefuzzifier(new fl::Centroid(100));
    emotionFS->setDefaultValue(0.0);
    emotionFS->setLockPreviousValue(false);
    
    emotionFS->addTerm(new fl::Triangle("AFRAID", -std::numeric_limits<double>::infinity(), 20.000, 40.000));
    emotionFS->addTerm(new fl::Triangle("SAD", 20.000, 40.000, 60.000));
    emotionFS->addTerm(new fl::Triangle("CALM", 40.000, 60.000, 80.000));
    emotionFS->addTerm(new fl::Triangle("HAPPY", 60.000, 80.000, 100.000));
    emotionFS->addTerm(new fl::Triangle("CALM", 80.000, 100.000, std::numeric_limits<double>::infinity()));
    emotionEngine->addOutputVariable(emotionFS);

    ruleBlock = new fl::RuleBlock;
    ruleBlock->setName("mamdani");
    ruleBlock->setDescription("");
    ruleBlock->setEnabled(true);
    ruleBlock->setConjunction(new fl::AlgebraicProduct);
    ruleBlock->setDisjunction(new fl::AlgebraicSum);
    ruleBlock->setImplication(new fl::AlgebraicProduct);
    ruleBlock->setActivation(new fl::General);

    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is ANGRY"));
   
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is ANGRY"));
   
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is ANGRY"));
   
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is VH then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is ANGRY"));
        
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is ANGRY"));
        
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is ANGRY"));

    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is L then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is L then outputEmotion is ANGRY"));
        
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is L then outputEmotion is ANGRY"));
        
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VH and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is HAPPY"));

    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is VH then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is VH then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is VH then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is VH then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is SAD"));  
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is VH then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is CALM"));
        
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is CALM"));
        
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is ANGRY"));
    
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is L then outputEmotion is SAD"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is L then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is H and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is ANGRY"));
 
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is HAPPY"));

    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is HAPPY"));

    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is HAPPY"));
    
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is HAPPY"));

    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is VH then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is CALM"));
    
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is ANGRY"));
    
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is ANGRY"));
        
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is L then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is L then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is N then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is N and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is L then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is N then outputEmotion i HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is HAPPY"));

    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is HAPPY"));

    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is HAPPY"));

    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is VH then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is L then outputEmotion is H"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is HAPPY"));
       
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is N then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is L then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is N then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is L then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is ANGRY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is ANGRY"));

    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is CALM"));
    
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is CALM"));
    
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL AND calmDialogInput is N and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is CALM"));
        
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion i SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is N then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is L then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is AFRAID")); 
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is L then outputEmotion is SAD"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is L then outputEmotion is SAD"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is N then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is L and happyDialogInput is VL and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is L then outputEmotion is ANGRY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is L then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is L then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VH and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is L then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is N then outputEmotion is H"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is VL then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is L then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is L then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is H then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is H and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is L then outputEmotion is HAPPY"));
   
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is N then outputEmotion is H"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is L then outputEmotion is"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is L then outputEmotion is SAD"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is L then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is N then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is N and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is L then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is VH and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is H and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is VH then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is L then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is N and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is L then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is VL then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is N then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is L then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is VL then outputEmotion is HAPPY"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is N then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is L and sadDialogInput is VL and afraidDialogInput is L then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is H and afraidDialogInput is L then outputEmotion is SAD"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is N and afraidDialogInput is L then outputEmotion is SAD"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is N then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is L and calmDialogInput is VL and sadDialogInput is L and afraidDialogInput is L then outputEmotion is HAPPY"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is H and afraidDialogInput is L then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is N and afraidDialogInput is L then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is VH and sadDialogInput is L and afraidDialogInput is L then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is H and afraidDialogInput is L then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is N and afraidDialogInput is L then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is H then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is H and sadDialogInput is L and afraidDialogInput is L then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is H and afraidDialogInput is L then outputEmotion is SAD"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is N and afraidDialogInput is L then outputEmotion is CALM"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is N and sadDialogInput is L and afraidDialogInput is N then outputEmotion is CALM"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is VH and afraidDialogInput is L then outputEmotion is SAD"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is H then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is H and afraidDialogInput is L then outputEmotion is SAD"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is N then outputEmotion is SAD"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is N and afraidDialogInput is L then outputEmotion is SAD"));
    
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is VH then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is H then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is N then outputEmotion is AFRAID"));
    ruleBlock->addRule(new fl::Rule("if angryDialogInput is VL and happyDialogInput is VL and calmDialogInput is L and sadDialogInput is L and afraidDialogInput is L then outputEmotion is CALM"));
    
    ruleBlock->loadRules(emotionEngine);
    emotionEngine->addRuleBlock(ruleBlock);
}

void RNEmotionsTask::getSystemInput(double* emotion){

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
        double emotion;
        getSystemInput(&emotion);
        setDialogState(emotion);
        setFace(emotion);
        spokenId = RN_NONE;
        // set face
        // modify speaking rate and volume.
    }
    

    
}

void RNEmotionsTask::setSpokenImpulseId(int id){
    this->spokenId = id;
}

void RNEmotionsTask::setDialogState(double outputEmotion){ //Función para enviar parámetro state a la clase RNDialogsTask para que Doris responda en función de su estado de ánimo
    std::string state = RN_NONE;
	if ((outputEmotion < -std::numeric_limits<double>::infinity()) and (outputEmotion > 80)){//outputEmotion=="ANGRY";
		state = "1";
	}
	else if (outputEmotion < 80 and outputEmotion > 60) {//outputEmotion=="HAPPY";
		state = "4";
	}
	else if (outpoutputEmotion < 60 and outputEmotion > 40) {//outputEmotion=="CALM";
		state = "3";
	}
	else if (outputEmotion < 40 and outputEmotion > 20 ) {//outputEmotion=="SAD";
		state = "2";
	}
	else if (outputEmotion < 20 and outputEmotion > -std::numeric_limits<double>::infinity()) {//outputEmotion=="AFRAID";
		state = "0";
	}
	
	//return state;
}

void RNEmotionsTask::setFace(double outputEmotion){ //Función para enviar parámetro id a RNGestureTask para indicar qué cara tiene que poner 
    std::string id;
	if ((outputEmotion < -std::numeric_limits<double>::infinity()) and (outputEmotion > 80) ){//outputEmotion=="ANGRY";
		if (outputEmotion < 85){ 
			id = "13";
		} else if ((outputEmotion > 85) and (outputEmotion < 90)){ 
			id = "12";
		} else if (outputEmotion > 90 and outputEmotion < 95){
			id = "4";
		} else if (outputEmotion > 95){ 
			id = "8";
		}
	} else if (outputEmotion < 80 and outputEmotion > 60 ) { //outputEmotion=="HAPPY";
		if (outputEmotion < 65){ 
			id = "24";
		} else if (outputEmotion > 65 and outputEmotion < 70){ 
			id = "25";
		} else if (outputEmotion > 70 and outputEmotion < 75){ 
			id = "26";
		} else if (outputEmotion > 80){ 
			id = "0";
		}
			
	} else if (outputEmotion < 60 and outputEmotion > 40) { //outputEmotion=="CALM";
		if (outputEmotion < 45){ 
			id = "11"; 
		} else if (outputEmotion > 45 and outputEmotion < 50){ 
			id = "29";
		} else if (outputEmotion > 50 and outputEmotion < 55){ 
			id = "16";
		} else if (outputEmotion > 55){ 
			id = "7";
		}
				
	} else if (outputEmotion < 40 and outputEmotion > 20 ) { //outputEmotion=="SAD";
		if (outputEmotion < 25) { 
			id = "21"; 
		} else if (outputEmotion > 25 and outputEmotion < 30){ 
			id = "20";
		} else if (outputEmotion > 30 and outputEmotion < 35){ 
			id = "30";
		} else if (outputEmotion > 35){ 
			id = "14";
		}
					
	} else if (outputEmotion < 20 and outputEmotion > -std::numeric_limits<double>::infinity() ) { //outputEmotion=="AFRAID";
		if (outputEmotion < 5){  
			id = "1";
		} else if (outputEmotion > 5 and outputEmotion < 10){ 
			id = "28"; 
		} else if (outputEmotion > 10 and outputEmotion < 15){ 
			id = "3";
		} else if (outputEmotion > 15){ 
			id = "10";
		}					
	}
	// mandar a la clase GN
}
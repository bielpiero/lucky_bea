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
    sadDialogInput->addTerm(new fl::Triangle("N", 40.000, 60.000, 80.000)); 
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
    emotionEngine->addOutputVariable(outputEmotion);

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

void RNEmotionsTask::getSystemInput(const double& distance, const double& angle, double* linearVelocity, double* angularVelocity){

    this->distanceError->setValue(distance);
    this->angleError->setValue(angle);

    emotionEngine->process();

    *linearVelocity = this->linearVelocity->getValue();
    *angularVelocity = this->angularVelocity->getValue();
}

void RNEmotionsTask::task(){ //This is supposed to be sensing Doris emotion by: speaking....
    //conditions by speaking
    if(spokenId != RN_NONE){
        for (int i = 0; i < impulses->size(); i++){
            if(id_input == id->at(i)->getId()){
                currentState->setAngry(impulses->at(i)->getAngry() + currentState->getAngry());
                currentState->setHappy(impulses->at(i)->getHappy() + currentState->getHappy());
                currentState->setCalm(impulses->at(i)->getCalm() + currentState->getCalm());
                currentState->setSad(impulses->at(i)->getSad() + currentState->getSad());
                currentState->setAfraid(impulses->at(i)->getAfraid() + currentState->getAfraid());
                
             }
        }
    }
    //future conditions

    //
}

void RNEmotionsTask::onKilled(){
	
}

void RNEmotionsTask::setSpokenImpulseId(int id){
    this->spokenId = id;
}
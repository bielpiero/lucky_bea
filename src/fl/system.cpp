#include "system.h"

namespace fuzzy{
    System::System(const std::string name, fuzzy::SystemType sysType, fuzzy::DefuzzificationType defuzzType){
            this->name = name;
            this->sysType = sysType;
            if(sysType == Mamdani){
                    if(defuzzType != Centroid && defuzzType != Bisector &&
                            defuzzType != MeanOfMaximum && defuzzType != SmallestOfMaximum && defuzzType != LargestOfMaximum){
                            defuzzType = Centroid;
                    }
            } else {
                    if(defuzzType != WeightedSum && defuzzType != WeightedAverage){
                            defuzzType = WeightedSum;
                    }
            }
            this->defuzzType = defuzzType;
    }

    System::~System(){

    }
    void System::setName(const std::string name){
            this->name = name;
    }

    std::string System::getName()const{
        return this->name;
    }

    void System::setSystemType(const fuzzy::SystemType value){
        this->sysType = value;
    }
    
    fuzzy::SystemType System::getSystemType() const{
        return this->sysType;
    }

    void System::setDefuzzificationType(const fuzzy::DefuzzificationType value){
        this->defuzzType = value;
    }
    
    fuzzy::DefuzzificationType System::getDefuzzificationType() const{
        return this->defuzzType;
    }
    
    void System::start(){
        
    }
    
    std::vector<std::vector<float> > System::identifyFromData(std::string filename, 
                                                                  std::vector<float> na, 
                                                                  std::vector<std::vector<float> > nb,
                                                                  std::vector<std::vector<float> > nk){
        
    }

    void System::addInput(InputVariable* item){
        if(item->getName() == ""){
			std::ostringstream ss;
			ss << "input" << this->inputs.size();
            item->setName(ss.str());
        }
        this->inputs.push_back(item);
    }

    void System::addInputAt(InputVariable* item, int index){
        this->inputs.insert(this->inputs.begin() + index, item);
    }

    void System::removeInputAt(int index){
        this->inputs.erase(this->inputs.begin() + index);
    }

    void System::removeAllInputs(){
        this->inputs.clear();
    }

    int System::numberOfInputs(){
        return this->inputs.size();
    }

    InputVariable* System::getInputByName(const std::string name){
        bool found = false;
        InputVariable* item = NULL;

        for(int i = 0; i < this->inputs.size() && !found; i++){
            if(inputs[i]->getName() == name){
                found = true;
                item = inputs[i];
            }
        }
        return item;
    }

    InputVariable* System::getInputByIndex(int index){
        return this->inputs[index];
    }


    void System::addOutput(OutputVariable* item){
        if(item->getName() == ""){
			std::ostringstream ss;
			ss << "output" << this->outputs.size();
            item->setName(ss.str());
        }
        this->outputs.push_back(item);
    }

    void System::addOutputAt(OutputVariable* item, int index){
        this->outputs.insert(this->outputs.begin() + index, item);
    }

    void System::removeOutputAt(int index){
        this->outputs.erase(this->outputs.begin() + index);
    }

    void System::removeAllOutputs(){
        this->outputs.clear();
    }

    int System::numberOfOutputs(){
        return this->outputs.size();
    }

    OutputVariable* System::getOutputByName(const std::string name){
        bool found = false;
        OutputVariable* item = NULL;

        for(int i = 0; i < this->outputs.size() && !found; i++){
            if(outputs[i]->getName() == name){
                found = true;
                item = outputs[i];
            }
        }
        return item;
    }

    OutputVariable* System::getOutputByIndex(int index){
        return this->outputs[index];
    }


    void System::addRule(Rule* item){
        this->rules.push_back(item);
    }

    void System::addRuleAt(Rule* item, int index){
        this->rules.insert(this->rules.begin() + index, item);
    }

    void System::removeRuleAt(int index){
        this->rules.erase(this->rules.begin() + index);
    }

    void System::removeAllRules(){
        this->rules.clear();
    }

    int System::numberOfRules(){
        return this->rules.size();
    }

    Rule* System::getRuleByIndex(int index){
        return this->rules[index];
    }
	
}
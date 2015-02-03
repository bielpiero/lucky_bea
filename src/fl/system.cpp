#include "system.h"

namespace fuzzy{
    system::system(const std::string name, fuzzy::systemType sysType, fuzzy::defuzzificationType defuzzType){
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

    system::~system(){

    }
    void system::setName(const std::string name){
            this->name = name;
    }

    std::string system::getName()const{
        return this->name;
    }

    void system::setSystemType(const fuzzy::systemType value){
        this->sysType = value;
    }
    
    fuzzy::systemType system::getSystemType() const{
        return this->sysType;
    }

    void system::setDefuzzificationType(const fuzzy::defuzzificationType value){
        this->defuzzType = value;
    }
    
    fuzzy::defuzzificationType system::getDefuzzificationType() const{
        return this->defuzzType;
    }
    
    void system::start(){
        
    }
    
    std::vector<std::vector<float> > system::identifyFromData(std::string filename, 
                                                                  std::vector<float> na, 
                                                                  std::vector<std::vector<float> > nb,
                                                                  std::vector<std::vector<float> > nk){
        
    }

    void system::addInput(inputVariable* item){
        if(item->getName() == ""){
            std::string name = "input" + this->inputs.size();
            item->setName(name);
        }
        this->inputs.push_back(item);
    }

    void system::addInputAt(inputVariable* item, int index){
        this->inputs.insert(this->inputs.begin() + index, item);
    }

    void system::removeInputAt(int index){
        this->inputs.erase(this->inputs.begin() + index);
    }

    void system::removeAllInputs(){
        this->inputs.clear();
    }

    int system::numberOfInputs(){
        return this->inputs.size();
    }

    inputVariable* system::getInputByName(const std::string name){
        bool found = false;
        inputVariable* item = NULL;

        for(int i = 0; i < this->inputs.size() && !found; i++){
            if(inputs[i]->getName() == name){
                found = true;
                item = inputs[i];
            }
        }
        return item;
    }

    inputVariable* system::getInputByIndex(int index){
        return this->inputs[index];
    }


    void system::addOutput(outputVariable* item){
        if(item->getName() == ""){
            std::string name = "output" + this->outputs.size();
            item->setName(name);
        }
        this->outputs.push_back(item);
    }

    void system::addOutputAt(outputVariable* item, int index){
        this->outputs.insert(this->outputs.begin() + index, item);
    }

    void system::removeOutputAt(int index){
        this->outputs.erase(this->outputs.begin() + index);
    }

    void system::removeAllOutputs(){
        this->outputs.clear();
    }

    int system::numberOfOutputs(){
        return this->outputs.size();
    }

    outputVariable* system::getOutputByName(const std::string name){
        bool found = false;
        outputVariable* item = NULL;

        for(int i = 0; i < this->outputs.size() && !found; i++){
            if(outputs[i]->getName() == name){
                found = true;
                item = outputs[i];
            }
        }
        return item;
    }

    outputVariable* system::getOutputByIndex(int index){
        return this->outputs[index];
    }


    void system::addRule(rule* item){
        this->rules.push_back(item);
    }

    void system::addRuleAt(rule* item, int index){
        this->rules.insert(this->rules.begin() + index, item);
    }

    void system::removeRuleAt(int index){
        this->rules.erase(this->rules.begin() + index);
    }

    void system::removeAllRules(){
        this->rules.clear();
    }

    int system::numberOfRules(){
        return this->rules.size();
    }

    rule* system::getRuleByIndex(int index){
        return this->rules[index];
    }
	
}
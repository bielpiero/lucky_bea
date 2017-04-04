#include "rule.h"
#include "antecedent.h"
#include "consequent.h"
#include "system.h"

namespace fuzzy{
    Rule::Rule(const std::string& text, float weight){
        this->text = text;
        this->weight = weight;
    }
    
    Rule::~Rule(){
        
    }

    Rule* Rule::parseFromString(const std::string& rule){
        Rule* result = NULL;
        std::string ruleTrimmed = FLUtils::trim(rule);
        if(ruleTrimmed != ""){
            int ifIndex = -1, thenIndex = -1;
            if((ifIndex = ruleTrimmed.find(IF_STR)) != std::string::npos and (thenIndex = ruleTrimmed.find(THEN_STR)) != std::string::npos){
                
            }
        }

        return result;

    }

    void Rule::setText(std::string text){
        this->text = text;
    }

    std::string Rule::getText() const{
        return this->text;
    }

    void Rule::setWeight(float value){
        this->weight = value;        
    }
    
    float Rule::getWeight() const{
        return this->weight;
    }
    
    void Rule::setAntecedent(Antecedent* antecedent){
        this->antecedent = antecedent;
    }

    Antecedent* Rule::getAntecedent() const{
        return this->antecedent;
    }

    void Rule::setConsequent(Consequent* consequent){
        this->consequent = consequent;        
    }
    
    Consequent* Rule::getConsequent() const{
        return this->consequent;
    }
}

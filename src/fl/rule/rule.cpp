#include "rule.h"
#include "antecedent.h"
#include "consequent.h"

namespace fuzzy{
    Rule::Rule(const std::string text, float weight){
        this->text = text;
        this->weight = weight;
    }
    
    Rule::~Rule(){
        
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
    
    void Rule::setAntecedents(std::vector<Antecedent*> antecedents){
        this->antecedents = antecedents;
    }

    std::vector<Antecedent*> Rule::getAntecedents() const{
        return this->antecedents;
    }

    void Rule::setConsequents(std::vector<Consequent*> consequents){
        this->consequents = consequents;        
    }
    
    std::vector<Consequent*> Rule::getConsequents() const{
        return this->consequents;
    }
}

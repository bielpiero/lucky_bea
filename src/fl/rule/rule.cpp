#include "rule.h"

namespace fuzzy{
    rule::rule(const std::string text, float weight){
        this->text = text;
        this->weight = weight;
    }
    
    rule::~rule(){
        
    }

    void rule::setText(std::string text){
        this->text = text;
    }

    std::string rule::getText() const{
        return this->text;
    }

    void rule::setWeight(float value){
        this->weight = value;        
    }
    
    float rule::getWeight() const{
        return this->weight;
    }
    
    void rule::setAntecedents(std::vector<antecedent> antecedents){
        this->antecedents = antecedents;
    }

    std::vector<antecedent> rule::getAntecedents() const{
        return this->antecedents;
    }

    void rule::setConsequents(std::vector<consequent> consequents){
        this->consequents = consequents;        
    }
    
    std::vector<consequent> rule::setConsequents() const{
        return this->consequents;
    }
    
    rule* rule::fromString(const std::string rule){
        
    }
}
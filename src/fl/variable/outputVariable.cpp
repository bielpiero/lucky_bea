#include "outputVariable.h"

namespace fuzzy{
    outputVariable::outputVariable(const std::string name, float minRange, float maxRange) 
                                    : variable(name, minRange, maxRange),
                                    outputValue(fuzzy::nan),
                                    defaultValue(fuzzy::nan){
        defuzz = NULL;

    }
    outputVariable::~outputVariable(){

    }
    
    defuzzifier* outputVariable::getDefuzzifier() const{
        return this->defuzz;
    }
    
    void outputVariable::setDefuzzifier(defuzzifier* defuzz){
        this->defuzz = defuzz->clone();
    }
    
    float outputVariable::getOutputValue() const{
        return outputValue;
    }
    
    void outputVariable::setOutputValue(float value){
        this->outputValue = value;
    }

    float outputVariable::getDefaultValue() const{
        return defaultValue;
    }
    
    void outputVariable::setDefaultValue(float value){
        this->defaultValue = value;
    }
    
    void outputVariable::defuzzify(){
        float result = fuzzy::nan;
        if(defuzz != NULL){
            
        } else {
            throw std::invalid_argument("Defuzzifier is null.");
        }
        
    }

}
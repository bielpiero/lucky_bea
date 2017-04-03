#include "outputVariable.h"

namespace fuzzy{
    OutputVariable::OutputVariable(const std::string name, float minRange, float maxRange) 
                                    : Variable(name, minRange, maxRange),
                                    outputValue(fuzzy::nan),
                                    defaultValue(fuzzy::nan){
        defuzz = NULL;

    }
    OutputVariable::~OutputVariable(){

    }
    
    Defuzzifier* OutputVariable::getDefuzzifier() const{
        return this->defuzz;
    }
    
    void OutputVariable::setDefuzzifier(Defuzzifier* defuzz){
        this->defuzz = defuzz->clone();
    }
    
    float OutputVariable::getOutputValue() const{
        return outputValue;
    }
    
    void OutputVariable::setOutputValue(float value){
        this->outputValue = value;
    }

    float OutputVariable::getDefaultValue() const{
        return defaultValue;
    }
    
    void OutputVariable::setDefaultValue(float value){
        this->defaultValue = value;
    }
    
    void OutputVariable::defuzzify(){
        float result = fuzzy::nan;
        if(defuzz != NULL){
            //this->defuzz->defuzzify(, minRange, maxRange);
        } else {
            throw std::invalid_argument("Defuzzifier is null.");
        }
        
    }

}
#include "inputVariable.h"

namespace fuzzy{
    InputVariable::InputVariable(const std::string name, float minRange, float maxRange) : Variable(name, minRange, maxRange){
        this->input = fuzzy::nan;
    }

    InputVariable::~InputVariable(){
    }

    void InputVariable::setInput(float value){
        this->input = value;
    }

    float InputVariable::getInput() const{
        return this->input;
    }

    std::vector<float> InputVariable::fuzzyInput() const{
        return this->fuzzify(this->input);
    }

}
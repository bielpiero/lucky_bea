#ifndef FUZZY_INPUT_VARIABLE_H
#define FUZZY_INPUT_VARIABLE_H

#include "variable.h"
namespace fuzzy{

    class InputVariable : public Variable{
    private:
        float input;
    public:
        InputVariable(const std::string name = "", float minRange = 0.0, float maxRange = 1.0);
        virtual ~InputVariable();

        virtual void setInput(float value);
        virtual float getInput() const;
        
        virtual std::vector<float> fuzzyInput() const;
    };
}

#endif
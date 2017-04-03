#ifndef FUZZY_OUTPUT_VARIABLE_H
#define FUZZY_OUTPUT_VARIABLE_H

#include "variable.h"
#include "fl/defuzzifier/defuzzifier.h"
#include "fl/defuzzifier/bisector.h"
#include "fl/defuzzifier/centroid.h"
#include "fl/defuzzifier/largestOfMaximum.h"
#include "fl/defuzzifier/smallestOfMaximum.h"
#include "fl/defuzzifier/meanOfMaximum.h"

namespace fuzzy{

    class OutputVariable : public Variable{
    private:
        float defaultValue;
        float outputValue;
        Defuzzifier* defuzz;
        
    public:
        OutputVariable(const std::string name = "", 
                float minRange = 0.0, 
                float maxRange = 1.0);
        
        virtual float getOutputValue() const;
        virtual void setOutputValue(float value);
        
        virtual float getDefaultValue() const;
        virtual void setDefaultValue(float value);
        
        virtual Defuzzifier* getDefuzzifier() const;
        virtual void setDefuzzifier(Defuzzifier* defuzz);
        
        virtual void defuzzify();
                        
        virtual ~OutputVariable();
    };
}

#endif
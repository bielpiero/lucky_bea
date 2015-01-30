#ifndef FUZZY_SYSTEM_H
#define FUZZY_SYSTEM_H

#include <vector>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <string>
#include <matio.h>

#include "constants.h"
#include "variable/inputVariable.h"
#include "variable/outputVariable.h"
#include "rule/rule.h"

namespace fuzzy{
    class system{
    private:
        std::string name;
        fuzzy::systemType sysType;
        fuzzy::defuzzificationType defuzzType;

        std::vector<inputVariable*> inputs;
        std::vector<outputVariable*> outputs;
        std::vector<rule*> rules;

    public:
        system(const std::string name = "", fuzzy::systemType sysType = Mamdani, fuzzy::defuzzificationType defuzzType = Centroid);
        ~system();

        virtual void setName(std::string name);
        virtual std::string getName() const;

        virtual void setSystemType(fuzzy::systemType value);
        virtual fuzzy::systemType getSystemType() const;

        virtual void setDefuzzificationType(fuzzy::defuzzificationType value);
        virtual fuzzy::defuzzificationType getDefuzzificationType() const;
        
        virtual std::vector<std::vector<float> > identifyFromData(std::string filename, 
                                                                  std::vector<float> na = std::vector<float>(1,1), 
                                                                  std::vector<std::vector<float> > nb = std::vector<std::vector<float> >(1, std::vector<float>(1, 1)),
                                                                  std::vector<std::vector<float> > nk = std::vector<std::vector<float> >(1, std::vector<float>(1, 0)));
        virtual void start();

        virtual void addInput(inputVariable* item);
        virtual void addInputAt(inputVariable* item, int index);
        virtual void removeInputAt(int index);
        virtual void removeAllInputs();
        virtual int numberOfInputs();
        virtual inputVariable* getInputByName(const std::string name);
        virtual inputVariable* getInputByIndex(int index);

        virtual void addOutput(outputVariable* item);
        virtual void addOutputAt(outputVariable* item, int index);
        virtual void removeOutputAt(int index);
        virtual void removeAllOutputs();
        virtual int numberOfOutputs();
        virtual outputVariable* getOutputByName(const std::string name);
        virtual outputVariable* getOutputByIndex(int index);

        virtual void addRule(rule* item);
        virtual void addRuleAt(rule* item, int index);
        virtual void removeRuleAt(int index);
        virtual void removeAllRules();
        virtual int numberOfRules();
        virtual rule* getRuleByName(const std::string name);
        virtual rule* getRuleByIndex(int index);
    };
}
#endif

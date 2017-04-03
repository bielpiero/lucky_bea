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
    class System{
    private:
        std::string name;
        fuzzy::SystemType sysType;
        fuzzy::DefuzzificationType defuzzType;

        std::vector<InputVariable*> inputs;
        std::vector<OutputVariable*> outputs;
        std::vector<Rule*> rules;

    public:
        System(const std::string name = "", fuzzy::SystemType sysType = Mamdani, fuzzy::DefuzzificationType defuzzType = Centroid);
        ~System();

        virtual void setName(std::string name);
        virtual std::string getName() const;

        virtual void setSystemType(fuzzy::SystemType value);
        virtual fuzzy::SystemType getSystemType() const;

        virtual void setDefuzzificationType(fuzzy::DefuzzificationType value);
        virtual fuzzy::DefuzzificationType getDefuzzificationType() const;
        
        virtual std::vector<std::vector<float> > identifyFromData(std::string filename, 
                                                                  std::vector<float> na = std::vector<float>(1,1), 
                                                                  std::vector<std::vector<float> > nb = std::vector<std::vector<float> >(1, std::vector<float>(1, 1)),
                                                                  std::vector<std::vector<float> > nk = std::vector<std::vector<float> >(1, std::vector<float>(1, 0)));
        virtual void start();

        virtual void addInput(InputVariable* item);
        virtual void addInputAt(InputVariable* item, int index);
        virtual void removeInputAt(int index);
        virtual void removeAllInputs();
        virtual int numberOfInputs();
        virtual InputVariable* getInputByName(const std::string name);
        virtual InputVariable* getInputByIndex(int index);

        virtual void addOutput(OutputVariable* item);
        virtual void addOutputAt(OutputVariable* item, int index);
        virtual void removeOutputAt(int index);
        virtual void removeAllOutputs();
        virtual int numberOfOutputs();
        virtual OutputVariable* getOutputByName(const std::string name);
        virtual OutputVariable* getOutputByIndex(int index);

        virtual void addRule(Rule* item);
        virtual void addRuleAt(Rule* item, int index);
        virtual void removeRuleAt(int index);
        virtual void removeAllRules();
        virtual int numberOfRules();
        virtual Rule* getRuleByIndex(int index);
    };
}
#endif

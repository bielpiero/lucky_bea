#ifndef FUZZY_RULE_H
#define FUZZY_RULE_H

#include "FLUtils.h"

#include "stats.h"
#include "fl/constants.h"

namespace fuzzy{
    class Antecedent;
    class Consequent;
    class System;
    
    class Rule{
    private:
        std::string text;
        float weight;
        Antecedent* antecedent;
        Consequent* consequent;
        
    public:
        Rule(const std::string& text = "", float weight = 1.0);
        virtual ~Rule();

        static Rule* parseFromString(const std::string& rule);

        virtual void setText(std::string text);
        virtual std::string getText() const;

        virtual void setWeight(float value);
        virtual float getWeight() const;
        
        virtual void setAntecedent(Antecedent* antecedent);
        virtual Antecedent* getAntecedent() const;

        virtual void setConsequent(Consequent* consequent);
        virtual Consequent* getConsequent() const;
    };
}

#endif
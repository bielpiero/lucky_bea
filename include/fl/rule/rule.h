#ifndef FUZZY_RULE_H
#define FUZZY_RULE_H

#include <vector>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <string>

#include "stats.h"
#include "fl/constants.h"


namespace fuzzy{
    class Antecedent;
    class Consequent;
    
    class Rule{
    private:
        std::string text;
        float weight;
        Antecedent* antecedent;
        Consequent* consequent;
        
    public:
        Rule(const std::string text = "", float weight = 1.0);
        virtual ~Rule();

        virtual void setText(std::string text);
        virtual std::string getText() const;

        virtual void setWeight(float value);
        virtual float getWeight() const;
        
        virtual void setAntecedents(std::vector<Antecedent*> antecedents);
        virtual std::vector<Antecedent*> getAntecedents() const;

        virtual void setConsequents(std::vector<Consequent*> consequents);
        virtual std::vector<Consequent*> getConsequents() const;
    };
}

#endif
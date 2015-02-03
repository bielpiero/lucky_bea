#ifndef FUZZY_RULE_H
#define FUZZY_RULE_H

#include <vector>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <string>

#include "stats.h"

#include "antecedent.h"
#include "consequent.h"
#include "fl/constants.h"


namespace fuzzy{
    class rule{
    private:
        std::string text;
        float weight;
        std::vector<antecedent> antecedents;
        std::vector<consequent> consequents;
        
    public:
        rule(const std::string text = "", float weight = 1.0);
        virtual ~rule();

        virtual void setText(std::string text);
        virtual std::string getText() const;

        virtual void setWeight(float value);
        virtual float getWeight() const;
        
        virtual void setAntecedents(std::vector<antecedent> antecedents);
        virtual std::vector<antecedent> getAntecedents() const;

        virtual void setConsequents(std::vector<consequent> consequents);
        virtual std::vector<consequent> getConsequents() const;
    };
}

#endif
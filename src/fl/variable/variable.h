#ifndef FUZZY_VARIABLE_H
#define FUZZY_VARIABLE_H

#include <iostream>
#include <vector>
#include <string>

#include "stats.h"
#include "fl/mf/mf.h"
#include "fl/constants.h"

namespace fuzzy{
    class variable{
    protected:
        std::string name;
        float minRange, maxRange;
        std::vector<mf*> items;
    public:
        variable(const std::string name = "", float minRange = 0.0, float maxRange = 1.0);
        virtual ~variable();

        virtual std::vector<float> fuzzify(float value) const;

        virtual void setName(const std::string name);
        virtual std::string getName() const;

        virtual void setRange(float minimum, float maximum);
        virtual float range() const;

        virtual void setMinimum(float minimum);
        virtual float getMinimum() const;

        virtual void setMaximum(float maximum);
        virtual float getMaximum() const;

        virtual void addMF(mf* item);
        virtual void addMFAt(mf* item, int index);
        virtual mf* getMFByIndex(int index);
        virtual mf* getMFByName(const std::string name);

        virtual void removeMF(int index);
        virtual void removeAllMF();
        virtual int numberOfMFs() const;
    };
}
#endif
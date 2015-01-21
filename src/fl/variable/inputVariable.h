#ifndef FUZZY_INPUT_VARIABLE_H
#define FUZZY_INPUT_VARIABLE_H

#include "variable.h"

class outputVariable : public variable{
public:
	inputVariable(const std::string name = "", float minRange = 0.0, float maxRange = 1.0);
	virtual ~inputVariable();
	virtual float fuzzyfy();
};

#endif
#ifndef FUZZY_RULE_H
#define FUZZY_RULE_H

#include <vector>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <string>

#include "fl/constants.h"

namespace fuzzy{
	class rule{
	private:
		std::string name;
		float weight;
	public:
		rule(const std::string name = "", float weight = 1.0);
		virtual ~rule();
		
		virtual void setName(std::string name);
		virtual std::string getName() const;
		
		virtual void setWeight(float value);
		virtual float getWeight() const;
		
	};
}

#endif
#include "TNHamacher.h"
namespace fuzzy{
	std::string TNHamacher::className() const{
		return "TNHamacher";
	}

	float TNHamacher::evaluate(float a, float b) const{
		return ((a * b)/(a + b - (a * b)));
	}

	TNHamacher* TNHamacher::clone() const{
		return new TNHamacher(*this);
	}
}
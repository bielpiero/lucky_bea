#include "SNHamacher.h"
namespace fuzzy{
	std::string SNHamacher::className() const{
		return "SNHamacher";
	}

	float SNHamacher::evaluate(float a, float b) const{
		return ((a + b - (2.0 * a * b))/(1 - (a * b)));
	}

	SNHamacher* SNHamacher::clone() const{
		return new SNHamacher(*this);
	}
}
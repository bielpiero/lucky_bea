#include "TNProbabilistic.h"
namespace fuzzy{
	std::string TNProbabilistic::className() const{
		return "TNProbabilistic";
	}

	float TNProbabilistic::evaluate(float a, float b) const{
		return a * b;
	}

	TNProbabilistic* TNProbabilistic::clone() const{
		return new TNProbabilistic(*this);
	}
}
#include "SNProbabilistic.h"
namespace fuzzy{
	std::string SNProbabilistic::className() const{
		return "SNProbabilistic";
	}

	float SNProbabilistic::evaluate(float a, float b) const{
		return (a + b - (a * b));
	}

	SNProbabilistic* SNProbabilistic::clone() const{
		return new SNProbabilistic(*this);
	}
}
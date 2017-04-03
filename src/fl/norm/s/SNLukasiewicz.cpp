#include "SNLukasiewicz.h"
namespace fuzzy{
	std::string SNLukasiewicz::className() const{
		return "SNLukasiewicz";
	}

	float SNLukasiewicz::evaluate(float a, float b) const{
		std::vector<float> values(2);
		values[0] = 1;
		values[1] = a + b;
		return stats::min(values);
	}

	SNLukasiewicz* SNLukasiewicz::clone() const{
		return new SNLukasiewicz(*this);
	}
}
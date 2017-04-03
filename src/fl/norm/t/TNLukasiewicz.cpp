#include "TNLukasiewicz.h"
namespace fuzzy{
	std::string TNLukasiewicz::className() const{
		return "TNLukasiewicz";
	}

	float TNLukasiewicz::evaluate(float a, float b) const{
		std::vector<float> values(2);
		values[0] = 0;
		values[1] = a + b - 1;
		return stats::min(values);
	}

	TNLukasiewicz* TNLukasiewicz::clone() const{
		return new TNLukasiewicz(*this);
	}
}
#include "SNZadeh.h"
namespace fuzzy{
	std::string SNZadeh::className() const{
		return "SNZadeh";
	}

	float SNZadeh::evaluate(float a, float b) const{
		std::vector<float> values(2);
		values[0] = a;
		values[1] = b;
		return stats::max(values);
	}

	SNZadeh* SNZadeh::clone() const{
		return new SNZadeh(*this);
	}
}
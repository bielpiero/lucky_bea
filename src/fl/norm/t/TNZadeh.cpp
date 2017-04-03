#include "TNZadeh.h"
namespace fuzzy{
	std::string TNZadeh::className() const{
		return "TNZadeh";
	}

	float TNZadeh::evaluate(float a, float b) const{
		std::vector<float> values(2);
		values[0] = a;
		values[1] = b;
		return stats::min(values);
	}

	TNZadeh* TNZadeh::clone() const{
		return new TNZadeh(*this);
	}
}
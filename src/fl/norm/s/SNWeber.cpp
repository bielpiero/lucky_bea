#include "SNWeber.h"
namespace fuzzy{
	std::string SNWeber::className() const{
		return "SNHamacher";
	}

	float SNWeber::evaluate(float a, float b) const{
		float result = 0;
		if(b == 0.0){
			result = a;
		} else if(a == 0.0){
			result = b;
		}
		return result;
	}

	SNWeber* SNWeber::clone() const{
		return new SNWeber(*this);
	}
}
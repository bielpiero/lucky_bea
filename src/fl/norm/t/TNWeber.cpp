#include "TNWeber.h"
namespace fuzzy{
	std::string TNWeber::className() const{
		return "TNWeber";
	}

	float TNWeber::evaluate(float a, float b) const{
		float result = 0;
		if(b == 1.0){
			result = a;
		} else if(a == 1.0){
			result = b;
		}
		return result;
	}

	TNWeber* TNWeber::clone() const{
		return new TNWeber(*this);
	}
}
#include "inputVariable.h"

namespace fuzzy{
	inputVariable::inputVariable(const std::string name, float minRange, float maxRange) : variable(name, minRange, maxRange){
		this->input = fuzzy::nan;
	}
	
	inputVariable::~inputVariable(){
	}
	
	void inputVariable::setInput(float value){
		this->input = value;
	}
	
	float inputVariable::getInput() const{
		return this->input;
	}

}
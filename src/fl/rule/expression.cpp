#include "expression.h"

namespace fuzzy{
	Proposition::Proposition() : Expression(), variable(NULL), mf(NULL){
		isNot = false;
	}

	Proposition::~Proposition(){
		if(variable) delete variable;
		if(mf) delete mf;
	}

	Expression::Type Proposition::type() const{
		return Expression::Proposition;
	}

	std::string Proposition::className() const{
		return "Proposition";
	}

	Proposition* Proposition::clone() const{
		return new Proposition(*this);
	}

	std::string Proposition::toString() const{
		std::ostringstream out;
		if(variable){
			out << variable->getName() << " is ";
		}
		if (isNot){
			out << "not ";
		}
		if(mf){
			out << mf->getName();
		}
		return out.str();
	}

	BooleanOperator::BooleanOperator() : Expression(), left(NULL), right(NULL), name(""){

	}

	BooleanOperator::~BooleanOperator(){
		if(left) delete left;
		if(right) delete right;
	}

	std::string BooleanOperator::className() const{
		return "BooleanOperator";
	}

	BooleanOperator* BooleanOperator::clone() const{
		return new BooleanOperator(*this);
	}

	Expression::Type BooleanOperator::type() const{
		return Expression::BooleanOperator;
	}

	std::string BooleanOperator::toString() const{
		return name;
	}

}
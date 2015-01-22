#ifndef FUZZY_INPUT_VARIABLE_H
#define FUZZY_INPUT_VARIABLE_H

#include "variable.h"
namespace fuzzy{

	class inputVariable : public variable{
	private:
		float input;
	public:
		inputVariable(const std::string name = "", float minRange = 0.0, float maxRange = 1.0);
		virtual ~inputVariable();
		
		virtual void setInput(float value);
		virtual float getInput() const;
	};
}

#endif
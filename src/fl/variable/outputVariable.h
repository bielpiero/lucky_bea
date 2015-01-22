#ifndef FUZZY_OUTPUT_VARIABLE_H
#define FUZZY_OUTPUT_VARIABLE_H

#include "variable.h"
namespace fuzzy{

	class outputVariable : public variable{
        private:
            
	public:
		outputVariable(const std::string name = "", float minRange = 0.0, float maxRange = 1.0);
		virtual ~outputVariable();
	};
}

#endif
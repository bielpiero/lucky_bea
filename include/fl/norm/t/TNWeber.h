#ifndef FUZZY_TNWEBER_H
#define FUZZY_TNWEBER_H

#include "tnorm.h"

namespace fuzzy{
	class TNWeber : public TNorm{
		public:
			TNWeber(){}
			~TNWeber(){}
			virtual std::string className() const;
			virtual float evaluate(float a, float b) const;
			virtual TNWeber* clone() const;
	};
}


#endif
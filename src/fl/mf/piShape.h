#ifndef FUZZY_PI_SHAPE_H
#define FUZZY_PI_SHAPE_H

#include "mf.h"

namespace fuzzy{

	class piShape : public mf{
	public:
		piShape(std::string name="", 
				float vertexA = fuzzy::inf, 
				float vertexB = fuzzy::inf, 
				float vertexC = fuzzy::inf, 
				float height = 1.0);
		virtual ~piShape();
		virtual std::string className() const;
		virtual float evaluate(float value) const;
		virtual piShape* clone() const;
	private:
		float vertexA;
		float vertexB;
		float vertexC;
	};
}
#endif
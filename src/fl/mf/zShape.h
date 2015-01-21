#ifndef FUZZY_Z_SHAPE_H
#define FUZZY_Z_SHAPE_H

#include "mf.h"

namespace fuzzy{

	class zShape : public mf{
	public:
		zShape(std::string name="", 
				float vertexA = fuzzy::inf, 
				float vertexB = fuzzy::inf, 
				float vertexC = fuzzy::inf, 
				float height = 1.0);
		virtual ~zShape();
		virtual std::string className() const;
		virtual float evaluate(float value) const;
		virtual zShape* clone() const;
	private:
		float vertexA;
		float vertexB;
		float vertexC;
	};
}
#endif
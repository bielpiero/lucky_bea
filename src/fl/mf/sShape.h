#ifndef FUZZY_S_SHAPE_H
#define FUZZY_S_SHAPE_H

#include "mf.h"

namespace fuzzy{

	class sShape : public mf{
	public:
		sShape(std::string name="", 
				float vertexA = fuzzy::inf, 
				float vertexB = fuzzy::inf, 
				float vertexC = fuzzy::inf, 
				float height = 1.0);
		virtual ~sShape();
		virtual std::string className() const;
		virtual float evaluate(float value) const;
		virtual sShape* clone() const;
	private:
		float vertexA;
		float vertexB;
		float vertexC;
	};
}
#endif
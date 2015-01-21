#ifndef FUZZY_TRAPEZOID_H
#define FUZZY_TRAPEZOID_H

#include "mf.h"

namespace fuzzy{

	class trapezoid : public mf{
	public:
		trapezoid(std::string name="", 
				float vertexA = fuzzy::inf, 
				float vertexB = fuzzy::inf, 
				float vertexC = fuzzy::inf, 
				float vertexD = fuzzy::inf, 
				float height = 1.0);
		virtual ~trapezoid();
		
		virtual float getVertexA() const;
		virtual void setVertexA(float value);
		
		virtual float getVertexB() const;
		virtual void setVertexB(float value);
		
		virtual float getVertexC() const;
		virtual void setVertexC(float value);
		
		virtual float getVertexD() const;
		virtual void setVertexD(float value);
		
		virtual std::string className() const;
		virtual float evaluate(float value) const;
		virtual trapezoid* clone() const;
	private:
		float vertexA;
		float vertexB;
		float vertexC;
		float vertexD;
	};
}
#endif
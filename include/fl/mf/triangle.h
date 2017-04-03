#ifndef FUZZY_TRIANGLE_H
#define FUZZY_TRIANGLE_H

#include "mf.h"

namespace fuzzy{

	class Triangle : public MF{
	public:
		Triangle(std::string name="", 
				float vertexA = fuzzy::inf, 
				float vertexB = fuzzy::inf, 
				float vertexC = fuzzy::inf, 
				float height = 1.0);
		virtual ~Triangle();
		
		virtual float getVertexA() const;
		virtual void setVertexA(float value);
		
		virtual float getVertexB() const;
		virtual void setVertexB(float value);
		
		virtual float getVertexC() const;
		virtual void setVertexC(float value);
		
		virtual Triangle* operator+(const Triangle& rhs) const;
		virtual Triangle* operator-(const Triangle& rhs) const;
		
		virtual Triangle* operator*(const float& rhs) const;
		
		virtual std::string className() const;
		virtual float evaluate(float value) const;
		virtual Triangle* clone() const;
	private:
		float vertexA;
		float vertexB;
		float vertexC;
	};
}
#endif

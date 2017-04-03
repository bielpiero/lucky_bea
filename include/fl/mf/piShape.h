#ifndef FUZZY_PI_SHAPE_H
#define FUZZY_PI_SHAPE_H

#include "mf.h"
#include  "sShape.h"
#include  "zShape.h"

namespace fuzzy{

	class PiShape : public MF{
	public:
		PiShape(std::string name="", 
				float bottomLeft = fuzzy::inf, 
				float topLeft = fuzzy::inf, 
				float bottomRight = fuzzy::inf, 
				float topRight = fuzzy::inf, 
				float height = 1.0);
		virtual ~PiShape();
		
		virtual float getBottomLeft() const;
		virtual void setBottomLeft(float value);
		
		virtual float getTopLeft() const;
		virtual void setTopLeft(float value);
		
		virtual float getBottomRight() const;
		virtual void setBottomRight(float value);
		
		virtual float getTopRight() const;
		virtual void setTopRight(float value);
		
		virtual PiShape* operator+(const PiShape& rhs) const;
		virtual PiShape* operator-(const PiShape& rhs) const;
		
		virtual PiShape* operator*(const float& rhs) const;
		
		virtual std::string className() const;
		virtual float evaluate(float value) const;
		virtual PiShape* clone() const;
	private:
		float bottomLeft;
		float topLeft;
		float bottomRight;
		float topRight;
	};
}
#endif
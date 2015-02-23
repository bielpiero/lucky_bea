#ifndef FUZZY_Z_SHAPE_H
#define FUZZY_Z_SHAPE_H

#include "mf.h"

namespace fuzzy{

	class zShape : public mf{
	public:
		zShape(std::string name="", 
				float start = fuzzy::inf, 
				float end = fuzzy::inf, 
				float height = 1.0);
		virtual ~zShape();
		
		virtual void setStart(float start);
        virtual float getStart() const;

        virtual void setEnd(float end);
        virtual float getEnd() const;
		
		virtual zShape* operator+(const zShape* rhs) const;
		virtual zShape* operator-(const zShape* rhs) const;
		
		virtual zShape* operator*(const float& rhs) const;
		
		virtual std::string className() const;
		virtual float evaluate(float value) const;
		virtual zShape* clone() const;
	private:
		float start;
		float end;
	};
}
#endif
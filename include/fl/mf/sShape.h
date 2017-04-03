#ifndef FUZZY_S_SHAPE_H
#define FUZZY_S_SHAPE_H

#include "mf.h"

namespace fuzzy{

	class SShape : public MF{
	public:
		SShape(std::string name="", 
				float start = fuzzy::inf, 
				float end = fuzzy::inf, 
				float height = 1.0);
		virtual ~SShape();
		
		virtual void setStart(float start);
        virtual float getStart() const;

        virtual void setEnd(float end);
        virtual float getEnd() const;
		
		virtual SShape* operator+(const SShape& rhs) const;
		virtual SShape* operator-(const SShape& rhs) const;
		
		virtual SShape* operator*(const float& rhs) const;
		
		virtual std::string className() const;
		virtual float evaluate(float value) const;
		virtual SShape* clone() const;
	private:
		float start;
		float end;
	};
}
#endif
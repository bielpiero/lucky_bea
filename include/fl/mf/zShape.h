#ifndef FUZZY_Z_SHAPE_H
#define FUZZY_Z_SHAPE_H

#include "mf.h"

namespace fuzzy{

	class ZShape : public MF{
	public:
		ZShape(std::string name="", 
				float start = fuzzy::inf, 
				float end = fuzzy::inf, 
				float height = 1.0);
		virtual ~ZShape();
		
		virtual void setStart(float start);
        virtual float getStart() const;

        virtual void setEnd(float end);
        virtual float getEnd() const;
		
		virtual ZShape* operator+(const ZShape& rhs) const;
		virtual ZShape* operator-(const ZShape& rhs) const;
		
		virtual ZShape* operator*(const float& rhs) const;
		
		virtual std::string className() const;
		virtual float evaluate(float value) const;
		virtual ZShape* clone() const;
	private:
		float start;
		float end;
	};
}
#endif
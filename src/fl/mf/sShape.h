#ifndef FUZZY_S_SHAPE_H
#define FUZZY_S_SHAPE_H

#include "mf.h"

namespace fuzzy{

	class sShape : public mf{
	public:
		sShape(std::string name="", 
				float start = fuzzy::inf, 
				float end = fuzzy::inf, 
				float height = 1.0);
		virtual ~sShape();
		
		virtual void setStart(float start);
        virtual float getStart() const;

        virtual void setEnd(float end);
        virtual float getEnd() const;
		
		virtual std::string className() const;
		virtual float evaluate(float value) const;
		virtual sShape* clone() const;
	private:
		float start;
		float end;
	};
}
#endif
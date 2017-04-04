#ifndef FUZZY_EXPRESSION_H
#define FUZZY_EXPRESSION_H

#include <vector>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <string>

#include "variable.h"
#include "mf.h"

namespace fuzzy{
	class Expression{
		public:
			Expression(){}
			virtual ~Expression(){}
			enum Type{
				Proposition,
				BooleanOperator
			};
		protected:
			virtual Type type() const = 0;
			virtual std::string className() const = 0;
			virtual Expression* clone() const = 0;
			virtual std::string toString() const = 0;
	};

	class BooleanOperator : public Expression{
		public:
			BooleanOperator();
			virtual ~BooleanOperator();
			virtual Expression::Type type() const;
			virtual std::string className() const;
			virtual BooleanOperator* clone() const;
			virtual std::string toString() const;
		private:
			Expression* left;
			Expression* rigth;
			std::string name;
	};

	class Proposition : public Expression{
		public:
			Proposition();
			virtual ~Proposition();
			virtual Expression::Type type() const;
			virtual std::string className() const;
			virtual Proposition* clone() const;
			virtual std::string toString() const;
		private:
			Variable* variable;
			// 
			MF* mf;

			bool isNot;
	};
}


#endif
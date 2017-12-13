#ifndef FUZZY_FSTATS_H
#define FUZZY_FSTATS_H

#include "fl/term/Term.h"
#include "stats.h"

namespace fuzzy{

	class FStats{
	private:
		static double area(double x1, double x2, double x3, double x4);
	public:
		
		static double evaluateMF(fl::Term* member, double value);
		static std::vector<double> evaluateMF(fl::Term* member, std::vector<double> values);
		
		static double expectation(double x1, double x2, double x3, double x4);
		static double expectation(std::vector<double> evaluatedMembership, std::vector<double> values);

		static double uncertainty(double x1, double x2, double x3, double x4);
		static double uncertainty(std::vector<double> evaluatedMembership, std::vector<double> values);
		
		static double dependency(std::vector<double> evaluatedMembershipX, std::vector<double> x, std::vector<double> evaluatedMembershipY, std::vector<double> y);
		static double standarDeviation(std::vector<double> evaluatedMembership, std::vector<double> values);
		static double correlationCoefficient(std::vector<double> evaluatedMembershipX, std::vector<double> x, std::vector<double> evaluatedMembershipY, std::vector<double> y);
	};
}
#endif
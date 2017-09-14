#ifndef FUZZY_FSTATS_H
#define FUZZY_FSTATS_H

#include "fl/term/Term.h"
#include "stats.h"

namespace fuzzy{

	class FStats{
	private:
		static float area(float x1, float x2, float x3, float x4);
	public:
		
		static float evaluateMF(fl::Term* member, float value);
		static std::vector<float> evaluateMF(fl::Term* member, std::vector<float> values);
		
		static float expectation(float x1, float x2, float x3, float x4);
		static float expectation(std::vector<float> evaluatedMembership, std::vector<float> values);

		static float uncertainty(float x1, float x2, float x3, float x4);
		static float uncertainty(std::vector<float> evaluatedMembership, std::vector<float> values);
		
		static float dependency(std::vector<float> evaluatedMembershipX, std::vector<float> x, std::vector<float> evaluatedMembershipY, std::vector<float> y);
		static float standarDeviation(std::vector<float> evaluatedMembership, std::vector<float> values);
		static float correlationCoefficient(std::vector<float> evaluatedMembershipX, std::vector<float> x, std::vector<float> evaluatedMembershipY, std::vector<float> y);
	};
}
#endif
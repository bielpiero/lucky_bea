#ifndef FUZZY_FSTATS_H
#define FUZZY_FSTATS_H

#include "fl/mf/mf.h"

namespace fuzzy{

	class fstats{
	public:
		
		static float evaluateMF(mf* member, float value);
		static std::vector<float> evaluateMF(mf* member, std::vector<float> values);
		
		static float expectation(std::vector<float> evaluatedMembership, std::vector<float> values);
		static float uncertainty(std::vector<float> evaluatedMembership, std::vector<float> values);
		static float dependency(std::vector<float> evaluatedMembershipX, std::vector<float> x, std::vector<float> evaluatedMembershipY, std::vector<float> y);
		static float standarDeviation(std::vector<float> evaluatedMembership, std::vector<float> values);
		static float correlationCoefficient(std::vector<float> evaluatedMembershipX, std::vector<float> x, std::vector<float> evaluatedMembershipY, std::vector<float> y);
	};
}
#endif
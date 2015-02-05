#ifndef FUZZY_STATS_H
#define FUZZY_STATS_H

#include "fl/mf/mf.h"

namespace fuzzy{

	class stats{
	public:
		
		static std::vector<float> evaluateMF(mf* member, std::vector<float> values);
		
		static float expectation(std::vector<float> evaluatedMembership, std::vector<float> values);
		static float uncertainty(std::vector<float> evaluatedMembership, std::vector<float> values);
		static float dependency(std::vector<float> evaluatedMembershipX, std::vector<float> x, std::vector<float> evaluatedMembershipY, std::vector<float> y);
		static float standarDeviation(std::vector<float> evaluatedMembership, std::vector<float> values);
		static float correlationCoefficient(std::vector<float> evaluatedMembershipX, std::vector<float> x, std::vector<float> evaluatedMembershipY, std::vector<float> y);
	};
}
#endif
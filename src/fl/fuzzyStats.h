#ifndef FUZZY_STATS_H
#define FUZZY_STATS_H

#include "stats.h"
#include "fl/mf/mf.h"

namespace fuzzy{

	class stats{
	public:
		static float expectation(mf* member, std::vector<float> values);
		static float uncertainty(mf* member, std::vector<float> values);
		static float covariance(mf* memberX, std::vector<float> x, mf* memberY, std::vector<float> y);
		static float standarDeviation(mf* member, std::vector<float> values);
		static float correlationCoefficient(mf* memberX, std::vector<float> x, mf* memberY, std::vector<float> y);
	};
}
#endif
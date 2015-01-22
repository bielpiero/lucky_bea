#ifndef FUZZY_CONSTANTS_H
#define FUZZY_CONSTANTS_H

#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>
#include <limits>
#include <memory>

namespace fuzzy{
	const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
	
	enum systemType{
		Mamdani,
		TakagiSugeno
	};
	
	enum defuzzificationType{
		Bisector,
		Centroid,
		LargestOfMaximum,
		MeanOfMaximum,
		SmallestOfMaximum,
		WeightedAverage,
		WeightedSum
	};
}

#endif
#ifndef FUZZY_CONSTANTS_H
#define FUZZY_CONSTANTS_H

#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>
#include <limits>
#include <memory>
#include <stdexcept>

namespace fuzzy{
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    
    const std::string IF_KEY = "if";
    const std::string IS_KEY = "is";
    const std::string THEN_KEY = "then";
    const std::string AND_KEY = "and";
    const std::string OR_KEY = "or";
    
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
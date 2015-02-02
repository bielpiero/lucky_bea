#ifndef STATS_H
#define STATS_H

#include <iostream>
#include <cmath>
#include <algorithm>
#include <limits>
#include <vector>
#include <stdexcept>
#include <functional>

class stats{
public:
	static float max(std::vector<float> values);
	static float min(std::vector<float> values);
	
        static float sum(std::vector<float> values);
        static std::vector<int> findIndicesEqual(std::vector<float> values, float k);
        static std::vector<int> findIndicesHigherThan(std::vector<float> values, float k);
        static std::vector<int> findIndicesLessThan(std::vector<float> values, float k);
        
	static bool isInf(float value);
	static bool isNaN(float value);
	
	static float expectation(std::vector<float> values);
	static float variance(std::vector<float> values);
	static float variance(std::vector<float> values, float expectation);
	static float covariance(std::vector<float> x, std::vector<float> y);
	static float standarDeviation(std::vector<float> values);
	static float correlationCoefficient(std::vector<float> x, std::vector<float> y);
};

#endif
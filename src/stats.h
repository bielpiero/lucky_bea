#include <iostream>
#include <cmath>
#include <algorithm>
#include <limits>

class stats{
public:
	static float max(std::vector<float> values);
	static float min(std::vector<float> values);
	
	static bool isInf(float value);
	static bool isNaN(float value);
	
	static float expectation(std::vector<float> values);
	static float variance(std::vector<float> values);
	static float variance(std::vector<float> values, float expectation);
	static float covariance(std::vector<float> x, std::vector<float> y);
	static float standarDeviation(std::vector<float> values);
	static float correlationCoefficient(std::vector<float> x, std::vector<float> y);
};
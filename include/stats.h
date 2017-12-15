#ifndef STATS_H
#define STATS_H

#include <iostream>
#include <cstdarg>
#include <cmath>
#include <algorithm>
#include <limits>
#include <vector>
#include <stdexcept>

class stats{
public:
	static double max(double a, double b);
	static double min(double a, double b);

	static double max(std::vector<double> values);
	static double min(std::vector<double> values);
	
	static double sum(std::vector<double> values);
	static std::vector<int> findIndicesEqual(std::vector<double> values, double k);
	static std::vector<int> findIndicesHigherThan(std::vector<double> values, double k);
	static std::vector<int> findIndicesLessThan(std::vector<double> values, double k);
        
	static bool isInf(double value);
	static bool isNaN(double value);
	
	static double expectation(std::vector<double> values);
	static double variance(std::vector<double> values);
	static double variance(std::vector<double> values, double expectation);
	static double covariance(std::vector<double> x, std::vector<double> y);
	static double standarDeviation(std::vector<double> values);
	static double correlationCoefficient(std::vector<double> x, std::vector<double> y);
};

#endif
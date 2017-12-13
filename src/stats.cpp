#include "stats.h"


double stats::max(std::vector<double> values){
	double max = -std::numeric_limits<double>::infinity();
	if(values.size() > 0){
		for(int i = 0; i < values.size(); i++){
			if(values[i] > max  && values[i] != std::numeric_limits<double>::quiet_NaN()){
				max = values[i];
			}
		}
	}
	return max;
}

double stats::min(std::vector<double> values){
	double min = std::numeric_limits<double>::infinity();
	if(values.size() > 0){
		for(int i = 0; i < values.size(); i++){
			if(values[i] < min && values[i] != std::numeric_limits<double>::quiet_NaN()){
				min = values[i];
			}
		}
	}
	return min;
}

std::vector<int> stats::findIndicesEqual(std::vector<double> values, double k){
    std::vector<int> indices;
    for(int i = 0; i < values.size(); i++){
        if(k == values[i]){
            indices.push_back(i);
        }
    }
    return indices;
}

std::vector<int> stats::findIndicesHigherThan(std::vector<double> values, double k){
    std::vector<int> indices;
    for(int i = 0; i < values.size(); i++){
        if(values[i] > k){
            indices.push_back(i);
        }
    }
    return indices;
}

std::vector<int> stats::findIndicesLessThan(std::vector<double> values, double k){
    std::vector<int> indices;
    for(int i = 0; i < values.size(); i++){
        if(values[i] < k){
            indices.push_back(i);
        }
    }
    return indices;
}

double stats::sum(std::vector<double> values){
    double sum = std::numeric_limits<double>::quiet_NaN();
    if(values.size() > 0){  
        sum = 0;
        for(int i = 0; i < values.size(); i++){
            sum += values[i];
        }
    }
    return sum;
}

bool stats::isInf(double value){
	return (std::abs(value) == std::numeric_limits<double>::infinity());
}

bool stats::isNaN(double value){
	return (value == std::numeric_limits<double>::quiet_NaN());
}

double stats::expectation(std::vector<double> values){
	double mean = std::numeric_limits<double>::quiet_NaN();
	if(values.size() > 0){
		mean = 0;
		for(int i = 0; i < values.size(); i++){
			mean += values[i];
		}
		mean /= values.size();
	}
	return mean;
}

double stats::variance(std::vector<double> values){
	return variance(values, expectation(values));
}

double stats::variance(std::vector<double> values, double expect){
	double var = std::numeric_limits<double>::quiet_NaN();
	if(!isNaN(expect)){
		var = 0;
		for(int i = 0; i < values.size(); i++){
			var += std::pow(values[i] - expect, 2);
		}
		var /= (values.size() - 1);
	}
	return var;
}

double stats::covariance(std::vector<double> x, std::vector<double> y){
	double expX = expectation(x);
	double expY = expectation(y);
	double cov = std::numeric_limits<double>::quiet_NaN();
	if(!isNaN(expX) && !isNaN(expY)){
		if(x.size() == y.size()){
			cov = 0;
			for(int i = 0; i < x.size(); i++){
				cov += (x[i] - expX)*(y[i] - expY);
			}
		} else{
			throw std::invalid_argument("Vectors must have same dimension");			
		}
		
	}
	return cov;
}

double stats::standarDeviation(std::vector<double> values){
	double var = variance(values);
	double stdD = std::numeric_limits<double>::quiet_NaN();
	if(!isNaN(var)){
		stdD = std::sqrt(var);
	}
	return stdD;
}


double stats::correlationCoefficient(std::vector<double> x, std::vector<double> y){
	double cov = covariance(x, y);
	double stdX = standarDeviation(x);
	double stdY = standarDeviation(y);
	double cc = std::numeric_limits<double>::quiet_NaN();
	if(!isNaN(cov) && !isNaN(stdX) && !isNaN(stdY)){
		cc = cov / (stdX * stdY);
	}
	return cc;
}
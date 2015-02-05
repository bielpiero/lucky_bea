#include "stats.h"


float stats::max(std::vector<float> values){
	float max = std::numeric_limits<float>::quiet_NaN();
	if(values.size() > 0){
 		max = values[0];
		for(int i = 0; i < values.size(); i++){
			if(values[i] > max){
				max = values[i];
			}
		}
	}
	return max;
}

float stats::min(std::vector<float> values){
	float min = std::numeric_limits<float>::quiet_NaN();
	if(values.size() > 0){
		min = values[0];
		for(int i = 0; i < values.size(); i++){
			if(values[i] < min){
				min = values[i];
			}
		}
	}
	return min;
}

std::vector<int> stats::findIndicesEqual(std::vector<float> values, float k){
    std::vector<int> indices;
    for(int i = 0; i < values.size(); i++){
        if(k == values[i]){
            indices.push_back(i);
        }
    }
    return indices;
}

std::vector<int> stats::findIndicesHigherThan(std::vector<float> values, float k){
    std::vector<int> indices;
    for(int i = 0; i < values.size(); i++){
        if(k < values[i]){
            indices.push_back(i);
        }
    }
    return indices;
}

std::vector<int> stats::findIndicesLessThan(std::vector<float> values, float k){
    std::vector<int> indices;
    for(int i = 0; i < values.size(); i++){
        if(k > values[i]){
            indices.push_back(i);
        }
    }
    return indices;
}

float stats::sum(std::vector<float> values){
    float sum = std::numeric_limits<float>::quiet_NaN();
    if(values.size() > 0){  
        sum = 0;
        for(int i = 0; i < values.size(); i++){
            sum += values[i];
        }
    }
    return sum;
}

bool stats::isInf(float value){
	return (std::abs(value) == std::numeric_limits<float>::infinity());
}

bool stats::isNaN(float value){
	return (value == std::numeric_limits<float>::quiet_NaN());
}

float stats::expectation(std::vector<float> values){
	float mean = std::numeric_limits<float>::quiet_NaN();
	if(values.size() > 0){
		mean = 0;
		for(int i = 0; i < values.size(); i++){
			mean += values[i];
		}
		mean /= values.size();
	}
	return mean;
}

float stats::variance(std::vector<float> values){
	return variance(values, expectation(values));
}

float stats::variance(std::vector<float> values, float expect){
	float var = std::numeric_limits<float>::quiet_NaN();
	if(!isNaN(expect)){
		var = 0;
		for(int i = 0; i < values.size(); i++){
			var += std::pow(values[i] - expect, 2);
		}
		var /= (values.size() - 1);
	}
	return var;
}

float stats::covariance(std::vector<float> x, std::vector<float> y){
	float expX = expectation(x);
	float expY = expectation(y);
	float cov = std::numeric_limits<float>::quiet_NaN();
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

float stats::standarDeviation(std::vector<float> values){
	float var = variance(values);
	float stdD = std::numeric_limits<float>::quiet_NaN();
	if(!isNaN(var)){
		stdD = std::sqrt(var);
	}
	return stdD;
}


float stats::correlationCoefficient(std::vector<float> x, std::vector<float> y){
	float cov = covariance(x, y);
	float stdX = standarDeviation(x);
	float stdY = standarDeviation(y);
	float cc = std::numeric_limits<float>::quiet_NaN();
	if(!isNaN(cov) && !isNaN(stdX) && !isNaN(stdY)){
		cc = cov / (stdX * stdY);
	}
	return cc;
}
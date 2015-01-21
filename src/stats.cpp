#include "stats.h"

float stats::max(std::vector<float> values){

}

float stats::min(std::vector<float> values){

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
			mean += values(i);
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
			var += std::pow(values(i) - expect, 2);
		}
		var /= (values.size() - 1);
	}
	return var;
}

float stats::covariance(std::vector<float> x, std::vector<float> y){
	float expX = expectation(x);
}

float stats::standarDeviation(std::vector<float> values){
	return std::sqrt(variance(values));
}


float stats::correlationCoefficient(std::vector<float> x, std::vector<float> y){

}

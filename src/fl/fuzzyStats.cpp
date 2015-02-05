#include "fuzzyStats.h"

namespace fuzzy{
	std::vector<float> stats::evaluateMF(mf* member, std::vector<float> values){
		std::vector<float> result;
		if(values.size() > 0 && member != NULL){
			for(int i = 0; i < values.size(); i++){
				float mfr = member->evaluate(values[i]);
				result.push_back(mfr);
			}
		}
		return result;
	}
	
	float stats::expectation(std::vector<float> evaluatedMembership, std::vector<float> values){
		float result = fuzzy::nan;
		if(values.size() > 0 && evaluatedMembership.size() > 0 && values.size() == evaluatedMembership.size()){
			float area = 0;
			flaot valuesTotal = 0;
			for(int i = 0; i < values.size(); i++){
				area += evaluatedMembership[i];
				valuesTotal += evaluatedMembership[i] * values[i];
			}
			result = valuesTotal / area;
		}
		return result;
	}
	
	float stats::uncertainty(std::vector<float> evaluatedMembership, std::vector<float> values){
		float result = fuzzy::nan;
		if(values.size() > 0 && evaluatedMembership.size() > 0 && values.size() == evaluatedMembership.size()){
			float expect = expectation(evaluatedMembership, values);
			for(int i = 0; i < values.size(); i++){
				values[i] = std::pow(values[i], 2);
			}
			float expect2 = expectation(evaluatedMembership, values);
			result = expect2 - std::pow(expect, 2);
		}
		return result;
	}
	
	float stats::dependency(std::vector<float> evaluatedMembershipX, std::vector<float> x, std::vector<float> evaluatedMembershipY, std::vector<float> y){
		float result = fuzzy::nan;
		float result = fuzzy::nan;
		if((x.size() > 0 && evaluatedMembershipX.size() > 0 && x.size() == evaluatedMembershipX.size()) &&
			(y.size() > 0 && evaluatedMembershipY.size() > 0 && y.size() == evaluatedMembershipY.size()) &&
			x.size() == y.size()){

			std::vector<float> xy;
			std::vector<float> evaluatedMembershipXY;
			for(int i = 0; i < values.size(); i++){
				xy.push_back(x[i] * y[i]);
				evaluatedMembershipXY.push_back(evaluatedMembershipX * evaluatedMembershipY);
			}
						
			float expectX = expectation(evaluatedMembershipX, x);
			float expectY = expectation(evaluatedMembershipY, y);
			float expectXY = expectation(evaluatedMembershipXY, xy);
			if(expectX != fuzzy::nan && expectY != fuzzy::nan && expectXY != fuzzy::nan){
				result = expectXY - expectX * expectY;
			}
		}
		
		return result;
	}
	
	float stats::standarDeviation(std::vector<float> evaluatedMembership, std::vector<float> values){
		float result = fuzzy::nan;
		if(values.size() > 0 && evaluatedMembership.size() > 0 && values.size() == evaluatedMembership.size()){
			float vari = uncertainty(evaluatedMembership, values);
			if(vari != fuzzy::nan){
				result = std::sqrt(vari);
			}
		}
		return result;
	}
	
	float stats::correlationCoefficient(std::vector<float> evaluatedMembershipX, std::vector<float> x, std::vector<float> evaluatedMembershipY, std::vector<float> y){
		float result = fuzzy::nan;
		if((x.size() > 0 && evaluatedMembershipX.size() > 0 && x.size() == evaluatedMembershipX.size()) &&
			(y.size() > 0 && evaluatedMembershipY.size() > 0 && y.size() == evaluatedMembershipY.size())){
			float dep = dependency(evaluatedMembershipX, x, evaluatedMembershipY, y);
			float stdDevX = standarDeviation(evaluatedMembershipX, x);
			float stdDevY = standarDeviation(evaluatedMembershipY, y);
			if(dep != fuzzy::nan && stdDevX != fuzzy::nan && stdDevY != fuzzy::nan){
				result = dep / (stdDevX * stdDevY);
			}
			
		}
		return result;
	}
}
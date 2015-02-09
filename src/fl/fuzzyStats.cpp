#include "fuzzyStats.h"

namespace fuzzy{
	std::vector<float> fstats::evaluateMF(mf* member, std::vector<float> values){
		std::vector<float> result;
		if(values.size() > 0 && member != NULL){
			for(int i = 0; i < values.size(); i++){
				float mfr = member->evaluate(values[i]);
				result.push_back(mfr);
			}
		}
		return result;
	}
	
	float fstats::expectation(std::vector<float> evaluatedMembership, std::vector<float> values){
		float result = fuzzy::nan;
		if(values.size() > 0 && evaluatedMembership.size() > 0 && values.size() == evaluatedMembership.size()){
			float area = 0;
			float valuesTotal = 0;
			for(int i = 0; i < values.size(); i++){
				area += evaluatedMembership[i];
				valuesTotal += evaluatedMembership[i] * values[i];
			}
			result = valuesTotal / area;
		}
		return result;
	}
	
	float fstats::uncertainty(std::vector<float> evaluatedMembership, std::vector<float> values){
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
	
	float fstats::dependency(std::vector<float> evaluatedMembershipX, std::vector<float> x, std::vector<float> evaluatedMembershipY, std::vector<float> y){
		float result = fuzzy::nan;
		if((x.size() > 0 && evaluatedMembershipX.size() > 0 && x.size() == evaluatedMembershipX.size()) &&
			(y.size() > 0 && evaluatedMembershipY.size() > 0 && y.size() == evaluatedMembershipY.size()) &&
			x.size() == y.size()){

			std::vector<float> xy;
			std::vector<float> evaluatedMembershipXY;
			for(int i = 0; i < x.size(); i++){
				std::vector<float> mini(2);
				xy.push_back(x[i] * y[i]);
				mini[0] = evaluatedMembershipX[i];
				mini[1] = evaluatedMembershipY[i];
				evaluatedMembershipXY.push_back(stats::max(mini));
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
	
	float fstats::standarDeviation(std::vector<float> evaluatedMembership, std::vector<float> values){
		float result = fuzzy::nan;
		if(values.size() > 0 && evaluatedMembership.size() > 0 && values.size() == evaluatedMembership.size()){
			float vari = uncertainty(evaluatedMembership, values);
			if(vari != fuzzy::nan){
				result = std::sqrt(vari);
			}
		}
		return result;
	}
	
	float fstats::correlationCoefficient(std::vector<float> evaluatedMembershipX, std::vector<float> x, std::vector<float> evaluatedMembershipY, std::vector<float> y){
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
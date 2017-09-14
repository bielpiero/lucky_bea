#include "fuzzyStats.h"

namespace fuzzy{

	float FStats::area(float x1, float x2, float x3, float x4){
		return (x4 + x3 - x2 - x1)/2.0;
	}

	float FStats::evaluateMF(fl::Term* member, float value){
		float result;
		if(member != NULL){
			result = member->membership(value);			
		}
		return result;
	}
	std::vector<float> FStats::evaluateMF(fl::Term* member, std::vector<float> values){
		std::vector<float> result;
		if(values.size() > 0 && member != NULL){
			for(int i = 0; i < values.size(); i++){
				float mfr = member->membership(values[i]);
				result.push_back(mfr);
			}
		}
		return result;
	}

	float FStats::expectation(float x1, float x2, float x3, float x4){
		float a = area(x1, x2, x3, x4);
		float result = x1;
		if(a > 0){
			result = ((x4*x4 + x4*x3 + x3*x3) - (x2*x2 + x2*x1 + x1*x1))/(6 * a);
		}
		return result;
	}
	
	float FStats::expectation(std::vector<float> evaluatedMembership, std::vector<float> values){
		float result = fl::nan;
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

	float FStats::uncertainty(float x1, float x2, float x3, float x4){
		float a = area(x1, x2, x3, x4);
		float cent = expectation(x1, x2, x3, x4);
		float result = 0;
		if(a > 0){
			result = (1/a) * (((1/(x2 - x1))*(((std::pow(x2, 4) - std::pow(x1, 4))/4) - ((x1*std::pow(x2, 3) - std::pow(x1, 4))/3))) + ((std::pow(x3, 3) - std::pow(x2, 3))/3) + ((1/(x4 - x3))*(((std::pow(x4, 4) - x4*std::pow(x3, 3))/3)-((std::pow(x4, 4) - std::pow(x3, 4))/4)))) - (cent*cent);
		}
		return result;
	}
	
	float FStats::uncertainty(std::vector<float> evaluatedMembership, std::vector<float> values){
		float result = fl::nan;
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
	
	float FStats::dependency(std::vector<float> evaluatedMembershipX, std::vector<float> x, std::vector<float> evaluatedMembershipY, std::vector<float> y){
		float result = fl::nan;
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
			if(expectX != fl::nan && expectY != fl::nan && expectXY != fl::nan){
				result = expectXY - expectX * expectY;
			}
		}
		
		return result;
	}
	
	float FStats::standarDeviation(std::vector<float> evaluatedMembership, std::vector<float> values){
		float result = fl::nan;
		if(values.size() > 0 && evaluatedMembership.size() > 0 && values.size() == evaluatedMembership.size()){
			float vari = uncertainty(evaluatedMembership, values);
			if(vari != fl::nan){
				result = std::sqrt(vari);
			}
		}
		return result;
	}
	
	float FStats::correlationCoefficient(std::vector<float> evaluatedMembershipX, std::vector<float> x, std::vector<float> evaluatedMembershipY, std::vector<float> y){
		float result = fl::nan;
		if((x.size() > 0 && evaluatedMembershipX.size() > 0 && x.size() == evaluatedMembershipX.size()) &&
			(y.size() > 0 && evaluatedMembershipY.size() > 0 && y.size() == evaluatedMembershipY.size())){
			float dep = dependency(evaluatedMembershipX, x, evaluatedMembershipY, y);
			float stdDevX = standarDeviation(evaluatedMembershipX, x);
			float stdDevY = standarDeviation(evaluatedMembershipY, y);
			if(dep != fl::nan && stdDevX != fl::nan && stdDevY != fl::nan){
				result = dep / (stdDevX * stdDevY);
			}
			
		}
		return result;
	}
}
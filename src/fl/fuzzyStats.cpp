#include "fuzzyStats.h"

namespace fuzzy{

	double FStats::area(double x1, double x2, double x3, double x4){
		return (x4 + x3 - x2 - x1)/2.0;
	}

	double FStats::evaluateMF(fl::Term* member, double value){
		double result;
		if(member != NULL){
			result = member->membership(value);			
		}
		return result;
	}
	std::vector<double> FStats::evaluateMF(fl::Term* member, std::vector<double> values){
		std::vector<double> result;
		if(values.size() > 0 && member != NULL){
			for(int i = 0; i < values.size(); i++){
				double mfr = member->membership(values[i]);
				result.push_back(mfr);
			}
		}
		return result;
	}

	double FStats::expectation(double x1, double x2, double x3, double x4){
		double a = area(x1, x2, x3, x4);
		double result = x1;
		if(a > 0){
			result = ((x4*x4 + x4*x3 + x3*x3) - (x2*x2 + x2*x1 + x1*x1))/(6 * a);
		}
		return result;
	}
	
	double FStats::expectation(std::vector<double> evaluatedMembership, std::vector<double> values){
		double result = fl::nan;
		if(values.size() > 0 && evaluatedMembership.size() > 0 && values.size() == evaluatedMembership.size()){
			double area = 0;
			double valuesTotal = 0;
			for(int i = 0; i < values.size(); i++){
				area += evaluatedMembership[i];
				valuesTotal += evaluatedMembership[i] * values[i];
			}
			result = valuesTotal / area;
		}
		return result;
	}

	double FStats::uncertainty(double x1, double x2, double x3, double x4){
		double a = area(x1, x2, x3, x4);
		double cent = expectation(x1, x2, x3, x4);
		double result = 0;
		if(a > 0){
			result = (1/a) * (((1/(x2 - x1))*(((std::pow(x2, 4) - std::pow(x1, 4))/4) - ((x1*std::pow(x2, 3) - std::pow(x1, 4))/3))) + ((std::pow(x3, 3) - std::pow(x2, 3))/3) + ((1/(x4 - x3))*(((std::pow(x4, 4) - x4*std::pow(x3, 3))/3)-((std::pow(x4, 4) - std::pow(x3, 4))/4)))) - (cent*cent);
		}
		return result;
	}
	
	double FStats::uncertainty(std::vector<double> evaluatedMembership, std::vector<double> values){
		double result = fl::nan;
		if(values.size() > 0 && evaluatedMembership.size() > 0 && values.size() == evaluatedMembership.size()){
			double expect = expectation(evaluatedMembership, values);
			for(int i = 0; i < values.size(); i++){
				values[i] = std::pow(values[i], 2);
			}
			double expect2 = expectation(evaluatedMembership, values);
			result = expect2 - std::pow(expect, 2);
		}
		return result;
	}
	
	double FStats::dependency(std::vector<double> evaluatedMembershipX, std::vector<double> x, std::vector<double> evaluatedMembershipY, std::vector<double> y){
		double result = fl::nan;
		if((x.size() > 0 && evaluatedMembershipX.size() > 0 && x.size() == evaluatedMembershipX.size()) &&
			(y.size() > 0 && evaluatedMembershipY.size() > 0 && y.size() == evaluatedMembershipY.size()) &&
			x.size() == y.size()){

			std::vector<double> xy;
			std::vector<double> evaluatedMembershipXY;
			for(int i = 0; i < x.size(); i++){
				std::vector<double> mini(2);
				xy.push_back(x[i] * y[i]);
				mini[0] = evaluatedMembershipX[i];
				mini[1] = evaluatedMembershipY[i];
				evaluatedMembershipXY.push_back(stats::max(mini));
			}
						
			double expectX = expectation(evaluatedMembershipX, x);
			double expectY = expectation(evaluatedMembershipY, y);
			double expectXY = expectation(evaluatedMembershipXY, xy);
			if(expectX != fl::nan && expectY != fl::nan && expectXY != fl::nan){
				result = expectXY - expectX * expectY;
			}
		}
		
		return result;
	}
	
	double FStats::standarDeviation(std::vector<double> evaluatedMembership, std::vector<double> values){
		double result = fl::nan;
		if(values.size() > 0 && evaluatedMembership.size() > 0 && values.size() == evaluatedMembership.size()){
			double vari = uncertainty(evaluatedMembership, values);
			if(vari != fl::nan){
				result = std::sqrt(vari);
			}
		}
		return result;
	}
	
	double FStats::correlationCoefficient(std::vector<double> evaluatedMembershipX, std::vector<double> x, std::vector<double> evaluatedMembershipY, std::vector<double> y){
		double result = fl::nan;
		if((x.size() > 0 && evaluatedMembershipX.size() > 0 && x.size() == evaluatedMembershipX.size()) &&
			(y.size() > 0 && evaluatedMembershipY.size() > 0 && y.size() == evaluatedMembershipY.size())){
			double dep = dependency(evaluatedMembershipX, x, evaluatedMembershipY, y);
			double stdDevX = standarDeviation(evaluatedMembershipX, x);
			double stdDevY = standarDeviation(evaluatedMembershipY, y);
			if(dep != fl::nan && stdDevX != fl::nan && stdDevY != fl::nan){
				result = dep / (stdDevX * stdDevY);
			}
			
		}
		return result;
	}
}
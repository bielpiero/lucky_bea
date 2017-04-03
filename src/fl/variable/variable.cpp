#include "variable.h"

namespace fuzzy{

	Variable::Variable(const std::string name, float minRange, float maxRange){
		this->name = name;
		this->minRange = minRange;
		this->maxRange = maxRange;
	}		
	
	Variable::~Variable(){
	}
	
	std::vector<float> Variable::fuzzify(float value) const{
		std::vector<float> result;
		
		if((value != fuzzy::nan) && (std::abs(value) != fuzzy::inf)){
			for(int i = 0; i < this->items.size(); i++){
				float eval = items[i]->evaluate(value);
				if(eval != fuzzy::nan){
					result.push_back(eval);
				}
			}
		}
		
		return result;
	}

	void Variable::setName(const std::string name){
		this->name = name;
	}
	
	std::string Variable::getName() const{
		return this->name;
	}

	void Variable::setRange(float minimum, float maximum){
		setMinimum(minimum);
		setMaximum(maximum);
	}
	
	float Variable::range() const{
		return std::abs(std::abs(maxRange) - std::abs(minRange));
	}

	void Variable::setMinimum(float minimum){
		this->minRange = minimum;
	}
	
	float Variable::getMinimum() const{
		return this->minRange;
	}

	void Variable::setMaximum(float maximum){
		this->maxRange = maxRange;
	}
	
	float Variable::getMaximum() const{
		return this->maxRange;
	}

	void Variable::addMF(MF* item){
            if (item->getName() == ""){
				std::ostringstream ss;
				ss << "MF" << this->items.size();
                item->setName(ss.str());
            }
            this->items.push_back(item);
	}
	
	void Variable::addMFAt(MF* item, int index){
		this->items.insert(this->items.begin() + index, item);
	}
	
	void Variable::setMFAt(MF* item, int index){
		this->items[index] = item;
	}
	
	MF* Variable::getMFByIndex(int index){
		return this->items[index];
	}
	
	MF* Variable::getMFByName(const std::string name){
		bool found = false;
		MF* item = NULL;
		
		for(int i = 0; i < this->items.size() && !found; i++){
			if(items[i]->getName() == name){
				found = true;
				item = items[i];
			}
		}
		return item;
	}

	void Variable::removeMF(int index){
		this->items.erase(this->items.begin() + index);
	}
	
	void Variable::removeAllMF(){
		this->items.clear();
	}
	
	int Variable::numberOfMFs() const{
		return items.size();
	}

}
#include "variable.h"

namespace fuzzy{

	variable::variable(const std::string name, float minRange, float maxRange){
		this->name = name;
		this->minRange = minRange;
		this->maxRange = maxRange;
	}		
	
	variable::~variable(){
	}
	
	std::vector<float> variable::fuzzify(float value) const{
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

	void variable::setName(const std::string name){
		this->name = name;
	}
	
	std::string variable::getName() const{
		return this->name;
	}

	void variable::setRange(float minimum, float maximum){
		setMinimum(minimum);
		setMaximum(maximum);
	}
	
	float variable::range() const{
		return std::abs(std::abs(maxRange) - std::abs(minRange));
	}

	void variable::setMinimum(float minimum){
		this->minRange = minimum;
	}
	
	float variable::getMinimum() const{
		return this->minRange;
	}

	void variable::setMaximum(float maximum){
		this->maxRange = maxRange;
	}
	
	float variable::getMaximum() const{
		return this->maxRange;
	}

	void variable::addMF(mf* item){
            if (item->getName() == ""){
                std::string name = "mf" + this->items.size();
                item->setName(name);
            }
            this->items.push_back(item);
	}
	
	void variable::addMFAt(mf* item, int index){
		this->items.insert(this->items.begin() + index, item);
	}
	
	mf* variable::getMFByIndex(int index){
		return this->items[index];
	}
	
	mf* variable::getMFByName(const std::string name){
		bool found = false;
		mf* item = NULL;
		
		for(int i = 0; i < this->items.size() && !found; i++){
			if(items[i]->getName() == name){
				found = true;
				item = items[i];
			}
		}
		return item;
	}

	void variable::removeMF(int index){
		this->items.erase(this->items.begin() + index);
	}
	
	void variable::removeAllMF(){
		this->items.clear();
	}
	
	int variable::numberOfTerms() const{
		return items.size();
	}

}
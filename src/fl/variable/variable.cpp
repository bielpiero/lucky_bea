#include "variable.h"

namespace fuzzy{

	variable::variable(const std::string name = "", float minRange = 0.0, float maxRange = 1.0){
		this->name = name;
		this->minRange = minRange;
		this->maxRange = maxRange;
	}		
	
	variable::~variable(){
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
		return std::abs(std::abs(maximum) - std::abs(minimum));
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
		mf* item = nullptr;
		
		for(int i = 0; i < this->items->size() && !found; i++){
			if(items[i]->getName() == name){
				found = true;
				item = items[i];
			}
		}
		return item;
	}

	void variable::removeMF(int index){
		this->items->erase(this->items.begin() + index);
	}
	
	void variable::removeAllMF(){
		this->items->clear();
	}
	
	int variable::numberOfTerms() const{
		return items.size();
	}

}
#include "sigmoidDifference.h"

namespace fuzzy{
	sigmoidDifference::sigmoidDifference(std::string name, float left, float rising, float falling, float right, float height){
		this->name = name;
		this->height = height;
		
		this->left = left;
		this->rising = rising;
		this->falling = falling;
		this->right = right;
	}
	
	sigmoidDifference::~sigmoidDifference(){
	
	}
	
	std::string sigmoidDifference::className() const{
		return "sigmoidDifference";
	}
	
	float sigmoidDifference::evaluate(float value) const{
		float result = fuzzy::nan;
		
		if(value != fuzzy::nan){
			float f1 = 1 / (1 + std::exp(-rising * (value - left)));
			float f2 = 1 / (1 + std::exp(-falling * (value - right)));
			result = std::abs(f1 - f2);
		} 
		return (result * height);
	}
	
	sigmoidDifference* sigmoidDifference::clone() const{
		return new sigmoidDifference(*this);
	}
	
	sigmoidDifference* sigmoidDifference::operator+(const sigmoidDifference& rhs) const{
		return new sigmoidDifference("", this->left + rhs.left, this->rising + rhs.rising, this->falling + rhs.falling, this->right + rhs.right);
		
	}
	
	sigmoidDifference* sigmoidDifference::operator-(const sigmoidDifference& rhs) const{
		return new sigmoidDifference("", this->left - rhs.left, this->rising - rhs.rising, this->falling - rhs.falling, this->right - rhs.right);
	}
	
	sigmoidDifference* sigmoidDifference::operator*(const float& rhs) const{
		return new sigmoidDifference("", this->left * rhs, this->rising * rhs, this->falling * rhs, this->right * rhs);
	}
	
	void sigmoidDifference::setLeft(float value) {
        this->left = value;
    }

    float sigmoidDifference::getLeft() const {
        return this->left;
    }

    void sigmoidDifference::setRising(float value) {
        this->rising = value;
    }

    float sigmoidDifference::getRising() const {
		return this->rising;
    }

    void sigmoidDifference::setFalling(float value) {
        this->falling = value;
    }

    float sigmoidDifference::getFalling() const {
        return this->falling;
    }

    void sigmoidDifference::setRight(float value) {
        this->right = value;
    }

    float sigmoidDifference::getRight() const {
        return this->right;
    }	
	
}
#include "sigmoidDifference.h"

namespace fuzzy{
	SigmoidDifference::SigmoidDifference(std::string name, float left, float rising, float falling, float right, float height){
		this->name = name;
		this->height = height;
		
		this->left = left;
		this->rising = rising;
		this->falling = falling;
		this->right = right;
	}
	
	SigmoidDifference::~SigmoidDifference(){
	
	}
	
	std::string SigmoidDifference::className() const{
		return "SigmoidDifference";
	}
	
	float SigmoidDifference::evaluate(float value) const{
		float result = fuzzy::nan;
		
		if(value != fuzzy::nan){
			float f1 = 1 / (1 + std::exp(-rising * (value - left)));
			float f2 = 1 / (1 + std::exp(-falling * (value - right)));
			result = std::abs(f1 - f2);
		} 
		return (result * height);
	}
	
	SigmoidDifference* SigmoidDifference::clone() const{
		return new SigmoidDifference(*this);
	}
	
	SigmoidDifference* SigmoidDifference::operator+(const SigmoidDifference& rhs) const{
		return new SigmoidDifference("", this->left + rhs.left, this->rising + rhs.rising, this->falling + rhs.falling, this->right + rhs.right);
		
	}
	
	SigmoidDifference* SigmoidDifference::operator-(const SigmoidDifference& rhs) const{
		return new SigmoidDifference("", this->left - rhs.left, this->rising - rhs.rising, this->falling - rhs.falling, this->right - rhs.right);
	}
	
	SigmoidDifference* SigmoidDifference::operator*(const float& rhs) const{
		return new SigmoidDifference("", this->left * rhs, this->rising * rhs, this->falling * rhs, this->right * rhs);
	}
	
	void SigmoidDifference::setLeft(float value) {
        this->left = value;
    }

    float SigmoidDifference::getLeft() const {
        return this->left;
    }

    void SigmoidDifference::setRising(float value) {
        this->rising = value;
    }

    float SigmoidDifference::getRising() const {
		return this->rising;
    }

    void SigmoidDifference::setFalling(float value) {
        this->falling = value;
    }

    float SigmoidDifference::getFalling() const {
        return this->falling;
    }

    void SigmoidDifference::setRight(float value) {
        this->right = value;
    }

    float SigmoidDifference::getRight() const {
        return this->right;
    }	
	
}
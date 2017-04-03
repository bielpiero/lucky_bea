#include "sigmoidProduct.h"

namespace fuzzy{
	SigmoidProduct::SigmoidProduct(std::string name, float left, float rising, float falling, float right, float height){
		this->name = name;
		this->height = height;
		
		this->left = left;
		this->rising = rising;
		this->falling = falling;
		this->right = right;
	}
	
	SigmoidProduct::~SigmoidProduct(){
	
	}
	
	std::string SigmoidProduct::className() const{
		return "SigmoidProduct";
	}
	
	float SigmoidProduct::evaluate(float value) const{
		float result = fuzzy::nan;
		
		if(value != fuzzy::nan){
			float f1 = 1 / (1 + std::exp(-rising * (value - left)));
			float f2 = 1 / (1 + std::exp(-falling * (value - right)));
			result = f1 * f2;
		} 
		return (result * height);
	}
	
	SigmoidProduct* SigmoidProduct::operator+(const SigmoidProduct& rhs) const{
		return new SigmoidProduct("", this->left + rhs.left, this->rising + rhs.rising, this->falling + rhs.falling, this->right + rhs.right);
		
	}
	
	SigmoidProduct* SigmoidProduct::operator-(const SigmoidProduct& rhs) const{
		return new SigmoidProduct("", this->left - rhs.left, this->rising - rhs.rising, this->falling - rhs.falling, this->right - rhs.right);
	}
	
	SigmoidProduct* SigmoidProduct::operator*(const float& rhs) const{
		return new SigmoidProduct("", this->left * rhs, this->rising * rhs, this->falling * rhs, this->right * rhs);
	}
	
	SigmoidProduct* SigmoidProduct::clone() const{
		return new SigmoidProduct(*this);
	}
	
	void SigmoidProduct::setLeft(float value) {
        this->left = value;
    }

    float SigmoidProduct::getLeft() const {
        return this->left;
    }

    void SigmoidProduct::setRising(float value) {
        this->rising = value;
    }

    float SigmoidProduct::getRising() const {
		return this->rising;
    }

    void SigmoidProduct::setFalling(float value) {
        this->falling = value;
    }

    float SigmoidProduct::getFalling() const {
        return this->falling;
    }

    void SigmoidProduct::setRight(float value) {
        this->right = value;
    }

    float SigmoidProduct::getRight() const {
        return this->right;
	}
}
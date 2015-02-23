#include "sigmoidProduct.h"

namespace fuzzy{
	sigmoidProduct::sigmoidProduct(std::string name, float left, float rising, float falling, float right, float height){
		this->name = name;
		this->height = height;
		
		this->left = left;
		this->rising = rising;
		this->falling = falling;
		this->right = right;
	}
	
	sigmoidProduct::~sigmoidProduct(){
	
	}
	
	std::string sigmoidProduct::className() const{
		return "sigmoidProduct";
	}
	
	float sigmoidProduct::evaluate(float value) const{
		float result = fuzzy::nan;
		
		if(value != fuzzy::nan){
			float f1 = 1 / (1 + std::exp(-rising * (value - left)));
			float f2 = 1 / (1 + std::exp(-falling * (value - right)));
			result = f1 * f2;
		} 
		return (result * height);
	}
	
	sigmoidProduct* sigmoidProduct::operator+(const sigmoidProduct& rhs) const{
		return new sigmoidProduct("", this->left + rhs.left, this->rising + rhs.rising, this->falling + rhs.falling, this->right + rhs.right);
		
	}
	
	sigmoidProduct* sigmoidProduct::operator-(const sigmoidProduct& rhs) const{
		return new sigmoidProduct("", this->left - rhs.left, this->rising - rhs.rising, this->falling - rhs.falling, this->right - rhs.right);
	}
	
	sigmoidProduct* sigmoidProduct::operator*(const float& rhs) const{
		return new sigmoidProduct("", this->left * rhs, this->rising * rhs, this->falling * rhs, this->right * rhs);
	}
	
	sigmoidProduct* sigmoidProduct::clone() const{
		return new sigmoidProduct(*this);
	}
	
	void sigmoidProduct::setLeft(float value) {
        this->left = value;
    }

    float sigmoidProduct::getLeft() const {
        return this->left;
    }

    void sigmoidProduct::setRising(float value) {
        this->rising = value;
    }

    float sigmoidProduct::getRising() const {
		return this->rising;
    }

    void sigmoidProduct::setFalling(float value) {
        this->falling = value;
    }

    float sigmoidProduct::getFalling() const {
        return this->falling;
    }

    void sigmoidProduct::setRight(float value) {
        this->right = value;
    }

    float sigmoidProduct::getRight() const {
        return this->right;
	}
}
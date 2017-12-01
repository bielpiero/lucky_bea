#include "RNDualNumber.h"
RNDualNumber<int>::RNDualNumber(const int real, const int dual){
	this->real = real;
	this->dual = dual;
}

RNDualNumber<long int>::RNDualNumber(const long int real, const long int dual){
	this->real = real;
	this->dual = dual;
}

RNDualNumber<long long int>::RNDualNumber(const long long int real, const long long int dual){
	this->real = real;
	this->dual = dual;
}

RNDualNumber<unsigned int>::RNDualNumber(const unsigned int real, const unsigned int dual){
	this->real = real;
	this->dual = dual;
}

RNDualNumber<unsigned long int>::RNDualNumber(const unsigned long int real, const unsigned long int dual){
	this->real = real;
	this->dual = dual;
}

RNDualNumber<unsigned long long int>::RNDualNumber(const unsigned long long int real, const unsigned long long int dual){
	this->real = real;
	this->dual = dual;
}

RNDualNumber<float>::RNDualNumber(const float real, const float dual){
	this->real = real;
	this->dual = dual;
}

RNDualNumber<double>::RNDualNumber(const double real, const double dual){
	this->real = real;
	this->dual = dual;
}

RNDualNumber<RNQuaternion>::RNDualNumber(const RNQuaternion real, const RNQuaternion dual){
	this->real = real;
	this->dual = dual;
}


//AÑADIDO POR MI, AUN SIN COMPROBAR
//multiplicacion de dos quaternios duales
RNDualNumber<RNQuaternion> RNDualNumber<RNQuaternion>:: operator*(const RNDualNumber& q){
	RNDualNumber<RNQuaternion> R;
	R.real=this->real * q.real;
	R.dual=this->real*q.dual + this->dual*q.real;
	return R;
}


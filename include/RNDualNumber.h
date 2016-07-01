#ifndef RN_DUAL_NUMBER_H
#define RN_DUAL_NUMBER_H

#include <typeinfo>
#include "RNQuaternion.h"

template <typename type> class RNDualNumber { 
	type::unimplemented_function;
};

template <>
class RNDualNumber<int>{
private:
	int real;
	int dual;
public:
	RNDualNumber(const int real = 0, const int dual = 0);
	const char* toString() const;
};

template <>
class RNDualNumber<long int>{
private:
	long int real;
	long int dual;
public:
	RNDualNumber(const long int real = 0, const long int dual = 0);
	const char* toString() const;
};

template <>
class RNDualNumber<long long int>{
private:
	long long int real;
	long long int dual;
public:
	RNDualNumber(const long long int real = 0, const long long int dual = 0);
	const char* toString() const;
};

template <>
class RNDualNumber<unsigned int>{
private:
	unsigned int real;
	unsigned int dual;
public:
	RNDualNumber(const unsigned int real = 0, const unsigned int dual = 0);
	const char* toString() const;
};

template <>
class RNDualNumber<unsigned long int>{
private:
	unsigned long int real;
	unsigned long int dual;
public:
	RNDualNumber(const unsigned long int real = 0, const unsigned long int dual = 0);
	const char* toString() const;
};


template <>
class RNDualNumber<unsigned long long int>{
private:
	unsigned long long int real;
	unsigned long long int dual;
public:
	RNDualNumber(const unsigned long long int real = 0, const unsigned long long int dual = 0);
	const char* toString() const;
};

template <>
class RNDualNumber<float>{
private:
	float real;
	float dual;
public:
	RNDualNumber(const float real = 0.0, const float dual = 0.0);
	const char* toString() const;
};

template <>
class RNDualNumber<double>{
private:
	double real;
	double dual;
public:
	RNDualNumber(const double real = 0.0, const double dual = 0.0);
	const char* toString() const;
};

template <>
class RNDualNumber<RNQuaternion>{
private:
	RNQuaternion real;
	RNQuaternion dual;
public:
	RNDualNumber(const RNQuaternion real = RNQuaternion(), const RNQuaternion dual = RNQuaternion());
	const char* toString() const;
};

#endif
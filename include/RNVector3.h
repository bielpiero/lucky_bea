#ifndef RN_VECTOR_3_H
#define RN_VECTOR_3_H

#include <cmath>
#include <iostream>
#include <vector>
#include <limits>
#include <stdexcept>
#include <iomanip>
#include <cstdio>

class RNVector3{
private:
	std::vector<float> data;

	float& operator() (const int pos);
	const float& operator() (const int pos) const;
public:
	RNVector3();
	virtual ~RNVector3();
	RNVector3(const float x, const float y, const float z);

	void operator=(const RNVector3& v);

	RNVector3 operator+(const RNVector3& v);
	RNVector3 operator+(const float& v);
	friend RNVector3 operator+(const RNVector3& v1, const RNVector3& v2);
	friend RNVector3 operator+(const float& scalar, RNVector3 rhs);

	RNVector3& operator+=(const RNVector3& v);
	RNVector3& operator+=(const float& v);

	RNVector3 operator-() const;
	RNVector3 operator-(const RNVector3& v);
	RNVector3 operator-(const float& v);
	friend RNVector3 operator-(const RNVector3& v1, const RNVector3& v2);
	friend RNVector3 operator-(const float& scalar, RNVector3 rhs);

	RNVector3& operator-=(const RNVector3& v);
	RNVector3& operator-=(const float& v);

	RNVector3 operator*(const RNVector3& v);
	RNVector3 operator*(const float& v);
	friend RNVector3 operator*(const RNVector3& v1, const RNVector3& v2);
	friend RNVector3 operator*(const float& scalar, RNVector3 rhs);

	RNVector3& operator*=(const RNVector3& v);
	RNVector3& operator*=(const float& v);

	RNVector3 operator/(const RNVector3& v);
	RNVector3 operator/(const float& v);
	friend RNVector3 operator/(const RNVector3& v1, const RNVector3& v2);
	friend RNVector3 operator/(const float& scalar, RNVector3 rhs);

	RNVector3& operator/=(const RNVector3& v);
	RNVector3& operator/=(const float& v);

	bool operator==(const RNVector3& v) const;
	bool operator!=(const RNVector3& v) const;

	float dot(const RNVector3& v) const;

	float length2() const;
	float length() const;

	RNVector3& normalize();
	RNVector3 absolute() const;

	float angle(const RNVector3& v) const;

	float distance(const RNVector3& v) const;
	float distance2(const RNVector3& v) const;

	float getX() const;
	float getY() const;
	float getZ() const;

	void setX(float x);
	void setY(float y);
	void setZ(float z);

	void setZero();
	bool isZero();

	RNVector3 cross(const RNVector3& v) const;

	const char* toString() const;

};

#endif
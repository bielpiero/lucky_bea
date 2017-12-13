#ifndef RN_VECTOR_3_H
#define RN_VECTOR_3_H

#include <cmath>
#include <iostream>
#include <vector>
#include <limits>
#include <stdexcept>
#include <iomanip>
#include <cstdio>

#include <math.h>

class RNVector3{
private:
	std::vector<double> data;

	double& operator() (const unsigned int pos);
	const double& operator() (const unsigned int pos) const;
public:
	RNVector3();
	virtual ~RNVector3();
	RNVector3(const double x, const double y, const double z);

	void operator=(const RNVector3& v);

	RNVector3 operator+(const RNVector3& v);
	RNVector3 operator+(const double& v);
	friend RNVector3 operator+(const RNVector3& v1, const RNVector3& v2);
	friend RNVector3 operator+(const double& scalar, RNVector3 rhs);

	RNVector3& operator+=(const RNVector3& v);
	RNVector3& operator+=(const double& v);

	RNVector3 operator-() const;
	RNVector3 operator-(const RNVector3& v);
	RNVector3 operator-(const double& v);
	friend RNVector3 operator-(const RNVector3& v1, const RNVector3& v2);
	friend RNVector3 operator-(const double& scalar, RNVector3 rhs);

	RNVector3& operator-=(const RNVector3& v);
	RNVector3& operator-=(const double& v);

	RNVector3 operator*(const RNVector3& v);
	RNVector3 operator*(const double& v);
	friend RNVector3 operator*(const RNVector3& v1, const RNVector3& v2);
	friend RNVector3 operator*(const double& scalar, RNVector3 rhs);

	RNVector3& operator*=(const RNVector3& v);
	RNVector3& operator*=(const double& v);

	RNVector3 operator/(const RNVector3& v);
	RNVector3 operator/(const double& v);
	friend RNVector3 operator/(const RNVector3& v1, const RNVector3& v2);
	friend RNVector3 operator/(const double& scalar, RNVector3 rhs);

	RNVector3& operator/=(const RNVector3& v);
	RNVector3& operator/=(const double& v);

	bool operator==(const RNVector3& v) const;
	bool operator!=(const RNVector3& v) const;

	double dot(const RNVector3& v) const;

	double length2() const;
	double length() const;

	RNVector3 norm();
	RNVector3 absolute() const;
	RNVector3 sign();

	double angle(const RNVector3& v) const;

	double distance(const RNVector3& v) const;
	double distance2(const RNVector3& v) const;

	double getX() const;
	double getY() const;
	double getZ() const;

	void setX(double x);
	void setY(double y);
	void setZ(double z);

	void setZero();
	bool isZero();

	RNVector3 cross(const RNVector3& v) const;

	const char* toString() const;

};

#endif
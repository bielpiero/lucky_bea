#ifndef RN_QUATERNION_H
#define RN_QUATERNION_H

#include "RNVector3.h"

class RNQuaternion{
private:
	double scalar;
	RNVector3 axis;
public:
	RNQuaternion(const double scalar = 0, const RNVector3 axis = RNVector3());
	RNQuaternion (const double scalar, const double a, const double b,const double c);
	static RNQuaternion initFromEuler(const double yaw, const double pitch, const double roll);
	static RNQuaternion initFromRPY(const double roll, const double pitch, const double yaw);
	const char* toString() const;

	void operator=(const RNQuaternion& v);

	RNQuaternion operator+(const RNQuaternion& v);
	RNQuaternion operator+(const double& v);
	friend RNQuaternion operator+(const RNQuaternion& v1, const RNQuaternion& v2);

	RNQuaternion& operator+=(const RNQuaternion& v);
	RNQuaternion& operator+=(const double& v);

	RNQuaternion operator-() const;
	RNQuaternion operator-(const RNQuaternion& v);
	RNQuaternion operator-(const double& v);
	friend RNQuaternion operator-(const RNQuaternion& v1, const RNQuaternion& v2);

	RNQuaternion& operator-=(const RNQuaternion& v);
	RNQuaternion& operator-=(const double& v);

	RNQuaternion operator*(const RNQuaternion& v);
	RNQuaternion operator*(const double& v);
	friend RNQuaternion operator*(const RNQuaternion& v1, const RNQuaternion& v2);

	RNQuaternion& operator*=(const double& v);

	RNQuaternion operator/(const double& v);

	RNQuaternion& operator/=(const double& v);

	friend RNQuaternion operator!(RNQuaternion m);

	bool operator==(const RNQuaternion& v) const;
	bool operator!=(const RNQuaternion& v) const;

	bool isZero();

	double getScalar() const;
	RNVector3 getAxis() const;
	double getX() const;
	double getY() const;
	double getZ() const;

	double dot(const RNQuaternion& v) const;
	double length() const;
	double length2() const;

	double angleBetween(const RNQuaternion& q) const;
	double angleShortestPath(const RNQuaternion& q) const;

	RNQuaternion exp();
	RNQuaternion ln();
	RNQuaternion pow(double n);
	RNQuaternion sign();

	RNQuaternion norm();
	RNQuaternion conjugate();
	RNQuaternion inverse();
	RNQuaternion slerp(const RNQuaternion& q, const double& t) const;

	static const RNQuaternion& eye();

	void setScalar(double scalar);
	void setAxis(RNVector3 axis);

	RNQuaternion slerpQuat (const RNQuaternion& q, const double& t);
	double toAngle();
	
};

#endif
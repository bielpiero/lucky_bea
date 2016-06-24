#ifndef RN_QUATERNION_H
#define RN_QUATERNION_H

#include "RNVector3.h"

class RNQuaternion{
private:
	float scalar;
	RNVector3 axis;
public:
	RNQuaternion(const float scalar = 0, const RNVector3 axis = RNVector3());
	static RNQuaternion initFromEuler(const float yaw, const float pitch, const float roll);
	static RNQuaternion initFromRPY(const float roll, const float pitch, const float yaw);
	const char* toString() const;

	void operator=(const RNQuaternion& v);

	RNQuaternion operator+(const RNQuaternion& v);
	RNQuaternion operator+(const float& v);
	friend RNQuaternion operator+(const RNQuaternion& v1, const RNQuaternion& v2);

	RNQuaternion& operator+=(const RNQuaternion& v);
	RNQuaternion& operator+=(const float& v);

	RNQuaternion operator-() const;
	RNQuaternion operator-(const RNQuaternion& v);
	RNQuaternion operator-(const float& v);
	friend RNQuaternion operator-(const RNQuaternion& v1, const RNQuaternion& v2);

	RNQuaternion& operator-=(const RNQuaternion& v);
	RNQuaternion& operator-=(const float& v);

	RNQuaternion operator*(const RNQuaternion& v);
	RNQuaternion operator*(const float& v);
	friend RNQuaternion operator*(const RNQuaternion& v1, const RNQuaternion& v2);

	RNQuaternion& operator*=(const float& v);

	RNQuaternion operator/(const float& v);

	RNQuaternion& operator/=(const float& v);

	bool operator==(const RNQuaternion& v) const;
	bool operator!=(const RNQuaternion& v) const;

	float getScalar() const;
	RNVector3 getAxis() const;
	float getX() const;
	float getY() const;
	float getZ() const;

	float dot(const RNQuaternion& v) const;
	float length() const;
	float length2() const;

	float angle(const RNQuaternion& q) const;
	float angleShortestPath(const RNQuaternion& q) const;

	RNQuaternion& normalize();
	RNQuaternion slerp(const RNQuaternion& q, const float& t) const;

	static const RNQuaternion& eye();

	void setScalar(float scalar);
	RNVector3 setAxis(RNVector3 axis);

};

#endif

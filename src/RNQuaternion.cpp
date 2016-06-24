#include "RNQuaternion.h"

RNQuaternion::RNQuaternion(const float scalar, const RNVector3 axis){
	this->scalar = scalar;
	this->axis = axis;
}

RNQuaternion RNQuaternion::initFromEuler(const float yaw, const float pitch, const float roll){
	float halfYaw = yaw * 0.5;  
	float halfPitch = pitch * 0.5;
	float halfRoll = roll * 0.5;
	float cosYaw = std::cos(halfYaw);
	float sinYaw = std::sin(halfYaw);
	float cosPitch = std::cos(halfPitch);
	float sinPitch = std::sin(halfPitch);
	float cosRoll = std::cos(halfRoll);
	float sinRoll = std::sin(halfRoll);
	return RNQuaternion(cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw,
						RNVector3(cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw,
								  cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw,
								  sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw));
}

RNQuaternion RNQuaternion::initFromRPY(const float roll, const float pitch, const float yaw){
	float halfYaw = yaw * 0.5;  
	float halfPitch = pitch * 0.5;
	float halfRoll = roll * 0.5;
	float cosYaw = std::cos(halfYaw);
	float sinYaw = std::sin(halfYaw);
	float cosPitch = std::cos(halfPitch);
	float sinPitch = std::sin(halfPitch);
	float cosRoll = std::cos(halfRoll);
	float sinRoll = std::sin(halfRoll);
	return RNQuaternion(cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw,
						RNVector3(sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw,
								  cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw,
								  cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw));
}

void RNQuaternion::operator=(const RNQuaternion& v){
	scalar = v.scalar;
	axis = v.axis;
}

float RNQuaternion::getScalar() const{
	return this->scalar;
}

RNVector3 RNQuaternion::getAxis() const{
	return this->axis;
}

float RNQuaternion::getX() const{
	return this->axis.getX();
}

float RNQuaternion::getY() const{
	return this->axis.getY();
}

float RNQuaternion::getZ() const{
	return this->axis.getZ();
}

void RNQuaternion::setScalar(float scalar){
	this->scalar = scalar;
}

RNVector3 RNQuaternion::setAxis(RNVector3 axis){
	this->axis = axis;
}

RNQuaternion RNQuaternion::operator+(const RNQuaternion& v){
	return RNQuaternion(this->scalar + v.scalar, this->axis + v.axis);
}

RNQuaternion RNQuaternion::operator+(const float& v){
	return RNQuaternion(this->scalar + v, this->axis);
}

RNQuaternion operator+(const RNQuaternion& v1, const RNQuaternion& v2){
	return RNQuaternion(v1.scalar + v2.scalar, v1.axis + v2.axis);
}

RNQuaternion& RNQuaternion::operator+=(const RNQuaternion& v){
	scalar += v.scalar;
	axis += v.axis;
	return *this;
}

RNQuaternion& RNQuaternion::operator+=(const float& v){
	scalar += v;
	axis += RNVector3();
	return *this;
}
RNQuaternion RNQuaternion::operator-() const{
	return RNQuaternion(-this->scalar, -this->axis);
}

RNQuaternion RNQuaternion::operator-(const RNQuaternion& v){
	return RNQuaternion(this->scalar - v.scalar, this->axis - v.axis);
}

RNQuaternion RNQuaternion::operator-(const float& v){
	return RNQuaternion(this->scalar - v, this->axis);
}

RNQuaternion operator-(const RNQuaternion& v1, const RNQuaternion& v2){
	return RNQuaternion(v1.scalar - v2.scalar, v1.axis - v2.axis);
}

RNQuaternion& RNQuaternion::operator-=(const RNQuaternion& v){
	scalar -= v.scalar;
	axis -= v.axis;
	return *this;
}

RNQuaternion& RNQuaternion::operator-=(const float& v){
	scalar -= v;
	axis -= RNVector3();
	return *this;
}

RNQuaternion RNQuaternion::operator*(const RNQuaternion& v){
	RNQuaternion result(this->scalar * v.scalar - this->axis.dot(v.axis));
	result.setAxis(RNVector3(this->scalar * v.getX() + this->getX() * v.scalar + this->getY() * v.getZ() - this->getZ() * v.getY(),
	 						 this->scalar * v.getY() + this->getX() * v.getZ() + this->getY() * v.scalar - this->getZ() * v.getX(), 
	 						 this->scalar * v.getZ() + this->getX() * v.getY() + this->getY() * v.getX() - this->getZ() * v.scalar));
	return result;
}

RNQuaternion RNQuaternion::operator*(const float& v){
	return RNQuaternion(this->scalar * v, this->axis * v);
}

RNQuaternion operator*(const RNQuaternion& v1, const RNQuaternion& v2){
	RNQuaternion result(v1.scalar * v2.scalar - v1.axis.dot(v2.axis));
	result.setAxis(RNVector3(v1.scalar * v2.getX() + v1.getX() * v2.scalar + v1.getY() * v2.getZ() - v1.getZ() * v2.getY(),
	 						 v1.scalar * v2.getY() + v1.getX() * v2.getZ() + v1.getY() * v2.scalar - v1.getZ() * v2.getX(), 
	 						 v1.scalar * v2.getZ() + v1.getX() * v2.getY() + v1.getY() * v2.getX() - v1.getZ() * v2.scalar));
	return result;
}

RNQuaternion& RNQuaternion::operator*=(const float& v){
	this->scalar *= v;
	this->axis *= v;
	return *this;
}

RNQuaternion RNQuaternion::operator/(const float& v){
	return *this * (1.0 / v);
}

RNQuaternion& RNQuaternion::operator/=(const float& v){
	return *this *= (1.0 / v);
}

bool RNQuaternion::operator==(const RNQuaternion& v) const{
	return (this->scalar == v.scalar and this->axis == v.axis);
}

bool RNQuaternion::operator!=(const RNQuaternion& v) const{
	return not (*this == v);
}

float RNQuaternion::dot(const RNQuaternion& v) const{
	return this->scalar * v.scalar + this->axis.dot(v.axis);
}

float RNQuaternion::length() const{
	return std::sqrt(length2());
}

float RNQuaternion::length2() const{
	return dot(*this);
}

RNQuaternion& RNQuaternion::normalize(){
	return *this /= length();
}

float RNQuaternion::angle(const RNQuaternion& q) const{
	float s = std::sqrt(length2() * q.length2());
	return std::acos(dot(q) / s);
}

float RNQuaternion::angleShortestPath(const RNQuaternion& q) const{
	float s = std::sqrt(length2() * q.length2());
	float result = 0;
	if(dot(q) < 0){
		result = std::acos(dot(-q) / s) * 2.0;
	} else {
		result = std::acos(dot(q) / s) * 2.0;
	}
	return result;
}

RNQuaternion RNQuaternion::slerp(const RNQuaternion& q, const float& t) const{
	float theta = angleShortestPath(q) / 2.0;
	if(theta != 0.0){
		float d = 1.0 / std::sin(theta);
		float s0 = std::sin((1.0 - t) * theta);
		float s1 = std::sin(t * theta);
		RNQuaternion result;
		if(dot(q) < 0){
			result = RNQuaternion((this->scalar * s0 + -q.getScalar() * s1) * d,
								RNVector3((this->getX() * s0 + -q.getX() * s1) * d,
										  (this->getY() * s0 + -q.getY() * s1) * d,
										  (this->getY() * s0 + -q.getZ() * s1) * d));
		} else {
			result = RNQuaternion((this->scalar * s0 + q.getScalar() * s1) * d,
								RNVector3((this->getX() * s0 + q.getX() * s1) * d,
										  (this->getY() * s0 + q.getY() * s1) * d,
										  (this->getY() * s0 + q.getZ() * s1) * d));
		}
		return result;
	} else {
		return *this;
	}
}

const RNQuaternion& RNQuaternion::eye(){
	static const RNQuaternion result(1);
	return result;
}

const char* RNQuaternion::toString() const{
	char* buffer = new char[255];
	std::sprintf(buffer, "{scalar: %0.6f, vector: %s}", scalar, axis.toString());
	return buffer;
}
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

RNQuaternion::RNQuaternion (const float scalar, const float a, const float b,const float c){
	RNVector3 R (a,b,c);
	RNQuaternion (scalar,R);
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

void RNQuaternion::setAxis(RNVector3 axis){
	this->axis = axis;
}

RNQuaternion RNQuaternion::operator+(const RNQuaternion& v){
	return RNQuaternion(this->scalar + v.scalar, this->axis + v.axis);
}

RNQuaternion RNQuaternion::operator+(const float& v){
	return RNQuaternion(this->scalar + v, this->axis);
}
//NC
RNQuaternion operator+(const RNQuaternion& v1, const RNQuaternion& v2){
	printf("el de dos param\n");
	return RNQuaternion(v1.scalar + v2.scalar, v1.axis + v2.axis);
}

RNQuaternion& RNQuaternion::operator+=(const RNQuaternion& v){
	scalar += v.scalar;
	axis += v.axis;
	return *this;
}
//No lo hace así, solo suma la parte escalar
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
//NC
RNQuaternion operator-(const RNQuaternion& v1, const RNQuaternion& v2){
	return RNQuaternion(v1.scalar - v2.scalar, v1.axis - v2.axis);
}

RNQuaternion& RNQuaternion::operator-=(const RNQuaternion& v){
	scalar -= v.scalar;
	axis -= v.axis;
	return *this;
}
//No lo hace así, solo resta la parte escalar
RNQuaternion& RNQuaternion::operator-=(const float& v){
	scalar -= v;
	axis -= RNVector3();
	return *this;
}
//modified
RNQuaternion RNQuaternion::operator*(const RNQuaternion& v){
	RNQuaternion result;
	result.setScalar(this->scalar * v.scalar - this->axis.dot(v.axis));
	result.setAxis (this->axis.cross(v.axis)+v.scalar*this->axis + this->scalar*v.axis);
	return result;
}

RNQuaternion RNQuaternion::operator*(const float& v){
	return RNQuaternion(this->scalar * v, this->axis * v);
}
//modificado
RNQuaternion operator*(const RNQuaternion& v1, const RNQuaternion& v2){
	RNQuaternion result;
	result.setScalar(v1.scalar * v2.scalar - v1.axis.dot(v2.axis));
	result.setAxis (v1.axis.cross(v2.axis)+v2.scalar*v1.axis + v1.scalar*v2.axis);
	return result;
}
//DIFERENCIA ENTRE LOS DOS OPERATOR*????


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
//NC
RNQuaternion operator!(RNQuaternion m){
	return m.inverse();
}

bool RNQuaternion::operator==(const RNQuaternion& v) const{
	return (this->scalar == v.scalar and this->axis == v.axis);
}

bool RNQuaternion::operator!=(const RNQuaternion& v) const{
	return not (*this == v);
}

bool RNQuaternion::isZero(){
	return (this->scalar == 0.0 and this->axis.isZero());
}

float RNQuaternion::dot(const RNQuaternion& v) const{
	return this->scalar * v.scalar + this->axis.dot(v.axis);
}
//NC//NC
float RNQuaternion::length() const{
	return std::sqrt(length2());
}
//NC
float RNQuaternion::length2() const{
	return dot(*this);
}
//NC
RNQuaternion RNQuaternion::norm(){
	return *this / length();
}
//NC
RNQuaternion RNQuaternion::inverse(){
	return (this->conjugate() / length2());
}

RNQuaternion RNQuaternion::conjugate(){
	return RNQuaternion(this->scalar, -this->axis);
}
//NC
float RNQuaternion::angleBetween(const RNQuaternion& q) const{
	float s = std::sqrt(length2() * q.length2());
	return std::acos(dot(q) / s);
}
//NC
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
//gestionar los 180º


RNQuaternion RNQuaternion::slerp(const RNQuaternion& q, const float& t) const{
	float theta = angleShortestPath(q) / 2.0;
	if(theta != 0.0){
		float d = 1.0 / std::sin(theta);
		float s0 = std::sin((1.0 - t) * theta);
		float s1 = std::sin(t * theta);
		RNQuaternion result;

		//y si sintheta es muy pequeño?? mirar abajo
		if(dot(q) < 0){
			result = RNQuaternion((this->scalar * s0 + -q.getScalar() * s1) * d,
								RNVector3((this->getX() * s0 + -q.getX() * s1) * d,
										  (this->getY() * s0 + -q.getY() * s1) * d,
										  (this->getZ() * s0 + -q.getZ() * s1) * d));
		} else {
			result = RNQuaternion((this->scalar * s0 + q.getScalar() * s1) * d,
								RNVector3((this->getX() * s0 + q.getX() * s1) * d,
										  (this->getY() * s0 + q.getY() * s1) * d,
										  (this->getZ() * s0 + q.getZ() * s1) * d));
		}
		return result;
	} else {
		return *this;
	}
}
//no podemos dividir entre cero --qm es el quaternio medio
	/*
	if(fabs(sinHalfTheta)<0.001){	
		qm.setScalar= (qa.w * 0.5 + qb.w * 0.5);
		RNVector3 vm((qa.x * 0.5 + qb.x * 0.5),(qa.y * 0.5 + qb.y * 0.5),(qa.z * 0.5 + qb.z * 0.5));	
		qm.setAxis(vm);
	}

*/
//NC-- http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/slerp/index.htm

//interpolacion slerp entre cuaternios más rápida
/*
RNQuaternion RNQuaternion::slerpQuat (const RNQuaternion& q, const float& t){
	RNQuaternion R;
	R=this;
	R.inverse();
	R*=q;
	R.pow(t);
	return R;
}
*/

//NC
const RNQuaternion& RNQuaternion::eye(){
	static const RNQuaternion result(1);
	return result;
}
//NC
RNQuaternion RNQuaternion::exp(){
	return RNQuaternion(std::exp(this->scalar) * std::cos(this->axis.length()), std::exp(this->scalar) * (this->axis.sign() * std::sin(this->axis.length())));
}
//NC
RNQuaternion RNQuaternion::ln(){
	return RNQuaternion(log2(this->length()), (this->axis.sign() * std::acos(this->scalar / this->length())));
}
//NC
RNQuaternion RNQuaternion::pow(float n){
	return (n * this->ln()).exp();
}
//NC
RNQuaternion RNQuaternion::sign(){
	RNQuaternion result;
	if(not isZero()){
		result = *this / (length());
	}
	return result;
}

const char* RNQuaternion::toString() const{
	char* buffer = new char[255];
	std::sprintf(buffer, "{scalar: %0.6f, vector: %s}", scalar, axis.toString());
	return buffer;
}


//mejor a angulo o a pasos?
float RNQuaternion::toAngle(){
	return acos(this->scalar)*360/M_PI; //acos(this->scalar)*2*360/(2*PI)
}